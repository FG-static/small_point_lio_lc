#include "small_point_lio_pgo/small_point_lio_keyframe_bridge.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <utility>
#include <vector>

namespace small_point_lio_pgo {

    SmallPointLioKeyframeBridge::SmallPointLioKeyframeBridge()
        : Node("small_point_lio_keyframe_bridge") {

        loadParams();

        // 桥接节点只同步里程计与注册点云，并完成坐标系转换；
        // 每条点云消息独立生成候选，不在这里做跨帧点云累积。
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            rclcpp::QoS(100).reliable(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                callbackOdom(std::move(msg));
            }
        );
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_,
            rclcpp::QoS(20).reliable(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                callbackCloud(std::move(msg));
            }
        );
        pub_keyframe_ = create_publisher<small_point_lio_pgo::msg::KeyFrame>(
            keyframe_topic_, rclcpp::QoS(100).reliable()
        );

        RCLCPP_INFO(
            get_logger(),
            "Small Point-LIO bridge: odom=%s cloud=%s keyframe=%s "
            "odom_frame=%s body_frame=%s max_delay=%.3fs buffer=%.3fs",
            odom_topic_.c_str(),
            cloud_topic_.c_str(),
            keyframe_topic_.c_str(),
            odom_frame_.c_str(),
            body_frame_.c_str(),
            max_cloud_delay_sec_,
            odom_buffer_duration_sec_
        );
    }

    void SmallPointLioKeyframeBridge::loadParams() {

        declare_parameter("odom_topic", odom_topic_);
        declare_parameter("cloud_topic", cloud_topic_);
        declare_parameter("keyframe_topic", keyframe_topic_);
        declare_parameter("odom_frame", odom_frame_);
        declare_parameter("body_frame", body_frame_);
        declare_parameter("max_cloud_delay_sec", max_cloud_delay_sec_);
        declare_parameter(
            "odom_buffer_duration_sec", odom_buffer_duration_sec_
        );
        declare_parameter("min_cloud_points", min_cloud_points_);

        get_parameter("odom_topic", odom_topic_);
        get_parameter("cloud_topic", cloud_topic_);
        get_parameter("keyframe_topic", keyframe_topic_);
        get_parameter("odom_frame", odom_frame_);
        get_parameter("body_frame", body_frame_);
        get_parameter("max_cloud_delay_sec", max_cloud_delay_sec_);
        get_parameter(
            "odom_buffer_duration_sec", odom_buffer_duration_sec_
        );
        get_parameter("min_cloud_points", min_cloud_points_);

        if (odom_topic_.empty())
            odom_topic_ = "/Odometry";
        if (cloud_topic_.empty())
            cloud_topic_ = "/cloud_registered";
        if (keyframe_topic_.empty())
            keyframe_topic_ = "/keyframe_candidates";
        if (odom_frame_.empty())
            odom_frame_ = "odom";
        if (body_frame_.empty())
            body_frame_ = "base_link";
        if (!std::isfinite(max_cloud_delay_sec_))
            max_cloud_delay_sec_ = 0.05;
        if (!std::isfinite(odom_buffer_duration_sec_) ||
            odom_buffer_duration_sec_ <= 0.0)
            odom_buffer_duration_sec_ = 1.0;
        min_cloud_points_ = std::max(1, min_cloud_points_);
    }

    std::int64_t SmallPointLioKeyframeBridge::stampToNanoseconds(
        const builtin_interfaces::msg::Time &stamp
    ) {

        return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
            static_cast<std::int64_t>(stamp.nanosec);
    }

    bool SmallPointLioKeyframeBridge::poseFinite(
        const geometry_msgs::msg::Pose &pose
    ) {

        if (!std::isfinite(pose.position.x) ||
            !std::isfinite(pose.position.y) ||
            !std::isfinite(pose.position.z) ||
            !std::isfinite(pose.orientation.w) ||
            !std::isfinite(pose.orientation.x) ||
            !std::isfinite(pose.orientation.y) ||
            !std::isfinite(pose.orientation.z))
            return false;

        const Eigen::Quaterniond q(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );
        return q.norm() > 1e-9;
    }

    std::size_t SmallPointLioKeyframeBridge::pointCount(
        const sensor_msgs::msg::PointCloud2 &cloud
    ) {

        return static_cast<std::size_t>(cloud.width) *
            static_cast<std::size_t>(cloud.height);
    }

    void SmallPointLioKeyframeBridge::callbackOdom(
        nav_msgs::msg::Odometry::ConstSharedPtr msg
    ) {

        if (!msg || !poseFinite(msg->pose.pose)) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring invalid Small Point-LIO odometry"
            );
            return;
        }
        if (msg->header.frame_id != odom_frame_) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring odometry in frame '%s'; expected '%s'",
                msg->header.frame_id.c_str(),
                odom_frame_.c_str()
            );
            return;
        }
        if (!msg->child_frame_id.empty() &&
            msg->child_frame_id != body_frame_) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring odometry child frame '%s'; expected '%s'",
                msg->child_frame_id.c_str(),
                body_frame_.c_str()
            );
            return;
        }

        const std::int64_t stamp_ns = stampToNanoseconds(msg->header.stamp);
        bool time_reset = false;
        std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr>
            ready_clouds;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);

            if (has_last_odom_stamp_ && stamp_ns < last_odom_stamp_ns_) {

                // rosbag 回放跳时等情况会使旧缓存失效，避免跨时间段误匹配。
                odom_buffer_.clear();
                pending_clouds_.clear();
                time_reset = true;
            }

            // 缓存短时间窗内的里程计，供点云按时间戳查找最近位姿。
            odom_buffer_.push_back(*msg);
            last_odom_stamp_ns_ = stamp_ns;
            has_last_odom_stamp_ = true;

            const double newest_sec = static_cast<double>(stamp_ns) * 1e-9;
            while (!odom_buffer_.empty()) {

                const double oldest_sec = static_cast<double>(
                    stampToNanoseconds(odom_buffer_.front().header.stamp)
                ) * 1e-9;
                if (newest_sec - oldest_sec <= odom_buffer_duration_sec_)
                    break;
                odom_buffer_.pop_front();
            }

            auto cloud_it = pending_clouds_.begin();
            while (cloud_it != pending_clouds_.end()) {

                const std::int64_t cloud_stamp_ns = stampToNanoseconds(
                    (*cloud_it)->header.stamp
                );
                // 已有不早于点云的里程计后，才交给最近时间戳匹配处理。
                if (cloud_stamp_ns <= stamp_ns) {

                    ready_clouds.push_back(std::move(*cloud_it));
                    cloud_it = pending_clouds_.erase(cloud_it);
                } else {

                    ++cloud_it;
                }
            }
        }

        if (time_reset) {

            RCLCPP_WARN(
                get_logger(),
                "Odometry time moved backwards; reset the bridge buffer"
            );
        }

        // 两个 topic 的回调到达顺序不确定；等里程计到达后再处理暂存点云，
        // 后续仍会用 max_cloud_delay_sec_ 限制最终匹配误差。
        for (const auto &cloud : ready_clouds)
            processCloud(cloud);
    }

    bool SmallPointLioKeyframeBridge::findNearestOdom(
        const builtin_interfaces::msg::Time &stamp,
        nav_msgs::msg::Odometry &odom,
        double &delay_sec
    ) {

        const std::int64_t cloud_stamp_ns = stampToNanoseconds(stamp);
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_buffer_.empty())
            return false;

        const auto delay = [cloud_stamp_ns](
            const nav_msgs::msg::Odometry &candidate
        ) {

            const auto odom_stamp_ns = stampToNanoseconds(
                candidate.header.stamp
            );
            return std::abs(
                static_cast<double>(cloud_stamp_ns - odom_stamp_ns)
            );
        };

        auto nearest = odom_buffer_.begin();
        double nearest_delay_ns = delay(*nearest);
        for (auto it = std::next(nearest); it != odom_buffer_.end(); ++ it) {

            const double candidate_delay_ns = delay(*it);
            if (candidate_delay_ns < nearest_delay_ns) {

                nearest = it;
                nearest_delay_ns = candidate_delay_ns;
            }
        }

        odom = *nearest;
        delay_sec = nearest_delay_ns * 1e-9;
        return true;
    }

    void SmallPointLioKeyframeBridge::callbackCloud(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr msg
    ) {

        if (!msg || msg->data.empty() ||
            pointCount(*msg) < static_cast<std::size_t>(min_cloud_points_)) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring empty or undersized Small Point-LIO cloud"
            );
            return;
        }
        if (msg->header.frame_id != odom_frame_) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring cloud in frame '%s'; expected '%s'",
                msg->header.frame_id.c_str(),
                odom_frame_.c_str()
            );
            return;
        }

        const std::int64_t cloud_stamp_ns = stampToNanoseconds(
            msg->header.stamp
        );
        bool wait_for_odom = false;
        std::size_t dropped_pending_clouds = 0;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            wait_for_odom = odom_buffer_.empty() ||
                cloud_stamp_ns > last_odom_stamp_ns_;
            if (wait_for_odom) {

                // 这里只是等待同时间段的里程计，不会把队列中的点云合并。
                pending_clouds_.push_back(msg);
                const double newest_sec =
                    static_cast<double>(cloud_stamp_ns) * 1e-9;
                while (!pending_clouds_.empty()) {

                    const double oldest_sec = static_cast<double>(
                        stampToNanoseconds(
                            pending_clouds_.front()->header.stamp
                        )
                    ) * 1e-9;
                    if (newest_sec - oldest_sec <=
                        odom_buffer_duration_sec_)
                        break;
                    pending_clouds_.pop_front();
                    ++ dropped_pending_clouds;
                }
                while (pending_clouds_.size() > 100U) {

                    pending_clouds_.pop_front();
                    ++ dropped_pending_clouds;
                }
            }
        }

        if (dropped_pending_clouds > 0U) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropped %zu point clouds while waiting for odometry",
                dropped_pending_clouds
            );
        }
        if (wait_for_odom)
            return;

        processCloud(msg);
    }

    void SmallPointLioKeyframeBridge::processCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg
    ) {

        nav_msgs::msg::Odometry odom;
        double delay_sec = std::numeric_limits<double>::infinity();
        // 为当前点云选择时间戳最近的里程计，并限制允许的最大时间差。
        if (!findNearestOdom(msg->header.stamp, odom, delay_sec)) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for Small Point-LIO odometry on %s",
                odom_topic_.c_str()
            );
            return;
        }
        if (max_cloud_delay_sec_ >= 0.0 &&
            delay_sec > max_cloud_delay_sec_) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skipping cloud: nearest odom delay %.4fs exceeds %.4fs",
                delay_sec,
                max_cloud_delay_sec_
            );
            return;
        }

        publishCandidate(odom.pose.pose, *msg, delay_sec);
    }

    void SmallPointLioKeyframeBridge::publishCandidate(
        const geometry_msgs::msg::Pose &pose,
        const sensor_msgs::msg::PointCloud2 &cloud,
        double odom_delay_sec
    ) {

        Eigen::Quaterniond q_odom_body(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );
        if (q_odom_body.norm() <= 1e-9 || !q_odom_body.coeffs().allFinite())
            return;
        q_odom_body.normalize();

        Eigen::Isometry3d T_odom_body = Eigen::Isometry3d::Identity();
        T_odom_body.linear() = q_odom_body.toRotationMatrix();
        T_odom_body.translation() = Eigen::Vector3d(
            pose.position.x,
            pose.position.y,
            pose.position.z
        );

        pcl::PointCloud<pcl::PointXYZ> cloud_odom;
        pcl::fromROSMsg(cloud, cloud_odom);
        if (cloud_odom.empty())
            return;

        // pose 表示 T_odom_body，而 Point-LIO 的注册点云位于 odom 系；
        // 使用其逆变换得到当前 body 系局部点云，供后端关键帧使用。
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(
            cloud_odom,
            transformed_cloud,
            T_odom_body.inverse().matrix().cast<float>()
        );

        pcl::PointCloud<pcl::PointXYZ> cloud_body;
        cloud_body.reserve(transformed_cloud.size());
        for (const auto &point : transformed_cloud.points) {

            if (std::isfinite(point.x) &&
                std::isfinite(point.y) &&
                std::isfinite(point.z))
                cloud_body.push_back(point);
        }
        cloud_body.width = static_cast<std::uint32_t>(cloud_body.size());
        cloud_body.height = 1;
        cloud_body.is_dense = true;
        if (cloud_body.size() < static_cast<std::size_t>(min_cloud_points_)) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skipping cloud with too few finite transformed points"
            );
            return;
        }

        // 一帧输入点云对应一个候选；关键帧筛选和全局地图累积由后端节点完成。
        small_point_lio_pgo::msg::KeyFrame keyframe;
        keyframe.header.stamp = cloud.header.stamp;
        keyframe.header.frame_id = odom_frame_;
        keyframe.id = next_candidate_id_++;
        keyframe.pose = pose;
        keyframe.pose.orientation.w = q_odom_body.w();
        keyframe.pose.orientation.x = q_odom_body.x();
        keyframe.pose.orientation.y = q_odom_body.y();
        keyframe.pose.orientation.z = q_odom_body.z();
        keyframe.alignment_score = 0.0;
        pcl::toROSMsg(cloud_body, keyframe.cloud);
        keyframe.cloud.header.stamp = cloud.header.stamp;
        keyframe.cloud.header.frame_id = body_frame_;

        pub_keyframe_->publish(keyframe);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Published keyframe candidate: id=%u points=%zu "
            "pose=[%.3f %.3f %.3f] odom_delay=%.4fs",
            keyframe.id,
            cloud_body.size(),
            pose.position.x,
            pose.position.y,
            pose.position.z,
            odom_delay_sec
        );
    }

}  // namespace small_point_lio_pgo
