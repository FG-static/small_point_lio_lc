#include "small_point_lio_pgo/small_point_lio_keyframe_bridge.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <exception>
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

        // 桥接节点先同步里程计与注册点云，再按时间窗口整理成后端候选。
        // Point-LIO 前端仍按原始频率运行，不会因为后端累积而降频。
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
            "odom_frame=%s body_frame=%s max_delay=%.3fs buffer=%.3fs "
            "accumulation=%.1fms max_clouds=%d",
            odom_topic_.c_str(),
            cloud_topic_.c_str(),
            keyframe_topic_.c_str(),
            odom_frame_.c_str(),
            body_frame_.c_str(),
            max_cloud_delay_sec_,
            odom_buffer_duration_sec_,
            accumulation_time_ms_,
            max_accumulated_clouds_
        );
    }

    SmallPointLioKeyframeBridge::~SmallPointLioKeyframeBridge() {

        // 与 small_dlio 累积器一致，节点正常退出时尽量发布最后一个
        // 尚未达到完整时间窗的有效点云窗口。
        if (!rclcpp::ok() || !accumulation_active_)
            return;

        try {

            publishAccumulatedCandidate();
        } catch (const std::exception &error) {

            RCLCPP_WARN(
                get_logger(),
                "Failed to publish the final accumulated cloud: %s",
                error.what()
            );
        } catch (...) {

            RCLCPP_WARN(
                get_logger(),
                "Failed to publish the final accumulated cloud"
            );
        }
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
        declare_parameter("accumulation_time_ms", accumulation_time_ms_);
        declare_parameter(
            "max_accumulated_clouds", max_accumulated_clouds_
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
        get_parameter("accumulation_time_ms", accumulation_time_ms_);
        get_parameter(
            "max_accumulated_clouds", max_accumulated_clouds_
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
        if (!std::isfinite(accumulation_time_ms_) ||
            accumulation_time_ms_ < 0.0)
            accumulation_time_ms_ = 100.0;
        accumulation_time_ns_ = static_cast<std::int64_t>(
            accumulation_time_ms_ * 1e6
        );
        max_accumulated_clouds_ = std::max(1, max_accumulated_clouds_);
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

            resetAccumulation();
            RCLCPP_WARN(
                get_logger(),
                "Odometry time moved backwards; reset synchronization and "
                "cloud accumulation buffers"
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

        if (accumulation_time_ns_ <= 0) {

            // 设为 0 时保留旧行为，便于与未累积版本做 A/B 对照。
            publishCandidate(
                odom.pose.pose,
                msg->header.stamp,
                {msg},
                delay_sec,
                0.0
            );
            return;
        }

        accumulateCloud(msg, odom, delay_sec);
    }

    void SmallPointLioKeyframeBridge::accumulateCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
        const nav_msgs::msg::Odometry &odom,
        double odom_delay_sec
    ) {

        const std::int64_t stamp_ns = stampToNanoseconds(msg->header.stamp);
        if (accumulation_active_ &&
            stamp_ns < accumulation_last_stamp_ns_) {

            // rosbag 跳时或前端时间源重置后，旧窗口不能与新时间段混合。
            resetAccumulation();
            RCLCPP_WARN(
                get_logger(),
                "Cloud time moved backwards; reset cloud accumulation buffer"
            );
        }

        if (accumulation_active_ &&
            stamp_ns - accumulation_window_start_ns_ >=
                accumulation_time_ns_) {

            // 与 small_dlio 的累积器相同：越过窗口边界的当前包不放进
            // 旧窗口，而是先发布旧窗口，再由当前包开启新窗口。
            publishAccumulatedCandidate();
        }

        if (!accumulation_active_) {

            accumulation_window_start_ns_ = stamp_ns;
            accumulation_active_ = true;
        }

        // /cloud_registered 已处于 odom 系，因此窗口内可直接保存并合并；
        // 每包对应的 body 坐标系不同，不能逐包转到 body 后直接拼接。
        accumulated_clouds_.push_back(msg);
        accumulation_anchor_odom_ = odom;
        accumulation_anchor_delay_sec_ = odom_delay_sec;
        accumulation_last_stamp_ns_ = stamp_ns;

        if (accumulated_clouds_.size() >=
            static_cast<std::size_t>(max_accumulated_clouds_)) {

            RCLCPP_WARN(
                get_logger(),
                "Accumulation reached the safety limit of %d clouds; "
                "publishing the current window",
                max_accumulated_clouds_
            );
            publishAccumulatedCandidate();
        }
    }

    void SmallPointLioKeyframeBridge::publishAccumulatedCandidate() {

        if (!accumulation_active_ || accumulated_clouds_.empty()) {

            resetAccumulation();
            return;
        }

        const double window_span_ms = static_cast<double>(
            accumulation_last_stamp_ns_ - accumulation_window_start_ns_
        ) * 1e-6;
        const auto stamp = accumulated_clouds_.back()->header.stamp;

        // 末帧里程计是 T_odom_body_end。窗口内所有点仍在统一的 odom
        // 系，publishCandidate() 会用其逆变换一次性转到末帧 body 系。
        publishCandidate(
            accumulation_anchor_odom_.pose.pose,
            stamp,
            accumulated_clouds_,
            accumulation_anchor_delay_sec_,
            window_span_ms
        );
        resetAccumulation();
    }

    void SmallPointLioKeyframeBridge::resetAccumulation() {

        accumulated_clouds_.clear();
        accumulation_window_start_ns_ = 0;
        accumulation_last_stamp_ns_ = 0;
        accumulation_anchor_delay_sec_ = 0.0;
        accumulation_active_ = false;
    }

    void SmallPointLioKeyframeBridge::publishCandidate(
        const geometry_msgs::msg::Pose &pose,
        const builtin_interfaces::msg::Time &stamp,
        const std::vector<
            sensor_msgs::msg::PointCloud2::ConstSharedPtr
        > &clouds,
        double odom_delay_sec,
        double window_span_ms
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
        std::size_t expected_points = 0;
        for (const auto &cloud : clouds) {

            if (cloud)
                expected_points += pointCount(*cloud);
        }
        cloud_odom.reserve(expected_points);
        for (const auto &cloud : clouds) {

            if (!cloud)
                continue;

            pcl::PointCloud<pcl::PointXYZ> source_cloud;
            pcl::fromROSMsg(*cloud, source_cloud);
            cloud_odom += source_cloud;
        }
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

        // 一个时间窗口对应一个候选；关键帧筛选和全局地图累积仍由后端完成。
        small_point_lio_pgo::msg::KeyFrame keyframe;
        keyframe.header.stamp = stamp;
        keyframe.header.frame_id = odom_frame_;
        keyframe.id = next_candidate_id_++;
        keyframe.pose = pose;
        keyframe.pose.orientation.w = q_odom_body.w();
        keyframe.pose.orientation.x = q_odom_body.x();
        keyframe.pose.orientation.y = q_odom_body.y();
        keyframe.pose.orientation.z = q_odom_body.z();
        keyframe.alignment_score = 0.0;
        pcl::toROSMsg(cloud_body, keyframe.cloud);
        keyframe.cloud.header.stamp = stamp;
        keyframe.cloud.header.frame_id = body_frame_;

        pub_keyframe_->publish(keyframe);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Published keyframe candidate: id=%u clouds=%zu "
            "window_span=%.1fms points=%zu pose=[%.3f %.3f %.3f] "
            "odom_delay=%.4fs",
            keyframe.id,
            clouds.size(),
            window_span_ms,
            cloud_body.size(),
            pose.position.x,
            pose.position.y,
            pose.position.z,
            odom_delay_sec
        );
    }

}  // namespace small_point_lio_pgo
