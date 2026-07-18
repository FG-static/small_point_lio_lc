#include "small_point_lio_pgo/map_node.hpp"

#include "small_point_lio_pgo/msg/key_frame.hpp"
#include "small_point_lio_pgo/msg/optimized_key_frames.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <utility>
#include <vector>

namespace small_point_lio_pgo {

    MapNode::MapNode() : Node("map_node") {

        loadParams();

        global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        // map_node 是纯重建节点：订阅原始关键帧和优化关键帧，
        // 每次状态更新后重新拼接并发布 /global_map。
        pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            map_topic_, rclcpp::QoS(1).reliable().transient_local()
        );

        srv_save_map_ = create_service<std_srvs::srv::Trigger>(
            save_map_service_name_,
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response
            ) {

                callbackSaveMap(request, response);
            }
        );

        sub_keyframe_ =
            create_subscription<small_point_lio_pgo::msg::KeyFrame>(
                keyframe_topic_, rclcpp::QoS(100),
                [this](small_point_lio_pgo::msg::KeyFrame::ConstSharedPtr msg) {
                    callbackKeyFrameMsg(msg);
                }
            );
        sub_optimized_keyframes_ =
            create_subscription<small_point_lio_pgo::msg::OptimizedKeyFrames>(
                optimized_keyframes_topic_,
                rclcpp::QoS(1).reliable().transient_local(),
                [this](small_point_lio_pgo::msg::OptimizedKeyFrames::ConstSharedPtr msg) {
                    callbackOptimizedKeyFrames(msg);
                }
            );
    }

    void MapNode::loadParams() {

        // 地图重建参数包括输出话题/保存路径、body-lidar 外参，
        // 以及 PGO 后尾部未优化关键帧的处理策略。
        this->declare_parameter("map_leaf_size", 0.1);
        this->declare_parameter("map_topic", "/global_map");
        this->declare_parameter("map_frame", map_frame_);
        this->declare_parameter("odom_frame", odom_frame_);
        this->declare_parameter("keyframe_topic", "/keyframe_msg");
        this->declare_parameter("save_map_service_name", "/save_map");
        this->declare_parameter("map_save_path", "/home/goose/small_point_lio_lc/global_map.pcd");
        this->declare_parameter("optimized_keyframes_topic", "/optimized_keyframes");
        this->declare_parameter("body_frame", body_frame_);
        this->declare_parameter("lidar_frame", lidar_frame_);
        this->declare_parameter(
            "rebuild_unoptimized_keyframes_with_approximation",
            rebuild_unoptimized_keyframes_with_approximation_
        );
        this->declare_parameter(
            "apply_pgo_correction_to_unoptimized_keyframes",
            apply_pgo_correction_to_unoptimized_keyframes_
        );
        this->declare_parameter(
            "optimized_keyframes_tail_exclusion_count",
            optimized_keyframes_tail_exclusion_count_
        );

        this->get_parameter("map_leaf_size", map_leaf_size_);
        this->get_parameter("map_topic", map_topic_);
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("keyframe_topic", keyframe_topic_);
        this->get_parameter("save_map_service_name", save_map_service_name_);
        this->get_parameter("map_save_path", map_save_path_);
        this->get_parameter("optimized_keyframes_topic", optimized_keyframes_topic_);
        this->get_parameter("body_frame", body_frame_);
        this->get_parameter("lidar_frame", lidar_frame_);
        this->get_parameter(
            "rebuild_unoptimized_keyframes_with_approximation",
            rebuild_unoptimized_keyframes_with_approximation_
        );
        this->get_parameter(
            "apply_pgo_correction_to_unoptimized_keyframes",
            apply_pgo_correction_to_unoptimized_keyframes_
        );
        this->get_parameter(
            "optimized_keyframes_tail_exclusion_count",
            optimized_keyframes_tail_exclusion_count_
        );
        if (optimized_keyframes_tail_exclusion_count_ < 0)
            optimized_keyframes_tail_exclusion_count_ = 0;
        if (map_frame_.empty())
            map_frame_ = "map";
        if (odom_frame_.empty())
            odom_frame_ = "odom";

        const std::vector<double> t_default = {0.0, 0.0, 0.0};
        const std::vector<double> q_default = {1.0, 0.0, 0.0, 0.0};
        std::vector<double> t, q, matrix;
        this->declare_parameter("t_body_lidar", t_default);
        this->get_parameter("t_body_lidar", t);
        this->declare_parameter("q_body_lidar", q_default);
        this->get_parameter("q_body_lidar", q);
        this->declare_parameter("T_body_lidar", std::vector<double>{});
        this->get_parameter("T_body_lidar", matrix);

        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

        // 支持 t/q 和 T_body_lidar 两种外参配置。完整矩阵存在时优先使用，
        // 这样 launch/yaml 可以直接复用标定工具输出。
        if (t.size() == 3) {
            translation = Eigen::Vector3d(t[0], t[1], t[2]);
        } else {
            RCLCPP_WARN(
                get_logger(),
                "t_body_lidar must have 3 elements, got %zu; using zero translation",
                t.size()
            );
        }

        if (q.size() == 4) {
            rotation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
        } else {
            RCLCPP_WARN(
                get_logger(),
                "q_body_lidar must have 4 elements [w,x,y,z], got %zu; using identity rotation",
                q.size()
            );
        }

        if (matrix.size() == 16) {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    T(r, c) = matrix[static_cast<size_t>(r * 4 + c)];

            if (T.allFinite()) {
                translation = T.block<3, 1>(0, 3);
                rotation = Eigen::Quaterniond(T.block<3, 3>(0, 0));
            } else {
                RCLCPP_WARN(
                    get_logger(),
                    "T_body_lidar contains non-finite values; falling back to t_body_lidar/q_body_lidar"
                );
            }
        } else if (!matrix.empty()) {
            RCLCPP_WARN(
                get_logger(),
                "T_body_lidar must be a row-major 4x4 matrix with 16 elements, got %zu; "
                "falling back to t_body_lidar/q_body_lidar",
                matrix.size()
            );
        }

        if (!translation.allFinite() ||
            !rotation.coeffs().allFinite() ||
            rotation.norm() < 1e-9) {

            RCLCPP_WARN(
                get_logger(),
                "body->lidar extrinsic is invalid; using identity"
            );
            translation.setZero();
            rotation.setIdentity();
        } else {

            rotation.normalize();
        }

        T_body_lidar_ = Eigen::Isometry3d::Identity();
        T_body_lidar_.linear() = rotation.toRotationMatrix();
        T_body_lidar_.translation() = translation;

        RCLCPP_INFO(
            get_logger(),
            "Map T_body_lidar: t=[%.6f %.6f %.6f] q=[%.6f %.6f %.6f %.6f]",
            translation.x(), translation.y(), translation.z(),
            rotation.w(), rotation.x(), rotation.y(), rotation.z()
        );
        RCLCPP_INFO(
            get_logger(),
            "Map frames: map=%s raw_keyframes=%s; "
            "rebuild_unoptimized_keyframes_with_approximation=%d "
            "apply_pgo_correction_to_unoptimized_keyframes=%d "
            "optimized_keyframes_tail_exclusion_count=%d",
            map_frame_.c_str(),
            odom_frame_.c_str(),
            rebuild_unoptimized_keyframes_with_approximation_ ? 1 : 0,
            apply_pgo_correction_to_unoptimized_keyframes_ ? 1 : 0,
            optimized_keyframes_tail_exclusion_count_
        );
    }

    void MapNode::callbackKeyFrameMsg(
        small_point_lio_pgo::msg::KeyFrame::ConstSharedPtr msg
    ) {

        if (!msg) return;

        // 位姿来源（直接 odom）：msg->pose 是关键帧消息携带的 T_odom_body。
        // map_node 不使用 TF buffer，此入口没有应用 map->odom 修正。
        if (!msg->header.frame_id.empty() &&
            msg->header.frame_id != odom_frame_) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Raw keyframe pose frame is '%s', expected odom frame '%s'; "
                "map reconstruction assumes the pose is odom<-body",
                msg->header.frame_id.c_str(),
                odom_frame_.c_str()
            );
        }

        // 原始关键帧先统一到 body frame 存储。后续重建地图时，
        // 每个局部点云只需要乘 keyframe pose 到 map/world frame。
        auto cloud = normalizeCloudToBodyFrame(*msg);
        if (cloud->empty()) return;

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            StoredKeyFrame stored;
            stored.stamp = rclcpp::Time(msg->header.stamp);
            // 始终原样保存 T_odom_body，不能把它误标成 T_map_body。
            stored.raw_pose = msg->pose;
            stored.local_cloud = cloud;
            keyframes_[msg->id] = std::move(stored);

            const auto optimized_it = optimized_poses_.find(msg->id);
            if (optimized_it != optimized_poses_.end()) {

                // 如果优化结果先到、关键帧后到，用这一对 raw/optimized
                // 及时刷新 PGO correction，方便尾部未优化帧近似跟随。
                const Eigen::Isometry3d raw_pose =
                    poseToIsometry(msg->pose);
                const Eigen::Isometry3d optimized_pose =
                    poseToIsometry(optimized_it->second);
                if (raw_pose.matrix().allFinite() &&
                    optimized_pose.matrix().allFinite()) {

                    pgo_correction_ = optimized_pose * raw_pose.inverse();
                    has_pgo_correction_ =
                        pgo_correction_.matrix().allFinite();
                }
            }

            rebuildMap(rclcpp::Time(msg->header.stamp));
        }
    }

    void MapNode::callbackOptimizedKeyFrames(
        small_point_lio_pgo::msg::OptimizedKeyFrames::ConstSharedPtr msg
    ) {

        if (!msg || msg->ids.size() != msg->poses.size())
            return;

        // 位姿来源（PGO/map）：msg->poses 是 loop detector 直接发布的
        // g2o 优化结果 T_map_body，不是 map_node 通过 TF 查询得到的。
        if (!msg->header.frame_id.empty() &&
            msg->header.frame_id != map_frame_) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Optimized keyframe frame is '%s', expected map frame '%s'; "
                "map reconstruction assumes optimized poses are map<-body",
                msg->header.frame_id.c_str(),
                map_frame_.c_str()
            );
        }

        std::lock_guard<std::mutex> lock(map_mutex_);

        // 优化关键帧列表是全量快照。这里更新 optimized_poses_，
        // 同时可排除最新若干帧，避免还不稳定的 PGO 尾部拉动地图。
        bool have_latest_pair = false;
        double latest_stamp = -std::numeric_limits<double>::infinity();
        Eigen::Isometry3d latest_raw = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d latest_optimized = Eigen::Isometry3d::Identity();
        uint32_t latest_id = 0;
        const size_t tail_exclusion_count =
            std::min(
                static_cast<size_t>(optimized_keyframes_tail_exclusion_count_),
                msg->ids.size()
            );
        const size_t optimized_limit = msg->ids.size() - tail_exclusion_count;
        size_t excluded_tail_count = 0;

        for (size_t i = 0; i < msg->ids.size(); ++i) {

            const uint32_t id = msg->ids[i];
            if (i >= optimized_limit) {

                optimized_poses_.erase(id);
                ++ excluded_tail_count;
                continue;
            }
            optimized_poses_[id] = msg->poses[i];

            const auto kf_it = keyframes_.find(id);
            if (kf_it == keyframes_.end())
                continue;

            const Eigen::Isometry3d raw_pose =
                poseToIsometry(kf_it->second.raw_pose);
            const Eigen::Isometry3d optimized_pose =
                poseToIsometry(msg->poses[i]);
            if (!raw_pose.matrix().allFinite() ||
                !optimized_pose.matrix().allFinite())
                continue;

            const double stamp = kf_it->second.stamp.seconds();
            if (std::isfinite(stamp) && stamp >= latest_stamp) {

                latest_stamp = stamp;
                latest_raw = raw_pose;
                latest_optimized = optimized_pose;
                latest_id = id;
                have_latest_pair = true;
            }
        }

        if (have_latest_pair) {

            // 用最新的 raw/optimized 同名关键帧估计全局 correction。
            // 未出现在 optimized_poses_ 中的尾部关键帧可用它做近似修正。
            pgo_correction_ = latest_optimized * latest_raw.inverse();
            has_pgo_correction_ = pgo_correction_.matrix().allFinite();
            Eigen::Quaterniond q(pgo_correction_.rotation());
            q.normalize();
            double correction_rot_deg =
                Eigen::AngleAxisd(q).angle() * 180.0 / M_PI;
            if (correction_rot_deg > 180.0)
                correction_rot_deg = 360.0 - correction_rot_deg;
            RCLCPP_INFO(
                get_logger(),
                "Map updated PGO correction: anchor=%u raw=[%.3f %.3f %.3f] "
                "optimized=[%.3f %.3f %.3f] correction_t=[%.3f %.3f %.3f] "
                "correction_rot=%.2fdeg",
                latest_id,
                latest_raw.translation().x(),
                latest_raw.translation().y(),
                latest_raw.translation().z(),
                latest_optimized.translation().x(),
                latest_optimized.translation().y(),
                latest_optimized.translation().z(),
                pgo_correction_.translation().x(),
                pgo_correction_.translation().y(),
                pgo_correction_.translation().z(),
                correction_rot_deg
            );
        }
        if (excluded_tail_count > 0) {

            RCLCPP_INFO(
                get_logger(),
                "Map excluded optimized keyframe tail: excluded=%zu accepted=%zu total=%zu",
                excluded_tail_count,
                optimized_limit,
                msg->ids.size()
            );
        }

        rebuildMap(rclcpp::Time(msg->header.stamp));
    }

    void MapNode::callbackSaveMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) {

        (void)request;

        std::lock_guard<std::mutex> lock(map_mutex_);
        if (global_map_->empty()) {

            response->success = false;
            response->message = "global_map is empty";
            return;
        }

        const int ret = pcl::io::savePCDFileBinary(map_save_path_, *global_map_);
        response->success = (ret == 0);
        response->message = response->success
            ? "saved map to " + map_save_path_
            : "failed to save map to " + map_save_path_;
    }

    Eigen::Isometry3d MapNode::poseToIsometry(
        const geometry_msgs::msg::Pose &pose
    ) const {

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

        Eigen::Quaterniond q(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );
        q.normalize();

        T.linear() = q.toRotationMatrix();
        T.translation() = Eigen::Vector3d(
            pose.position.x,
            pose.position.y,
            pose.position.z
        );
        return T;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr MapNode::normalizeCloudToBodyFrame(
        const small_point_lio_pgo::msg::KeyFrame &msg
    ) const {

        // keyframe cloud 可能来自 body frame，也可能来自 lidar frame。
        // 内部统一存 body frame，避免 rebuildMap() 里重复判断 frame_id。
        auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(msg.cloud, *cloud_in);

        if (cloud_in->empty())
            return cloud_in;

        const std::string &frame_id = msg.cloud.header.frame_id;
        if (frame_id.empty() || frame_id == body_frame_)
            return cloud_in;

        auto cloud_body = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (frame_id == lidar_frame_) {

            pcl::transformPointCloud(
                *cloud_in,
                *cloud_body,
                T_body_lidar_.matrix().cast<float>()
            );
            return cloud_body;
        }

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Keyframe cloud frame '%s' is neither '%s' nor '%s'; treating it as body frame",
            frame_id.c_str(),
            body_frame_.c_str(),
            lidar_frame_.c_str()
        );
        return cloud_in;
    }

    void MapNode::rebuildMap(
        const rclcpp::Time &stamp
    ) {

        // 重建策略：
        // - 有优化位姿的关键帧使用 optimized pose；
        // - 没有优化位姿的尾部关键帧可跳过，或用 raw pose 叠加 PGO correction；
        // - 最后统一 voxel downsample 并发布 transient_local 地图。
        auto rebuilt = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        size_t optimized_count = 0;
        size_t corrected_unoptimized_count = 0;
        size_t raw_unoptimized_count = 0;
        size_t skipped_unoptimized_count = 0;
        for (const auto &kv : keyframes_) {

            const uint32_t id = kv.first;
            const StoredKeyFrame &keyframe = kv.second;
            if (!keyframe.local_cloud)
                continue;

            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            const auto optimized_it = optimized_poses_.find(id);
            if (optimized_it != optimized_poses_.end()) {

                // 位姿来源（PGO/map）：有精确优化结果时直接使用
                // T_map_body，不再乘 map->odom，否则会重复修正。
                T = poseToIsometry(optimized_it->second);
                ++ optimized_count;
            } else {

                // 一旦有过优化快照，未优化帧通常是最新尾部。
                // 可根据参数选择跳过它们，或者用 correction 近似贴合优化轨迹。
                if (!optimized_poses_.empty() &&
                    !rebuild_unoptimized_keyframes_with_approximation_) {

                    ++ skipped_unoptimized_count;
                    continue;
                }

                // 位姿来源（直接 odom）：尾部帧先读取保存的 T_odom_body。
                // 若启用近似修正，再在下一步显式左乘缓存的 T_map_odom；
                // 数学上等价于 TF 变换，但这里没有调用 TF 查询。
                T = poseToIsometry(keyframe.raw_pose);
                if (apply_pgo_correction_to_unoptimized_keyframes_ &&
                    has_pgo_correction_) {

                    T = pgo_correction_ * T;
                    ++ corrected_unoptimized_count;
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Applied PGO correction to unoptimized map keyframe: id=%u correction_t=[%.3f %.3f %.3f]",
                        id,
                        pgo_correction_.translation().x(),
                        pgo_correction_.translation().y(),
                        pgo_correction_.translation().z()
                    );
                } else {

                    ++ raw_unoptimized_count;
                }
            }

            if (!T.matrix().allFinite())
                continue;

            // local_cloud 已经在存储时转到 body frame，这里只做 body->map/world。
            pcl::PointCloud<pcl::PointXYZ> cloud_world;
            pcl::transformPointCloud(
                *keyframe.local_cloud,
                cloud_world,
                T.matrix().cast<float>()
            );

            *rebuilt += cloud_world;
        }

        if (rebuilt->empty()) return;
        // 地图每次全量重建后再降采样，保证 optimized pose 更新后
        // 不会混入旧位置的历史点云。
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setLeafSize(
            static_cast<float>(map_leaf_size_),
            static_cast<float>(map_leaf_size_),
            static_cast<float>(map_leaf_size_)
        );
        auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        voxel_filter.setInputCloud(rebuilt);
        voxel_filter.filter(*cloud_filtered);

        global_map_ = cloud_filtered;

        double correction_rot_deg = 0.0;
        if (has_pgo_correction_) {

            Eigen::Quaterniond q(pgo_correction_.rotation());
            q.normalize();
            correction_rot_deg = Eigen::AngleAxisd(q).angle() * 180.0 / M_PI;
            if (correction_rot_deg > 180.0)
                correction_rot_deg = 360.0 - correction_rot_deg;
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Map rebuild stats: stored=%zu optimized=%zu corrected_unoptimized=%zu "
            "raw_unoptimized=%zu skipped_unoptimized=%zu has_correction=%d "
            "correction_t=[%.3f %.3f %.3f] correction_rot=%.2fdeg points=%zu filtered=%zu",
            keyframes_.size(),
            optimized_count,
            corrected_unoptimized_count,
            raw_unoptimized_count,
            skipped_unoptimized_count,
            has_pgo_correction_ ? 1 : 0,
            pgo_correction_.translation().x(),
            pgo_correction_.translation().y(),
            pgo_correction_.translation().z(),
            correction_rot_deg,
            rebuilt->size(),
            global_map_->size()
        );

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header.stamp = stamp;
        map_msg.header.frame_id = map_frame_;
        pub_map_->publish(map_msg);
    }
} // small_point_lio_pgo
