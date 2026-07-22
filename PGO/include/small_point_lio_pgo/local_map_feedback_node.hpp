#ifndef SMALL_POINT_LIO_PGO__LOCAL_MAP_FEEDBACK_NODE_HPP
#define SMALL_POINT_LIO_PGO__LOCAL_MAP_FEEDBACK_NODE_HPP

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

#include "small_point_lio_interfaces/msg/local_tracking_map.hpp"
#include "small_point_lio_interfaces/msg/scan_to_map_correction.hpp"
#include "small_point_lio_pgo/msg/key_frame.hpp"
#include "small_point_lio_pgo/msg/optimized_key_frames.hpp"
#include "small_point_lio_pgo/pose_graph.hpp"

namespace small_point_lio_pgo {

    class LocalMapFeedbackTestPeer;

    class LocalMapFeedbackNode : public rclcpp::Node {

    public:

        LocalMapFeedbackNode();
        ~LocalMapFeedbackNode() override;
    private:

        friend class LocalMapFeedbackTestPeer;

        using SteadyClock = std::chrono::steady_clock;

        /// 后端保存的 packet-level scan-to-map 证据副本，用于质量门限和局部滑窗优化。
        struct CorrectionRecord {
            double stamp = 0.0;///< LiDAR packet 边界时间，单位秒。
            uint64_t sequence = 0;///< 前端分配的 correction 序号。
            uint64_t tracking_map_version = 0;///< 生成该证据时使用的 iVox 版本。
            uint64_t active_map_source_correction_sequence = 0;///< 当前 iVox 的来源 correction 序号。
            // ALL is IMU-ONLY
            // VVVV
            /// 从前一个 packet 边界仅用 IMU 推进的位姿。
            Eigen::Isometry3d packet_predicted =
                    Eigen::Isometry3d::Identity();
            /// 从当前地图 epoch 以来仅用 IMU 推进的位姿。
            Eigen::Isometry3d epoch_predicted =
                    Eigen::Isometry3d::Identity();
            // ^^^^
            /// 前端完成本 packet 点更新后的 ESKF 位姿。
            Eigen::Isometry3d corrected = Eigen::Isometry3d::Identity();
            uint32_t attempted_updates = 0;///< Point residuals submitted to the ESKF.
            uint32_t accepted_updates = 0; ///< Point residuals accepted by the ESKF.
            double residual_rms = 0.0;     ///< RMS over accepted point residuals.
            double residual_max_abs = 0.0; ///< Largest accepted absolute residual.
        };

        /// 持久化关键帧候选，同时服务于 active 局部窗口和 frozen 历史地图。
        struct CandidateRecord {
            uint32_t id = 0;///< 稳定的关键帧/图节点 ID。
            double stamp = 0.0;///< 关键帧采集时间，单位秒。
            Eigen::Isometry3d raw_pose = Eigen::Isometry3d::Identity();///< 原始 odom 位姿。
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_body;///< body 坐标系下的去畸变点云。
            bool has_epoch_prediction = false;///< 是否已经关联到有效的 correction-free 预测。
            uint64_t prediction_map_version = 0;///< 该预测对应的 tracking-map 版本。
            Eigen::Isometry3d epoch_prediction =
                    Eigen::Isometry3d::Identity();///< 关键帧时间处的 correction-free 预测。
        };

        /// 局部优化位姿缓存；只有地图版本和 PGO 版本都匹配时才可复用。
        struct LocalPoseOverride {
            uint64_t tracking_map_version = 0;///< 该位姿所属的 tracking-map 版本。
            uint64_t pgo_graph_version = 0;   ///< 该位姿吸收的 PGO 快照版本。
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();///< odom 坐标系下的局部优化位姿。
        };

        /// 复制到后台任务的不可变关键帧快照，避免 worker 读取 callback 容器。
        struct FrameSnapshot {
            uint32_t candidate_id = 0;///< 稳定的关键帧 ID。
            double stamp = 0.0;      ///< 关键帧时间，单位秒。
            /// 应用有效局部/PGO 修正后的优化初值。
            Eigen::Isometry3d initial_pose =
                    Eigen::Isometry3d::Identity();
            /// 用于构造局部 odom edge 的 correction-free 位姿。
            Eigen::Isometry3d epoch_prediction =
                    Eigen::Isometry3d::Identity();
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_body;///< 不可变 body 坐标系点云。
            double distance_to_current = 0.0;///< 用于挑选附近历史帧的距离。
        };

        /// 自包含异步任务：优化 active 尾部并合并附近 frozen 历史重建 tracking cloud。
        struct BuildJob {
            CorrectionRecord correction;///< 终端约束及其来源 correction 版本。
            uint64_t runtime_generation = 0;///< 运行代次，防止 reset 前任务生效。
            uint64_t pgo_graph_version = 0;///< 创建任务时捕获的 PGO 图版本。
            std::string trigger_reason;///< 任务触发原因。
            std::vector<FrameSnapshot> active_frames;///< 由局部滑窗图优化的关键帧。
            std::vector<FrameSnapshot> frozen_history;///< 只插入地图、不参与本轮优化的历史帧。
        };

        /// 局部优化位姿的事务暂存区，等待前端确认对应 tracking-map 已激活。
        struct PendingPoseCommit {
            uint64_t source_correction_sequence = 0;///< 优化使用的 correction 序号。
            uint64_t source_tracking_map_version = 0;///< 该证据观察到的旧地图版本。
            uint64_t target_tracking_map_version = 0;///< 前端切图后应出现的新版本。
            uint64_t pgo_graph_version = 0;///< 结果中包含的 PGO 图版本。
            std::unordered_map<uint32_t, Eigen::Isometry3d> poses;///< 关键帧 ID 到优化后 odom 位姿。
        };

        /// 汇总近期 packet 窗口的证据质量，用于判断是否触发优化。
        struct EvidenceSummary {
            size_t packet_count = 0;///< 证据窗口包含的 packet 数。
            uint64_t attempted_updates = 0;///< 窗口内提交的点残差总数。
            uint64_t accepted_updates = 0; ///< 窗口内接受的点残差总数。
            double duration_sec = 0.0;      ///< 证据窗口覆盖的时间跨度。
            double valid_ratio = 0.0;       ///< accepted / attempted 更新比例。
        };

        /// 重建 tracking map 时使用的整数体素坐标。
        struct VoxelKey {
            int64_t x = 0;
            int64_t y = 0;
            int64_t z = 0;

            bool operator==(const VoxelKey &other) const {
                return x == other.x && y == other.y && z == other.z;
            }
        };

        /// 使 VoxelKey 可以作为临时体素表哈希键。
        struct VoxelKeyHash {
            size_t operator()(const VoxelKey &key) const;
        };

        void load_parameters();

        /// 接收前端 packet correction，更新证据缓冲并检查触发条件。
        void correction_callback(
            small_point_lio_interfaces::msg::ScanToMapCorrection::
                ConstSharedPtr msg
        );

        /// 接收关键帧候选并与 correction 按时间关联。
        void candidate_callback(
            small_point_lio_pgo::msg::KeyFrame::ConstSharedPtr msg
        );

        /// 接收新的全局 PGO 快照并判断是否需要局部地图重建。
        void optimized_callback(
            small_point_lio_pgo::msg::OptimizedKeyFrames::
                ConstSharedPtr msg
        );

        // 名称带 _locked 的函数要求调用者已经持有 state_mutex_。
        /// 清空后端缓存并递增 runtime generation，使旧任务失效。
        void reset_runtime_state_locked(const std::string &reason);

        /// 根据前端新地图版本确认 pending pose 是否可以正式提交。
        void acknowledge_map_version_locked(
            uint64_t active_version,
            uint64_t active_map_source_correction_sequence
        );

        /// 为所有候选关键帧重新寻找时间上匹配的 correction。
        void associate_candidate_predictions_locked();

        /// 为单个关键帧关联最近的 correction-free epoch prediction。
        bool associate_candidate_prediction_locked(CandidateRecord &candidate);

        /// 汇总同一 tracking-map 版本内最近 packet 的更新质量。
        EvidenceSummary evidence_summary_locked(
            const CorrectionRecord &latest
        ) const;

        /// 判断触发条件并把最新任务放入 worker 队列。
        bool schedule_if_needed_locked(const CorrectionRecord &latest);

        /// 捕获版本、关键帧和地图快照，生成一个异步 BuildJob。
        std::optional<BuildJob> make_build_job_locked(
            const CorrectionRecord &latest,
            const std::string &reason
        );

        /// 为候选关键帧生成局部优化初值，并处理 PGO correction 插值。
        std::unordered_map<uint32_t, Eigen::Isometry3d>
            seed_poses_locked(
                uint64_t tracking_map_version,
                const std::unordered_map<uint32_t, Eigen::Isometry3d>
                    &optimized_poses,
                uint64_t optimized_version
            ) const;

        /// 判断新旧 PGO 快照造成的局部形变是否超过重建阈值。
        bool pgo_deformation_significant_locked(
            const std::unordered_map<uint32_t, Eigen::Isometry3d>
                &previous,
            const std::unordered_map<uint32_t, Eigen::Isometry3d>
                &current
        ) const;

        /// 后台等待并执行 BuildJob，发布仍然有效的地图结果。
        void worker_loop();

        /// 执行局部优化、构建地图消息并准备待提交位姿。
        bool execute_build_job(
            const BuildJob &job,
            small_point_lio_interfaces::msg::LocalTrackingMap &message,
            PendingPoseCommit &commit
        );

        /// 用局部 PoseGraph 优化 active 关键帧窗口。
        bool optimize_active_window(
            const BuildJob &job,
            std::vector<Eigen::Isometry3d> &optimized_poses
        ) const;

        /// 将历史和优化后的 active 点云变换到 odom 并体素降采样。
        pcl::PointCloud<pcl::PointXYZ>::Ptr build_tracking_cloud(
            const BuildJob &job,
            const std::vector<Eigen::Isometry3d> &optimized_poses
        ) const;

        static Eigen::Isometry3d pose_to_isometry(
            const geometry_msgs::msg::Pose &pose
        );

        static geometry_msgs::msg::Pose isometry_to_pose(
            const Eigen::Isometry3d &pose
        );

        static double stamp_to_seconds(
            const builtin_interfaces::msg::Time &stamp
        );

        static double rotation_angle(const Eigen::Matrix3d &rotation);

        static bool finite_pose(const Eigen::Isometry3d &pose);

        static Eigen::Isometry3d interpolate_correction(
            const Eigen::Isometry3d &from,
            const Eigen::Isometry3d &to,
            double alpha
        );

        mutable std::mutex state_mutex_;
        std::deque<CorrectionRecord> corrections_;///< 按时间保存的近期 correction 缓冲。
        std::deque<CandidateRecord> candidates_;///< 关键帧候选数据库。
        std::unordered_map<uint32_t, LocalPoseOverride> local_pose_overrides_;///< 已确认地图版本对应的局部位姿。
        std::unordered_map<uint32_t, Eigen::Isometry3d> optimized_poses_;///< 最新 PGO 全局位姿快照。
        uint64_t pgo_graph_version_ = 0;///< 后端当前收到的 PGO 图版本。
        uint64_t applied_pgo_graph_version_ = 0;///< 已随前端地图切换确认的 PGO 版本。
        uint64_t active_tracking_map_version_ = 0;///< 后端记录的前端当前地图版本。
        uint64_t runtime_generation_ = 0;///< 后端运行代次，隔离时间跳变和 reset。
        double last_correction_stamp_ = -1.0;///< 最近收到的 correction 时间。
        bool has_correction_ = false;///< 是否已经收到至少一条合法 correction。
        bool pgo_rebuild_pending_ = false;///< 是否有尚未折叠进局部地图的 PGO 更新。

        bool build_in_progress_ = false;///< 是否已有 BuildJob 在 worker 中执行。
        bool awaiting_map_apply_ = false;///< 是否已发布地图、等待前端确认切换。
        uint64_t awaiting_target_version_ = 0;///< 正在等待确认的目标地图版本。
        SteadyClock::time_point awaiting_since_ = SteadyClock::now();///< 开始等待前端切图的 steady 时间。
        SteadyClock::time_point last_map_applied_time_ =
            SteadyClock::time_point::min();///< 上一次确认地图切换的 steady 时间。
        std::optional<PendingPoseCommit> pending_pose_commit_;///< 尚未确认提交的局部优化位姿。

        std::mutex worker_mutex_;///< 保护 worker 任务队列。
        std::condition_variable worker_cv_;///< 唤醒后台优化线程。
        std::optional<BuildJob> queued_job_;///< 等待执行的最新任务。
        bool stop_worker_ = false;///< 请求 worker 退出。
        std::thread worker_thread_;///< 后端局部优化/建图线程。

        rclcpp::Subscription<
            small_point_lio_interfaces::msg::ScanToMapCorrection>::
            SharedPtr correction_subscription_;
        rclcpp::Subscription<small_point_lio_pgo::msg::KeyFrame>::SharedPtr
            candidate_subscription_;
        rclcpp::Subscription<
            small_point_lio_pgo::msg::OptimizedKeyFrames>::SharedPtr
            optimized_subscription_;
        rclcpp::Publisher<
            small_point_lio_interfaces::msg::LocalTrackingMap>::SharedPtr
            map_publisher_;

        std::string correction_topic_ = "/pointlio_scan_to_map_correction";
        std::string candidate_topic_ = "/keyframe_candidates";
        std::string optimized_topic_ = "/optimized_keyframes";
        std::string local_map_topic_ = "/local_tracking_map";
        std::string odom_frame_ = "odom";

        int evidence_window_packets_ = 30;
        double evidence_window_duration_sec_ = 0.50;
        int evidence_min_packets_ = 3;
        int evidence_min_valid_updates_ = 100;
        double evidence_min_duration_sec_ = 0.02;
        double evidence_min_valid_ratio_ = 0.02;
        double cumulative_trigger_translation_m_ = 0.15;
        double cumulative_trigger_rotation_deg_ = 2.0;
        double instant_trigger_translation_m_ = 0.25;
        double instant_trigger_rotation_deg_ = 3.0;
        int instant_trigger_min_valid_updates_ = 50;
        double max_safe_correction_translation_m_ = 2.0;
        double max_safe_correction_rotation_deg_ = 30.0;
        double optimization_cooldown_sec_ = 1.0;
        double map_apply_timeout_sec_ = 2.0;
        double candidate_correction_max_delay_sec_ = 0.06;
        double correction_buffer_duration_sec_ = 15.0;
        int candidate_database_max_keyframes_ = 5000;
        int active_window_keyframes_ = 20;
        int min_active_keyframes_ = 3;
        double history_search_radius_m_ = 20.0;
        int history_max_keyframes_ = 80;
        double map_voxel_leaf_size_m_ = 0.20;
        int map_max_points_ = 300000;
        int map_min_points_ = 100;
        int local_graph_max_iterations_ = 20;
        std::vector<double> local_odom_info_diag_ =
            {300.0, 300.0, 150.0, 120.0, 120.0, 300.0};
        double pgo_deformation_trigger_translation_m_ = 0.05;
        double pgo_deformation_trigger_rotation_deg_ = 0.5;
        bool trigger_pgo_enable_ = true;
        bool trigger_instant_enable_ = true;
        bool trigger_normal_enable_ = true;
    };

}  // namespace small_point_lio_pgo

#endif  // SMALL_POINT_LIO_PGO__LOCAL_MAP_FEEDBACK_NODE_HPP
