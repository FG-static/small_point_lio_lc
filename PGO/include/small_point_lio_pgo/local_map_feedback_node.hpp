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

        /// Normalized packet-level scan-to-map evidence retained by the local
        /// feedback node for quality gating and fixed-lag optimization.
        struct CorrectionRecord {
            double stamp = 0.0;///< LiDAR packet boundary time, in seconds.
            uint64_t sequence = 0;///< Monotonic sequence assigned by the frontend.
            uint64_t tracking_map_version = 0;///< Map version used by this correction.
            uint64_t active_map_source_correction_sequence = 0;///< Sequence that produced that map.
            // ALL is IMU-ONLY
            // VVVV
            /// IMU-only pose propagated from the preceding packet boundary.
            Eigen::Isometry3d packet_predicted =
                    Eigen::Isometry3d::Identity();
            /// Correction-free pose propagated from the current map epoch.
            Eigen::Isometry3d epoch_predicted =
                    Eigen::Isometry3d::Identity();
            // ^^^^
            /// Frontend ESKF pose after the packet's scan-to-map updates.
            Eigen::Isometry3d corrected = Eigen::Isometry3d::Identity();
            uint32_t attempted_updates = 0;///< Point residuals submitted to the ESKF.
            uint32_t accepted_updates = 0; ///< Point residuals accepted by the ESKF.
            double residual_rms = 0.0;     ///< RMS over accepted point residuals.
            double residual_max_abs = 0.0; ///< Largest accepted absolute residual.
        };

        /// Persistent keyframe candidate used by both the active local window
        /// and the nearby frozen-history portion of the rebuilt tracking map.
        struct CandidateRecord {
            uint32_t id = 0;///< Stable keyframe/pose-graph identifier.
            double stamp = 0.0;///< Keyframe acquisition time, in seconds.
            Eigen::Isometry3d raw_pose = Eigen::Isometry3d::Identity();///< Original odom pose.
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_body;///< Undistorted keyframe cloud in its body frame.
            bool has_epoch_prediction = false;///< Whether a matching correction-free pose exists.
            uint64_t prediction_map_version = 0;///< Map epoch of the associated prediction.
            Eigen::Isometry3d epoch_prediction =
                    Eigen::Isometry3d::Identity();///< Prediction at the keyframe timestamp.
        };

        /// Locally optimized pose cached only for the map and PGO versions that
        /// produced it; version tags prevent stale overrides from being reused.
        struct LocalPoseOverride {
            uint64_t tracking_map_version = 0;///< Tracking-map version containing the pose.
            uint64_t pgo_graph_version = 0;   ///< PGO snapshot folded into the pose.
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();///< Pose in odom.
        };

        /// Immutable candidate snapshot copied into a worker job so background
        /// optimization never reads callback-owned containers.
        struct FrameSnapshot {
            uint32_t candidate_id = 0;///< Stable keyframe identifier.
            double stamp = 0.0;      ///< Keyframe acquisition time, in seconds.
            /// Seed pose after applying the valid local/PGO correction field.
            Eigen::Isometry3d initial_pose =
                    Eigen::Isometry3d::Identity();
            /// Correction-free pose used to form local odometry edges.
            Eigen::Isometry3d epoch_prediction =
                    Eigen::Isometry3d::Identity();
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_body;///< Immutable body-frame cloud.
            double distance_to_current = 0.0;///< Seed-pose distance used for history selection.
        };

        /// Self-contained asynchronous job that optimizes the active tail and
        /// rebuilds a tracking cloud with nearby frozen history.
        struct BuildJob {
            CorrectionRecord correction;///< Terminal constraint and source version.
            uint64_t runtime_generation = 0;///< Invalidates jobs across runtime resets.
            uint64_t pgo_graph_version = 0;///< PGO snapshot captured for this build.
            std::string trigger_reason;///< Threshold or PGO event that scheduled the job.
            std::vector<FrameSnapshot> active_frames;///< Poses optimized by the fixed-lag graph.
            std::vector<FrameSnapshot> frozen_history;///< Spatial history inserted without optimization.
        };

        /// Optimized active-tail poses held transactionally until the frontend
        /// acknowledges that the matching tracking-map version is active.
        struct PendingPoseCommit {
            uint64_t source_correction_sequence = 0;///< Evidence used by the optimization.
            uint64_t source_tracking_map_version = 0;///< Map version observed by that evidence.
            uint64_t target_tracking_map_version = 0;///< Version expected after frontend swap.
            uint64_t pgo_graph_version = 0;///< PGO snapshot incorporated in the result.
            std::unordered_map<uint32_t, Eigen::Isometry3d> poses;///< Candidate ID to optimized odom pose.
        };

        /// Aggregate evidence quality over the recent packet window used by the
        /// optimization trigger gate.
        struct EvidenceSummary {
            size_t packet_count = 0;///< Packets included in the evidence window.
            uint64_t attempted_updates = 0;///< Summed submitted point residuals.
            uint64_t accepted_updates = 0; ///< Summed accepted point residuals.
            double duration_sec = 0.0;      ///< Time span covered by the window.
            double valid_ratio = 0.0;       ///< Accepted-to-attempted update ratio.
        };

        /// Integer voxel coordinate used to downsample the rebuilt tracking map.
        struct VoxelKey {
            int64_t x = 0;
            int64_t y = 0;
            int64_t z = 0;

            bool operator==(const VoxelKey &other) const {
                return x == other.x && y == other.y && z == other.z;
            }
        };

        /// Hash functor allowing VoxelKey to index the temporary voxel table.
        struct VoxelKeyHash {
            size_t operator()(const VoxelKey &key) const;
        };

        void load_parameters();

        void correction_callback(
            small_point_lio_interfaces::msg::ScanToMapCorrection::
                ConstSharedPtr msg
        );

        void candidate_callback(
            small_point_lio_pgo::msg::KeyFrame::ConstSharedPtr msg
        );

        void optimized_callback(
            small_point_lio_pgo::msg::OptimizedKeyFrames::
                ConstSharedPtr msg
        );

        // "_locked" is meaning that it need mutex lock to change
        void reset_runtime_state_locked(const std::string &reason);

        void acknowledge_map_version_locked(
            uint64_t active_version,
            uint64_t active_map_source_correction_sequence
        );

        void associate_candidate_predictions_locked();

        bool associate_candidate_prediction_locked(CandidateRecord &candidate);

        EvidenceSummary evidence_summary_locked(
            const CorrectionRecord &latest
        ) const;

        bool schedule_if_needed_locked(const CorrectionRecord &latest);

        std::optional<BuildJob> make_build_job_locked(
            const CorrectionRecord &latest,
            const std::string &reason
        );

        std::unordered_map<uint32_t, Eigen::Isometry3d>
            seed_poses_locked(
                uint64_t tracking_map_version,
                const std::unordered_map<uint32_t, Eigen::Isometry3d>
                    &optimized_poses,
                uint64_t optimized_version
            ) const;

        bool pgo_deformation_significant_locked(
            const std::unordered_map<uint32_t, Eigen::Isometry3d>
                &previous,
            const std::unordered_map<uint32_t, Eigen::Isometry3d>
                &current
        ) const;

        void worker_loop();

        // packet it to a msg
        bool execute_build_job(
            const BuildJob &job,
            small_point_lio_interfaces::msg::LocalTrackingMap &message,
            PendingPoseCommit &commit
        );

        bool optimize_active_window(
            const BuildJob &job,
            std::vector<Eigen::Isometry3d> &optimized_poses
        ) const;

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
        std::deque<CorrectionRecord> corrections_;
        std::deque<CandidateRecord> candidates_;
        std::unordered_map<uint32_t, LocalPoseOverride> local_pose_overrides_;
        std::unordered_map<uint32_t, Eigen::Isometry3d> optimized_poses_;
        uint64_t pgo_graph_version_ = 0;
        uint64_t applied_pgo_graph_version_ = 0;
        uint64_t active_tracking_map_version_ = 0;
        uint64_t runtime_generation_ = 0;
        double last_correction_stamp_ = -1.0;
        bool has_correction_ = false;
        bool pgo_rebuild_pending_ = false;

        bool build_in_progress_ = false;
        bool awaiting_map_apply_ = false;
        uint64_t awaiting_target_version_ = 0;
        SteadyClock::time_point awaiting_since_ = SteadyClock::now();
        SteadyClock::time_point last_map_applied_time_ =
            SteadyClock::time_point::min();
        std::optional<PendingPoseCommit> pending_pose_commit_;

        std::mutex worker_mutex_;
        std::condition_variable worker_cv_;
        std::optional<BuildJob> queued_job_;
        bool stop_worker_ = false;
        std::thread worker_thread_;

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
    };

}  // namespace small_point_lio_pgo

#endif  // SMALL_POINT_LIO_PGO__LOCAL_MAP_FEEDBACK_NODE_HPP
