#ifndef SMALL_POINT_LIO_PGO__LOOP_DETECTOR_NODE_HPP
#define SMALL_POINT_LIO_PGO__LOOP_DETECTOR_NODE_HPP

#include "LidarIris.h"
#include "cart_context.hpp"
#include "small_point_lio_pgo/msg/key_frame.hpp"
#include "gicp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pose_graph.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "small_point_lio_pgo/msg/optimized_key_frames.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <memory>
#include <limits>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <string>
#include <utility>
#include <vector>

namespace small_point_lio_pgo {

    class LoopDetectorNode : public rclcpp::Node {

    public:

        LoopDetectorNode();

    private:

        struct LoopKeyFrame {

            uint32_t id = 0;
            rclcpp::Time stamp;
            double travel_distance = 0.0;
            geometry_msgs::msg::Pose raw_pose;
            geometry_msgs::msg::Pose pose;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr descriptor_cloud;
            cv::Mat1b iris_image;
            LidarIris::FeatureDesc iris_descriptor;
            CartContext::Descriptor cart_descriptor;
            bool has_cart_descriptor = false;
        };

        struct RetrievalHit {

            const LoopKeyFrame *history = nullptr;
            float distance = std::numeric_limits<float>::infinity();
        };

        // Save potential loop-detection frame
        struct LoopCandidate {

            uint32_t current_id = 0;
            uint32_t history_id = 0;
            float iris_distance = 0.0F;
            float cart_distance = std::numeric_limits<float>::infinity();
            int cart_shift_cols = 0;
            double cart_lateral_offset_m = 0.0;
            int yaw_bias = 0;
            bool gicp_verified = false;
            double gicp_score = 1e9;
            double gicp_selection_cost = std::numeric_limits<double>::infinity();
            double gicp_correction_trans = 0.0;
            double gicp_correction_rot_deg = 0.0;
            double gicp_inlier_ratio = 0.0;
            double gicp_registration_error =
                std::numeric_limits<double>::infinity();
            size_t gicp_iterations = 0;
            size_t gicp_num_inliers = 0;
            Eigen::Matrix4d source_to_target = Eigen::Matrix4d::Identity();
        };

        void loadParams();

        void publishOptimizedKeyFrames() const;

        void updateMapToOdomFromOptimizedGraph();

        void publishMapToOdom() const;

        void callbackKeyFrame(
            const small_point_lio_pgo::msg::KeyFrame::SharedPtr msg
        );

        void callbackImu(
            const sensor_msgs::msg::Imu::SharedPtr msg
        );

        bool convertKeyFrameMsg(
            const small_point_lio_pgo::msg::KeyFrame &msg,
            LoopKeyFrame &keyframe
        ) const;

        bool filterKeyFrameMsgCloud(
            small_point_lio_pgo::msg::KeyFrame &msg
        ) const;

        bool shouldAcceptKeyFrameCandidate(
            const small_point_lio_pgo::msg::KeyFrame &msg
        ) const;

        std::pair<double, double> poseDelta(
            const geometry_msgs::msg::Pose &from,
            const geometry_msgs::msg::Pose &to
        ) const;

        static size_t pointCount(
            const sensor_msgs::msg::PointCloud2 &cloud
        );

        std::vector<RetrievalHit> retrieveByCartKey(
            const LoopKeyFrame &current,
            int &eligible_count
        ) const;

        float retrievalKeyDistance(
            const Eigen::VectorXf &a,
            const Eigen::VectorXf &b
        ) const;

        float retrievalKeyReverseDistance(
            const Eigen::VectorXf &a,
            const Eigen::VectorXf &b
        ) const;

        bool computeIrisDescriptor(
            LoopKeyFrame &keyframe
        );

        bool computeCartDescriptor(
            LoopKeyFrame &keyframe
        ) const;

        bool gravityAlignmentReady() const;

        Eigen::Isometry3d gravityAlignmentTransform() const;

        void configureManualGravityAlignment();

        bool finalizeGravityAlignmentFromAcceleration(
            const Eigen::Vector3d &acc_mean,
            size_t sample_count,
            double sample_span_sec,
            const char *source
        );

        Eigen::Isometry3d alignedPose(
            const geometry_msgs::msg::Pose &raw_pose
        ) const;

        pcl::PointCloud<pcl::PointXYZ>::Ptr makeDescriptorCloud(
            const LoopKeyFrame &keyframe
        ) const;

        pcl::PointCloud<pcl::PointXYZ>::Ptr makeDescriptorCloud(
            const LoopKeyFrame &keyframe,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud
        ) const;

        pcl::PointCloud<pcl::PointXYZ>::Ptr buildIrisSubmapCloud(
            const LoopKeyFrame &anchor,
            size_t &frames_used,
            size_t &input_points,
            double &span_m
        ) const;

        static Eigen::Matrix3d rotationFromRpyDeg(
            const std::vector<double> &rpy_deg
        );

        static Eigen::Vector3d rpyDegFromRotation(
            const Eigen::Matrix3d &R
        );

        bool detectLoopCandidate(
            const LoopKeyFrame &current,
            LoopCandidate &candidate
        ) const;

        bool detectLoopCandidate(
            const LoopKeyFrame &current,
            LoopCandidate &candidate,
            float &best_distance,
            float &best_cart_distance,
            int &eligible_count
        ) const;

        bool detectLoopCandidates(
            const LoopKeyFrame &current,
            std::vector<LoopCandidate> &candidates,
            float &best_distance,
            float &best_cart_distance,
            int &eligible_count
        ) const;

        bool verifyLoopCandidateByGicp(
            const LoopKeyFrame &current,
            LoopCandidate &candidate
        );

        pcl::PointCloud<pcl::PointXYZ>::Ptr buildLoopGicpSubmap(
            const LoopKeyFrame &anchor
        ) const;

        bool appendKeyFrameToSubmap(
            const LoopKeyFrame &anchor,
            const LoopKeyFrame &frame,
            pcl::PointCloud<pcl::PointXYZ> &submap
        ) const;

        bool isCandidateAllowed(
            const LoopKeyFrame &current,
            const LoopKeyFrame &history
        ) const;

        const LoopKeyFrame *findKeyFrame(
            uint32_t id
        ) const;

        Eigen::Matrix4d poseToMatrix(
            const geometry_msgs::msg::Pose &pose
        ) const;

        Eigen::Isometry3d poseToIsometry(
            const geometry_msgs::msg::Pose &pose
        ) const;

        Eigen::Isometry3d matrixToIsometry(
            const Eigen::Matrix4d &matrix
        ) const;

        geometry_msgs::msg::Pose isometryToPose(
            const Eigen::Isometry3d &pose
        ) const;

        Eigen::Matrix<double, 6, 6> makeInformationMatrix(
            const std::vector<double> &diag,
            double fallback_weight,
            const char *param_name
        ) const;

        std::pair<double, double> relativeDelta(
            const Eigen::Isometry3d &a,
            const Eigen::Isometry3d &b
        ) const;

        Eigen::Isometry3d makeLoopMeasurement(
            const Eigen::Isometry3d &T_world_history,
            const Eigen::Isometry3d &T_world_current,
            const Eigen::Isometry3d &full_measurement,
            Eigen::Vector3d &world_translation_correction,
            Eigen::Vector3d &world_rotation_correction_rpy_deg
        ) const;

        bool addKeyFrameToPoseGraph(
            const LoopKeyFrame &keyframe
        );

        bool addLoopCandidateToPoseGraph(
            const LoopCandidate &candidate
        );

        bool shouldAddLoopEdgeToPoseGraph(
            const LoopCandidate &candidate,
            int &current_gap,
            double &travel_gap
        ) const;

        double optimizePoseGraphIfNeeded(
            bool has_new_loop
        );

        void storeKeyFrame(
            LoopKeyFrame &&keyframe
        );

        void publishLoopMarkers() const;

        void publishOptimizedPath() const;

        void publishTiming(
            double lcd_ms,
            double pgo_ms,
            const rclcpp::Time &stamp
        ) const;

        rclcpp::Subscription<small_point_lio_pgo::msg::KeyFrame>::SharedPtr sub_keyframe_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            pub_loop_markers_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
            pub_optimized_path_;
        rclcpp::Publisher<small_point_lio_pgo::msg::KeyFrame>::SharedPtr
            pub_filtered_keyframe_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
            pub_timing_;

        std::unique_ptr<LidarIris> lidar_iris_;
        std::unique_ptr<CartContext> cart_context_;
        GicpMatcher gicp_matcher_;
        PoseGraph pose_graph_;
        std::vector<LoopKeyFrame> keyframes_;
        std::vector<LoopCandidate> loop_candidates_;
        rclcpp::Publisher<small_point_lio_pgo::msg::OptimizedKeyFrames>::SharedPtr
            pub_optimized_keyframes_;

        std::string optimized_keyframes_topic_ = "optimized_keyframes";
        std::string keyframe_topic_ = "keyframe_msg";
        std::string filtered_keyframe_topic_;
        std::string marker_topic_ = "loop_candidates_marker";
        std::string optimized_path_topic_ = "optimized_path";
        std::string timing_topic_ = "/lcd_pgo_timing";
        // The optimized graph and map outputs live in map. The input pose
        // stream remains in odom and is connected through a dynamic TF.
        std::string map_frame_ = "map";
        std::string odom_frame_ = "odom";
        // Kept as a compatibility alias for older launch files.
        std::string marker_frame_ = "map";
        // Compatibility parameter name: true applies the cached PGO
        // correction; false still publishes an identity map->odom TF.
        bool apply_map_to_odom_correction_ = true;
        bool map_to_odom_tf_enabled_ = true;
        double map_to_odom_publish_rate_hz_ = 20.0;

        double kf_trans_thresh_ = 0.5;
        double kf_rot_thresh_ = 0.1745;
        double min_kf_interval_sec_ = 0.0;
        int min_cloud_points_ = 10;

        bool loop_enable_ = true;
        int loop_min_keyframe_gap_ = 0;
        double loop_min_travel_distance_ = 8.0;
        float loop_iris_distance_thresh_ = 0.30F;
        bool loop_gicp_enable_ = true;
        double loop_gicp_score_thresh_ = 1.0;
        double loop_gicp_max_correction_trans_ = 3.0;
        double loop_gicp_max_correction_rot_deg_ = 20.0;
        double loop_gicp_selection_correction_trans_weight_ = 0.1;
        double loop_gicp_selection_correction_rot_weight_ = 0.0;
        bool loop_gicp_use_submap_ = false;
        int loop_gicp_submap_keyframes_ = 5;
        double loop_gicp_submap_leaf_size_ = 0.2;
        int loop_gicp_num_threads_ = 4;
        int loop_gicp_correspondence_randomness_ = 20;
        double loop_gicp_max_correspondence_distance_ = 1.0;
        int loop_gicp_max_iterations_ = 40;
        double loop_gicp_rotation_epsilon_ = 2e-3;
        double loop_gicp_transformation_epsilon_ = 1e-3;
        int loop_gicp_min_inliers_ = 100;
        double loop_gicp_min_inlier_ratio_ = 0.05;
        std::string body_frame_ = "base_link";
        std::string lidar_frame_ = "livox_frame";
        Eigen::Isometry3d T_body_lidar_ = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_lidar_body_ = Eigen::Isometry3d::Identity();

        bool gravity_align_enable_ = false;
        std::string gravity_align_source_ = "imu_average";
        std::string gravity_align_imu_topic_ = "/livox/imu";
        double gravity_align_sample_sec_ = 5.0;
        double gravity_align_acc_scale_ = 9.80665;
        std::vector<double> gravity_align_manual_rpy_deg_ = {0.0, 0.0, 0.0};
        bool gravity_align_apply_to_descriptor_ = true;
        bool gravity_align_apply_to_pgo_ = true;
        bool gravity_align_apply_to_map_ = true;
        mutable std::mutex gravity_align_mutex_;
        Eigen::Isometry3d T_gravity_align_ = Eigen::Isometry3d::Identity();
        Eigen::Vector3d gravity_align_acc_mean_ = Eigen::Vector3d::Zero();
        size_t gravity_align_sample_count_ = 0;
        double gravity_align_first_stamp_ = 0.0;
        double gravity_align_latest_stamp_ = 0.0;
        bool gravity_align_ready_ = false;
        bool gravity_align_started_ = false;

        bool keyframe_exclusion_box_enable_ = false;
        double keyframe_exclusion_min_x_ = -3.0;
        double keyframe_exclusion_max_x_ = -0.2;
        double keyframe_exclusion_min_y_ = -1.2;
        double keyframe_exclusion_max_y_ = 1.2;
        double keyframe_exclusion_min_z_ = -0.5;
        double keyframe_exclusion_max_z_ = 2.0;

        bool pgo_enable_ = false;
        bool pgo_optimize_on_loop_ = true;
        int pgo_max_iterations_ = 20;
        double pgo_odom_edge_weight_ = 100.0;
        double pgo_loop_edge_weight_ = 500.0;
        int loop_edge_min_current_gap_ = 20;
        double loop_edge_min_travel_gap_ = 5.0;
        std::vector<double> pgo_odom_info_diag_;
        std::vector<double> pgo_loop_info_diag_;
        bool pgo_loop_info_dynamic_enable_ = true;
        double pgo_loop_info_score_ref_ = 0.5;
        double pgo_loop_info_score_floor_ = 0.05;
        double pgo_loop_info_min_scale_ = 0.2;
        double pgo_loop_info_max_scale_ = 2.0;
        bool pgo_loop_robust_kernel_enable_ = true;
        double pgo_loop_robust_kernel_delta_ = 1.0;
        std::string pgo_loop_measurement_mode_ = "gravity_preserving";
        bool has_pgo_optimized_snapshot_ = false;

        // map->odom is a cached global correction. It changes only after a
        // successful PGO run; new frontend keyframes must not reset it.
        Eigen::Isometry3d map_from_odom_ = Eigen::Isometry3d::Identity();
        uint32_t map_to_odom_anchor_id_ = 0;
        bool map_to_odom_has_anchor_ = false;
        mutable std::mutex map_to_odom_mutex_;
        rclcpp::TimerBase::SharedPtr map_to_odom_timer_;

        bool lcd_retrieval_enable_ = false;
        int lcd_retrieval_top_k_ = 150;
        double lcd_retrieval_max_distance_ = -1.0;

        int iris_nscale_ = 4;
        int iris_min_wave_length_ = 18;
        float iris_mult_ = 1.6F;
        float iris_sigma_onf_ = 0.75F;
        int iris_match_num_ = 2;
        bool iris_submap_enable_ = true;
        int iris_submap_keyframes_ = 5;
        double iris_submap_voxel_leaf_m_ = 0.25;

        bool cart_enable_ = true;
        double cart_x_unit_m_ = 0.5;
        double cart_y_unit_m_ = 0.1;
        double cart_x_max_m_ = 50.0;
        double cart_y_max_m_ = 25.0;
        double cart_voxel_leaf_m_ = 0.0;
        double cart_height_offset_m_ = 0.0;
        bool cart_use_align_key_ = true;
        double cart_align_search_ratio_ = 0.04;
        int cart_candidate_top_k_ = 10;
        int loop_descriptor_verify_top_k_ = 5;
        double cart_distance_thresh_ = 0.65;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        uint64_t received_keyframes_ = 0;
        uint64_t stored_keyframes_ = 0;
        uint64_t iris_failures_ = 0;
        uint64_t candidate_hits_ = 0;
        uint64_t candidate_misses_ = 0;
        double accumulated_travel_distance_ = 0.0;
        // Keyframe gate compares incoming raw poses; never store the gravity-aligned pose here.
        geometry_msgs::msg::Pose last_accepted_keyframe_raw_pose_;
        rclcpp::Time last_accepted_keyframe_stamp_;
        bool has_last_accepted_keyframe_ = false;
        uint32_t last_added_loop_current_id_ = 0;
        double last_added_loop_current_travel_distance_ = 0.0;
        bool has_last_added_loop_edge_ = false;
    };

} // namespace small_point_lio_pgo

#endif  // SMALL_POINT_LIO_PGO__LOOP_DETECTOR_NODE_HPP
