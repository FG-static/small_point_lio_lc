#include "small_point_lio_pgo/loop_detector_node.hpp"

#include "small_point_lio_pgo/msg/optimized_key_frames.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/qos.hpp>
#include <sstream>
#include <rclcpp/qos_overriding_options.hpp>
#include <utility>
#include <vector>

namespace small_point_lio_pgo {

    namespace {

        using SteadyClock = std::chrono::steady_clock;

        double elapsedMs(
            const SteadyClock::time_point &start,
            const SteadyClock::time_point &end
        ) {

            return std::chrono::duration<double, std::milli>(end - start).count();
        }
    }

    LoopDetectorNode::LoopDetectorNode() : Node("small_point_lio_pgo_loop_detector") {

        loadParams();

        if (map_to_odom_tf_enabled_)
            tf_broadcaster_ =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        if (map_to_odom_tf_enabled_) {

            const auto period_ms = std::chrono::milliseconds(
                std::max<long long>(
                    1LL,
                    static_cast<long long>(
                        std::llround(1000.0 / map_to_odom_publish_rate_hz_)
                    )
                )
            );
            map_to_odom_timer_ = create_wall_timer(
                period_ms,
                [this]() {
                    publishMapToOdom();
                }
            );
        }

        // 初始化描述子、GICP 和 PGO 后端。后续 callbackKeyFrame()
        // 只负责按关键帧流推进 LCD -> GICP -> PGO 的数据流。
        lidar_iris_ = std::make_unique<LidarIris>(
            iris_nscale_,
            iris_min_wave_length_,
            iris_mult_,
            iris_sigma_onf_,
            iris_match_num_
        );
        if (cart_enable_) {

            CartContext::Params cart_params;
            cart_params.x_unit_m = cart_x_unit_m_;
            cart_params.y_unit_m = cart_y_unit_m_;
            cart_params.x_max_m = cart_x_max_m_;
            cart_params.y_max_m = cart_y_max_m_;
            cart_params.voxel_leaf_m = cart_voxel_leaf_m_;
            cart_params.height_offset_m = cart_height_offset_m_;
            cart_params.use_align_key = cart_use_align_key_;
            cart_params.align_search_ratio = cart_align_search_ratio_;
            cart_context_ = std::make_unique<CartContext>(cart_params);
        }

        gicp_matcher_.configure(GicpParams{
            loop_gicp_num_threads_,
            loop_gicp_correspondence_randomness_,
            loop_gicp_max_correspondence_distance_,
            loop_gicp_max_iterations_,
            loop_gicp_rotation_epsilon_,
            loop_gicp_transformation_epsilon_
        });
        pose_graph_.configure(PoseGraphOptions{
            pgo_max_iterations_,
            true,
            pgo_odom_edge_weight_,
            pgo_loop_edge_weight_
        });

        if (gravity_align_enable_ &&
            gravity_align_source_ == "imu_average") {

            sub_imu_ =
                create_subscription<sensor_msgs::msg::Imu>(
                    gravity_align_imu_topic_,
                    rclcpp::SensorDataQoS(),
                    [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                        callbackImu(msg);
                    }
                );
        }

        sub_keyframe_ =
            create_subscription<small_point_lio_pgo::msg::KeyFrame>(
                keyframe_topic_,
                rclcpp::SensorDataQoS(),
                [this](small_point_lio_pgo::msg::KeyFrame::SharedPtr msg) {
                    callbackKeyFrame(msg);
                }
            );

        pub_loop_markers_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(
                marker_topic_,
                10
            );

        pub_optimized_path_ =
            create_publisher<nav_msgs::msg::Path>(
                optimized_path_topic_,
                rclcpp::QoS(1).transient_local().reliable()
                // 队列深度 1
                // 记录最后一条发布的消息，新订阅者连上后立即收到
                // tcp 式多次重传，确保数据送到
            );

        pub_optimized_keyframes_ =
            create_publisher<small_point_lio_pgo::msg::OptimizedKeyFrames>(
                optimized_keyframes_topic_,
                rclcpp::QoS(1).reliable().transient_local()
            );
        pub_timing_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>(
                timing_topic_,
                10
            );
        if (!filtered_keyframe_topic_.empty() &&
            filtered_keyframe_topic_ != keyframe_topic_) {

            pub_filtered_keyframe_ =
                create_publisher<small_point_lio_pgo::msg::KeyFrame>(
                    filtered_keyframe_topic_,
                    rclcpp::QoS(100)
                );
        }

        RCLCPP_INFO(
            get_logger(),
            "Loop detector backend: loop_gicp_enable=%d pgo_enable=%d "
            "map_frame=%s odom_frame=%s map_to_odom_correction=%d "
            "map_to_odom_rate=%.1f "
            "optimized_path_topic=%s min_keyframe_gap=%d min_travel=%.2f "
            "kf_trans=%.3f kf_rot=%.3f min_interval=%.3f filtered_topic=%s",
            loop_gicp_enable_ ? 1 : 0,
            pgo_enable_ ? 1 : 0,
            map_frame_.c_str(),
            odom_frame_.c_str(),
            apply_map_to_odom_correction_ ? 1 : 0,
            map_to_odom_publish_rate_hz_,
            optimized_path_topic_.c_str(),
            loop_min_keyframe_gap_,
            loop_min_travel_distance_,
            kf_trans_thresh_,
            kf_rot_thresh_,
            min_kf_interval_sec_,
            filtered_keyframe_topic_.empty() ? "<disabled>" :
                filtered_keyframe_topic_.c_str()
        );
        RCLCPP_INFO(
            get_logger(),
            "Gravity align: enabled=%d source=%s imu_topic=%s sample_sec=%.3f "
            "apply_descriptor=%d apply_pgo=%d apply_map=%d ready=%d",
            gravity_align_enable_ ? 1 : 0,
            gravity_align_source_.c_str(),
            gravity_align_imu_topic_.c_str(),
            gravity_align_sample_sec_,
            gravity_align_apply_to_descriptor_ ? 1 : 0,
            gravity_align_apply_to_pgo_ ? 1 : 0,
            gravity_align_apply_to_map_ ? 1 : 0,
            gravityAlignmentReady() ? 1 : 0
        );
        RCLCPP_INFO(
            get_logger(),
            "Iris submap: enabled=%d keyframes=%d voxel_leaf=%.3f",
            iris_submap_enable_ ? 1 : 0,
            iris_submap_keyframes_,
            iris_submap_voxel_leaf_m_
        );
        RCLCPP_INFO(
            get_logger(),
            "Loop GICP quality: max_iterations=%d rotation_epsilon=%.6g "
            "transformation_epsilon=%.6g score_thresh=%.3f "
            "min_inliers=%d min_inlier_ratio=%.3f pgo_measurement_mode=%s",
            loop_gicp_max_iterations_,
            loop_gicp_rotation_epsilon_,
            loop_gicp_transformation_epsilon_,
            loop_gicp_score_thresh_,
            loop_gicp_min_inliers_,
            loop_gicp_min_inlier_ratio_,
            pgo_loop_measurement_mode_.c_str()
        );
    }

    void LoopDetectorNode::loadParams() {

        // 参数分三类：关键帧筛选/LCD/GICP、PGO、描述子与可视化输出。
        // 这里先声明默认值，再统一读取，最后做范围修正。
        declare_parameter("keyframe_topic", keyframe_topic_);
        declare_parameter("filtered_keyframe_topic", filtered_keyframe_topic_);
        declare_parameter("marker_topic", marker_topic_);
        declare_parameter("optimized_path_topic", optimized_path_topic_);
        declare_parameter("timing_topic", timing_topic_);
        declare_parameter("map_frame", map_frame_);
        declare_parameter("odom_frame", odom_frame_);
        declare_parameter(
            "publish_map_to_odom",
            apply_map_to_odom_correction_
        );
        declare_parameter(
            "map_to_odom_publish_rate_hz",
            map_to_odom_publish_rate_hz_
        );
        // Deprecated alias retained so old launch files do not silently lose
        // their marker frame setting during the coordinate-frame migration.
        declare_parameter("marker_frame", marker_frame_);
        declare_parameter("kf_trans_thresh", kf_trans_thresh_);
        declare_parameter("kf_rot_thresh", kf_rot_thresh_);
        declare_parameter("min_kf_interval_sec", min_kf_interval_sec_);
        declare_parameter("min_cloud_points", min_cloud_points_);
        declare_parameter("loop_enable", loop_enable_);
        declare_parameter("loop_min_keyframe_gap", loop_min_keyframe_gap_);
        declare_parameter("loop_min_travel_distance", loop_min_travel_distance_);
        declare_parameter("loop_iris_distance_thresh", loop_iris_distance_thresh_);
        declare_parameter("loop_gicp_enable", loop_gicp_enable_);
        declare_parameter("loop_gicp_score_thresh", loop_gicp_score_thresh_);
        declare_parameter("loop_gicp_max_correction_trans", loop_gicp_max_correction_trans_);
        declare_parameter("loop_gicp_max_correction_rot_deg", loop_gicp_max_correction_rot_deg_);
        declare_parameter(
            "loop_gicp_selection_correction_trans_weight",
            loop_gicp_selection_correction_trans_weight_
        );
        declare_parameter(
            "loop_gicp_selection_correction_rot_weight",
            loop_gicp_selection_correction_rot_weight_
        );
        declare_parameter("loop_gicp_use_submap", loop_gicp_use_submap_);
        declare_parameter("loop_gicp_submap_keyframes", loop_gicp_submap_keyframes_);
        declare_parameter("loop_gicp_submap_leaf_size", loop_gicp_submap_leaf_size_);
        declare_parameter("loop_gicp_num_threads", loop_gicp_num_threads_);
        declare_parameter(
            "loop_gicp_correspondence_randomness",
            loop_gicp_correspondence_randomness_);
        declare_parameter(
            "loop_gicp_max_correspondence_distance",
            loop_gicp_max_correspondence_distance_);
        declare_parameter("loop_gicp_max_iterations", loop_gicp_max_iterations_);
        declare_parameter(
            "loop_gicp_rotation_epsilon",
            loop_gicp_rotation_epsilon_);
        declare_parameter(
            "loop_gicp_transformation_epsilon",
            loop_gicp_transformation_epsilon_);
        declare_parameter("loop_gicp_min_inliers", loop_gicp_min_inliers_);
        declare_parameter(
            "loop_gicp_min_inlier_ratio",
            loop_gicp_min_inlier_ratio_
        );
        declare_parameter("body_frame", body_frame_);
        declare_parameter("lidar_frame", lidar_frame_);
        declare_parameter("gravity_align_enable", gravity_align_enable_);
        declare_parameter("gravity_align_source", gravity_align_source_);
        declare_parameter("gravity_align_imu_topic", gravity_align_imu_topic_);
        declare_parameter("gravity_align_sample_sec", gravity_align_sample_sec_);
        declare_parameter("gravity_align_acc_scale", gravity_align_acc_scale_);
        declare_parameter(
            "gravity_align_manual_rpy_deg",
            gravity_align_manual_rpy_deg_
        );
        declare_parameter(
            "gravity_align_apply_to_descriptor",
            gravity_align_apply_to_descriptor_
        );
        declare_parameter("gravity_align_apply_to_pgo", gravity_align_apply_to_pgo_);
        declare_parameter("gravity_align_apply_to_map", gravity_align_apply_to_map_);
        declare_parameter("keyframe_exclusion_box_enable", keyframe_exclusion_box_enable_);
        declare_parameter("keyframe_exclusion_min_x", keyframe_exclusion_min_x_);
        declare_parameter("keyframe_exclusion_max_x", keyframe_exclusion_max_x_);
        declare_parameter("keyframe_exclusion_min_y", keyframe_exclusion_min_y_);
        declare_parameter("keyframe_exclusion_max_y", keyframe_exclusion_max_y_);
        declare_parameter("keyframe_exclusion_min_z", keyframe_exclusion_min_z_);
        declare_parameter("keyframe_exclusion_max_z", keyframe_exclusion_max_z_);
        declare_parameter("pgo_enable", pgo_enable_);
        declare_parameter("pgo_optimize_on_loop", pgo_optimize_on_loop_);
        declare_parameter("pgo_max_iterations", pgo_max_iterations_);
        declare_parameter("pgo_odom_edge_weight", pgo_odom_edge_weight_);
        declare_parameter("pgo_loop_edge_weight", pgo_loop_edge_weight_);
        declare_parameter("loop_edge_min_current_gap", loop_edge_min_current_gap_);
        declare_parameter("loop_edge_min_travel_gap", loop_edge_min_travel_gap_);
        declare_parameter("pgo_odom_info_diag", std::vector<double>{});
        declare_parameter("pgo_loop_info_diag", std::vector<double>{});
        declare_parameter(
            "pgo_loop_info_dynamic_enable",
            pgo_loop_info_dynamic_enable_
        );
        declare_parameter("pgo_loop_info_score_ref", pgo_loop_info_score_ref_);
        declare_parameter(
            "pgo_loop_info_score_floor",
            pgo_loop_info_score_floor_
        );
        declare_parameter("pgo_loop_info_min_scale", pgo_loop_info_min_scale_);
        declare_parameter("pgo_loop_info_max_scale", pgo_loop_info_max_scale_);
        declare_parameter(
            "pgo_loop_robust_kernel_enable",
            pgo_loop_robust_kernel_enable_
        );
        declare_parameter(
            "pgo_loop_robust_kernel_delta",
            pgo_loop_robust_kernel_delta_
        );
        declare_parameter(
            "pgo_loop_measurement_mode",
            pgo_loop_measurement_mode_
        );
        declare_parameter("lcd_retrieval_enable", lcd_retrieval_enable_);
        declare_parameter("lcd_retrieval_top_k", lcd_retrieval_top_k_);
        declare_parameter("lcd_retrieval_max_distance", lcd_retrieval_max_distance_);
        declare_parameter("iris_nscale", iris_nscale_);
        declare_parameter("iris_min_wave_length", iris_min_wave_length_);
        declare_parameter("iris_mult", iris_mult_);
        declare_parameter("iris_sigma_onf", iris_sigma_onf_);
        declare_parameter("iris_match_num", iris_match_num_);
        declare_parameter("iris_submap_enable", iris_submap_enable_);
        declare_parameter("iris_submap_keyframes", iris_submap_keyframes_);
        declare_parameter("iris_submap_voxel_leaf_m", iris_submap_voxel_leaf_m_);
        declare_parameter("cart_enable", cart_enable_);
        declare_parameter("cart_x_unit_m", cart_x_unit_m_);
        declare_parameter("cart_y_unit_m", cart_y_unit_m_);
        declare_parameter("cart_x_max_m", cart_x_max_m_);
        declare_parameter("cart_y_max_m", cart_y_max_m_);
        declare_parameter("cart_voxel_leaf_m", cart_voxel_leaf_m_);
        declare_parameter("cart_height_offset_m", cart_height_offset_m_);
        declare_parameter("cart_use_align_key", cart_use_align_key_);
        declare_parameter("cart_align_search_ratio", cart_align_search_ratio_);
        declare_parameter("cart_candidate_top_k", cart_candidate_top_k_);
        declare_parameter("loop_descriptor_verify_top_k", loop_descriptor_verify_top_k_);
        declare_parameter("cart_distance_thresh", cart_distance_thresh_);
        declare_parameter("optimized_keyframes_topic", optimized_keyframes_topic_);

        get_parameter("keyframe_topic", keyframe_topic_);
        get_parameter("filtered_keyframe_topic", filtered_keyframe_topic_);
        get_parameter("marker_topic", marker_topic_);
        get_parameter("optimized_path_topic", optimized_path_topic_);
        get_parameter("timing_topic", timing_topic_);
        get_parameter("map_frame", map_frame_);
        get_parameter("odom_frame", odom_frame_);
        get_parameter(
            "publish_map_to_odom",
            apply_map_to_odom_correction_
        );
        get_parameter(
            "map_to_odom_publish_rate_hz",
            map_to_odom_publish_rate_hz_
        );
        get_parameter("marker_frame", marker_frame_);

        // Older configurations only had marker_frame. Use it as a fallback
        // when map_frame was not explicitly changed from its default.
        if (map_frame_ == "map" && marker_frame_ != "map")
            map_frame_ = marker_frame_;
        marker_frame_ = map_frame_;
        if (map_frame_.empty())
            map_frame_ = "map";
        if (odom_frame_.empty())
            odom_frame_ = "odom";
        if (map_frame_ == odom_frame_) {

            RCLCPP_WARN(
                get_logger(),
                "map_frame and odom_frame are both '%s'; disabling map->odom TF",
                map_frame_.c_str()
            );
            map_to_odom_tf_enabled_ = false;
        }
        if (!std::isfinite(map_to_odom_publish_rate_hz_) ||
            map_to_odom_publish_rate_hz_ <= 0.0)
            map_to_odom_publish_rate_hz_ = 20.0;
        get_parameter("kf_trans_thresh", kf_trans_thresh_);
        get_parameter("kf_rot_thresh", kf_rot_thresh_);
        get_parameter("min_kf_interval_sec", min_kf_interval_sec_);
        get_parameter("min_cloud_points", min_cloud_points_);
        get_parameter("loop_enable", loop_enable_);
        get_parameter("loop_min_keyframe_gap", loop_min_keyframe_gap_);
        get_parameter("loop_min_travel_distance", loop_min_travel_distance_);
        get_parameter("loop_iris_distance_thresh", loop_iris_distance_thresh_);
        get_parameter("loop_gicp_enable", loop_gicp_enable_);
        get_parameter("loop_gicp_score_thresh", loop_gicp_score_thresh_);
        get_parameter("loop_gicp_max_correction_trans", loop_gicp_max_correction_trans_);
        get_parameter("loop_gicp_max_correction_rot_deg", loop_gicp_max_correction_rot_deg_);
        get_parameter(
            "loop_gicp_selection_correction_trans_weight",
            loop_gicp_selection_correction_trans_weight_
        );
        get_parameter(
            "loop_gicp_selection_correction_rot_weight",
            loop_gicp_selection_correction_rot_weight_
        );
        get_parameter("loop_gicp_use_submap", loop_gicp_use_submap_);
        get_parameter("loop_gicp_submap_keyframes", loop_gicp_submap_keyframes_);
        get_parameter("loop_gicp_submap_leaf_size", loop_gicp_submap_leaf_size_);
        get_parameter("loop_gicp_num_threads", loop_gicp_num_threads_);
        get_parameter(
            "loop_gicp_correspondence_randomness",
            loop_gicp_correspondence_randomness_);
        get_parameter(
            "loop_gicp_max_correspondence_distance",
            loop_gicp_max_correspondence_distance_);
        get_parameter("loop_gicp_max_iterations", loop_gicp_max_iterations_);
        get_parameter(
            "loop_gicp_rotation_epsilon",
            loop_gicp_rotation_epsilon_);
        get_parameter(
            "loop_gicp_transformation_epsilon",
            loop_gicp_transformation_epsilon_);
        get_parameter("loop_gicp_min_inliers", loop_gicp_min_inliers_);
        get_parameter(
            "loop_gicp_min_inlier_ratio",
            loop_gicp_min_inlier_ratio_
        );
        get_parameter("body_frame", body_frame_);
        get_parameter("lidar_frame", lidar_frame_);
        get_parameter("gravity_align_enable", gravity_align_enable_);
        get_parameter("gravity_align_source", gravity_align_source_);
        get_parameter("gravity_align_imu_topic", gravity_align_imu_topic_);
        get_parameter("gravity_align_sample_sec", gravity_align_sample_sec_);
        get_parameter("gravity_align_acc_scale", gravity_align_acc_scale_);
        get_parameter(
            "gravity_align_manual_rpy_deg",
            gravity_align_manual_rpy_deg_
        );
        get_parameter(
            "gravity_align_apply_to_descriptor",
            gravity_align_apply_to_descriptor_
        );
        get_parameter("gravity_align_apply_to_pgo", gravity_align_apply_to_pgo_);
        get_parameter("gravity_align_apply_to_map", gravity_align_apply_to_map_);
        get_parameter("keyframe_exclusion_box_enable", keyframe_exclusion_box_enable_);
        get_parameter("keyframe_exclusion_min_x", keyframe_exclusion_min_x_);
        get_parameter("keyframe_exclusion_max_x", keyframe_exclusion_max_x_);
        get_parameter("keyframe_exclusion_min_y", keyframe_exclusion_min_y_);
        get_parameter("keyframe_exclusion_max_y", keyframe_exclusion_max_y_);
        get_parameter("keyframe_exclusion_min_z", keyframe_exclusion_min_z_);
        get_parameter("keyframe_exclusion_max_z", keyframe_exclusion_max_z_);
        get_parameter("pgo_enable", pgo_enable_);
        get_parameter("pgo_optimize_on_loop", pgo_optimize_on_loop_);
        get_parameter("pgo_max_iterations", pgo_max_iterations_);
        get_parameter("pgo_odom_edge_weight", pgo_odom_edge_weight_);
        get_parameter("pgo_loop_edge_weight", pgo_loop_edge_weight_);
        get_parameter("loop_edge_min_current_gap", loop_edge_min_current_gap_);
        get_parameter("loop_edge_min_travel_gap", loop_edge_min_travel_gap_);
        get_parameter("pgo_odom_info_diag", pgo_odom_info_diag_);
        get_parameter("pgo_loop_info_diag", pgo_loop_info_diag_);
        get_parameter(
            "pgo_loop_info_dynamic_enable",
            pgo_loop_info_dynamic_enable_
        );
        get_parameter("pgo_loop_info_score_ref", pgo_loop_info_score_ref_);
        get_parameter(
            "pgo_loop_info_score_floor",
            pgo_loop_info_score_floor_
        );
        get_parameter("pgo_loop_info_min_scale", pgo_loop_info_min_scale_);
        get_parameter("pgo_loop_info_max_scale", pgo_loop_info_max_scale_);
        get_parameter(
            "pgo_loop_robust_kernel_enable",
            pgo_loop_robust_kernel_enable_
        );
        get_parameter(
            "pgo_loop_robust_kernel_delta",
            pgo_loop_robust_kernel_delta_
        );
        get_parameter(
            "pgo_loop_measurement_mode",
            pgo_loop_measurement_mode_
        );
        get_parameter(
            "lcd_retrieval_enable",
            lcd_retrieval_enable_
        );
        get_parameter(
            "lcd_retrieval_top_k",
            lcd_retrieval_top_k_
        );
        get_parameter(
            "lcd_retrieval_max_distance",
            lcd_retrieval_max_distance_
        );
        get_parameter("iris_nscale", iris_nscale_);
        get_parameter("iris_min_wave_length", iris_min_wave_length_);
        get_parameter("iris_mult", iris_mult_);
        get_parameter("iris_sigma_onf", iris_sigma_onf_);
        get_parameter("iris_match_num", iris_match_num_);
        get_parameter("iris_submap_enable", iris_submap_enable_);
        get_parameter("iris_submap_keyframes", iris_submap_keyframes_);
        get_parameter("iris_submap_voxel_leaf_m", iris_submap_voxel_leaf_m_);
        get_parameter("cart_enable", cart_enable_);
        get_parameter("cart_x_unit_m", cart_x_unit_m_);
        get_parameter("cart_y_unit_m", cart_y_unit_m_);
        get_parameter("cart_x_max_m", cart_x_max_m_);
        get_parameter("cart_y_max_m", cart_y_max_m_);
        get_parameter("cart_voxel_leaf_m", cart_voxel_leaf_m_);
        get_parameter("cart_height_offset_m", cart_height_offset_m_);
        get_parameter("cart_use_align_key", cart_use_align_key_);
        get_parameter("cart_align_search_ratio", cart_align_search_ratio_);
        get_parameter("cart_candidate_top_k", cart_candidate_top_k_);
        get_parameter("loop_descriptor_verify_top_k", loop_descriptor_verify_top_k_);
        get_parameter("cart_distance_thresh", cart_distance_thresh_);
        get_parameter("optimized_keyframes_topic", optimized_keyframes_topic_);

        // 对会直接影响运行稳定性的参数做兜底，避免非法配置让
        // GICP、Cart Context 或 PGO 在运行中崩掉。
        if (loop_gicp_submap_keyframes_ < 1)
            loop_gicp_submap_keyframes_ = 1;
        if (!std::isfinite(loop_gicp_submap_leaf_size_) ||
            loop_gicp_submap_leaf_size_ < 0.0)
            loop_gicp_submap_leaf_size_ = 0.0;
        if (!std::isfinite(loop_gicp_max_correspondence_distance_) ||
            loop_gicp_max_correspondence_distance_ <= 0.0)
            loop_gicp_max_correspondence_distance_ = 1.0;
        if (!std::isfinite(loop_gicp_score_thresh_) ||
            loop_gicp_score_thresh_ <= 0.0)
            loop_gicp_score_thresh_ = 0.5;
        if (loop_gicp_max_iterations_ < 1)
            loop_gicp_max_iterations_ = 40;
        if (!std::isfinite(loop_gicp_rotation_epsilon_) ||
            loop_gicp_rotation_epsilon_ <= 0.0)
            loop_gicp_rotation_epsilon_ = 2e-3;
        if (!std::isfinite(loop_gicp_transformation_epsilon_) ||
            loop_gicp_transformation_epsilon_ <= 0.0)
            loop_gicp_transformation_epsilon_ = 1e-3;
        if (iris_submap_keyframes_ < 1)
            iris_submap_keyframes_ = 1;
        if (!std::isfinite(iris_submap_voxel_leaf_m_) ||
            iris_submap_voxel_leaf_m_ < 0.0)
            iris_submap_voxel_leaf_m_ = 0.0;
        if (!std::isfinite(loop_gicp_selection_correction_trans_weight_) ||
            loop_gicp_selection_correction_trans_weight_ < 0.0)
            loop_gicp_selection_correction_trans_weight_ = 0.0;
        if (!std::isfinite(loop_gicp_selection_correction_rot_weight_) ||
            loop_gicp_selection_correction_rot_weight_ < 0.0)
            loop_gicp_selection_correction_rot_weight_ = 0.0;
        if (loop_gicp_min_inliers_ < 0)
            loop_gicp_min_inliers_ = 0;
        if (!std::isfinite(loop_gicp_min_inlier_ratio_) ||
            loop_gicp_min_inlier_ratio_ < 0.0)
            loop_gicp_min_inlier_ratio_ = 0.0;
        loop_gicp_min_inlier_ratio_ =
            std::min(loop_gicp_min_inlier_ratio_, 1.0);
        if (!std::isfinite(cart_x_unit_m_) || cart_x_unit_m_ <= 0.0)
            cart_x_unit_m_ = 0.5;
        if (!std::isfinite(cart_y_unit_m_) || cart_y_unit_m_ <= 0.0)
            cart_y_unit_m_ = 0.1;
        if (!std::isfinite(cart_x_max_m_) || cart_x_max_m_ <= 0.0)
            cart_x_max_m_ = 50.0;
        if (!std::isfinite(cart_y_max_m_) || cart_y_max_m_ <= 0.0)
            cart_y_max_m_ = 25.0;
        if (!std::isfinite(cart_voxel_leaf_m_) || cart_voxel_leaf_m_ < 0.0)
            cart_voxel_leaf_m_ = 0.0;
        if (!std::isfinite(cart_align_search_ratio_) ||
            cart_align_search_ratio_ <= 0.0)
            cart_align_search_ratio_ = 1.0;
        if (cart_candidate_top_k_ < 1)
            cart_candidate_top_k_ = 1;
        if (loop_descriptor_verify_top_k_ < 1)
            loop_descriptor_verify_top_k_ = 1;
        loop_descriptor_verify_top_k_ =
            std::min(loop_descriptor_verify_top_k_, cart_candidate_top_k_);
        if (lcd_retrieval_top_k_ < 1)
            lcd_retrieval_top_k_ = 1;
        if (!std::isfinite(lcd_retrieval_max_distance_))
            lcd_retrieval_max_distance_ = -1.0;
        if (!std::isfinite(cart_distance_thresh_) ||
            cart_distance_thresh_ <= 0.0)
            cart_distance_thresh_ = std::numeric_limits<double>::infinity();
        // 对一定容器范围内元素调用特定函数
        std::transform(
            gravity_align_source_.begin(),
            gravity_align_source_.end(),
            gravity_align_source_.begin(),
            [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            }
        );
        if (gravity_align_source_ != "imu_average" &&
            gravity_align_source_ != "manual_rpy") {

            RCLCPP_WARN(
                get_logger(),
                "gravity_align_source must be imu_average or manual_rpy, got '%s'; using imu_average",
                gravity_align_source_.c_str()
            );
            gravity_align_source_ = "imu_average";
        }
        if (!std::isfinite(gravity_align_sample_sec_) ||
            gravity_align_sample_sec_ <= 0.0)
            gravity_align_sample_sec_ = 5.0;
        if (!std::isfinite(gravity_align_acc_scale_) ||
            gravity_align_acc_scale_ <= 0.0)
            gravity_align_acc_scale_ = 1.0;
        if (gravity_align_manual_rpy_deg_.size() != 3) {

            RCLCPP_WARN(
                get_logger(),
                "gravity_align_manual_rpy_deg must contain [roll,pitch,yaw] degrees; using zero"
            );
            gravity_align_manual_rpy_deg_ = {0.0, 0.0, 0.0};
        }
        if (gravity_align_apply_to_pgo_)
            gravity_align_apply_to_map_ = true;
        if (gravity_align_enable_ &&
            gravity_align_source_ == "manual_rpy")
            configureManualGravityAlignment();
        if (loop_edge_min_current_gap_ < 0)
            loop_edge_min_current_gap_ = 0;
        if (!std::isfinite(loop_edge_min_travel_gap_) ||
            loop_edge_min_travel_gap_ < 0.0)
            loop_edge_min_travel_gap_ = 0.0;
        if (!std::isfinite(pgo_loop_info_score_ref_) ||
            pgo_loop_info_score_ref_ <= 0.0)
            pgo_loop_info_score_ref_ = 0.5;
        if (!std::isfinite(pgo_loop_info_score_floor_) ||
            pgo_loop_info_score_floor_ <= 0.0)
            pgo_loop_info_score_floor_ = 0.05;
        if (!std::isfinite(pgo_loop_info_min_scale_) ||
            pgo_loop_info_min_scale_ <= 0.0)
            pgo_loop_info_min_scale_ = 0.2;
        if (!std::isfinite(pgo_loop_info_max_scale_) ||
            pgo_loop_info_max_scale_ <= 0.0)
            pgo_loop_info_max_scale_ = 2.0;
        if (pgo_loop_info_min_scale_ > pgo_loop_info_max_scale_)
            std::swap(pgo_loop_info_min_scale_, pgo_loop_info_max_scale_);
        if (!std::isfinite(pgo_loop_robust_kernel_delta_) ||
            pgo_loop_robust_kernel_delta_ <= 0.0)
            pgo_loop_robust_kernel_delta_ = 1.0;
        std::transform(
            pgo_loop_measurement_mode_.begin(),
            pgo_loop_measurement_mode_.end(),
            pgo_loop_measurement_mode_.begin(),
            [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            }
        );
        if (pgo_loop_measurement_mode_ != "gravity_preserving" &&
            pgo_loop_measurement_mode_ != "full_se3") {

            RCLCPP_WARN(
                get_logger(),
                "pgo_loop_measurement_mode must be gravity_preserving or full_se3, "
                "got '%s'; using gravity_preserving",
                pgo_loop_measurement_mode_.c_str()
            );
            pgo_loop_measurement_mode_ = "gravity_preserving";
        }
        const bool exclusion_box_valid =
            std::isfinite(keyframe_exclusion_min_x_) &&
            std::isfinite(keyframe_exclusion_max_x_) &&
            std::isfinite(keyframe_exclusion_min_y_) &&
            std::isfinite(keyframe_exclusion_max_y_) &&
            std::isfinite(keyframe_exclusion_min_z_) &&
            std::isfinite(keyframe_exclusion_max_z_) &&
            keyframe_exclusion_min_x_ <= keyframe_exclusion_max_x_ &&
            keyframe_exclusion_min_y_ <= keyframe_exclusion_max_y_ &&
            keyframe_exclusion_min_z_ <= keyframe_exclusion_max_z_;
        if (keyframe_exclusion_box_enable_ && !exclusion_box_valid) {

            RCLCPP_WARN(
                get_logger(),
                "keyframe exclusion box is invalid; disabling cloud exclusion"
            );
            keyframe_exclusion_box_enable_ = false;
        }

        auto read_body_lidar_extrinsic = [this]() {

            // 兼容两种外参写法：t/q 或完整 4x4 矩阵；矩阵优先级更高。
            const std::vector<double> t_default = {0.0, 0.0, 0.0};
            const std::vector<double> q_default = {1.0, 0.0, 0.0, 0.0};
            std::vector<double> t, q, matrix;

            declare_parameter("t_body_lidar", t_default);
            get_parameter("t_body_lidar", t);
            declare_parameter("q_body_lidar", q_default);
            get_parameter("q_body_lidar", q);
            declare_parameter("T_body_lidar", std::vector<double>{});
            get_parameter("T_body_lidar", matrix);

            Eigen::Vector3d translation = Eigen::Vector3d::Zero();
            Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

            // check
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

            // write
            T_body_lidar_ = Eigen::Isometry3d::Identity();
            T_body_lidar_.linear() = rotation.toRotationMatrix();
            T_body_lidar_.translation() = translation;
            T_lidar_body_ = T_body_lidar_.inverse();

            RCLCPP_INFO(
                get_logger(),
                "Loop backend T_body_lidar: t=[%.6f %.6f %.6f] q=[%.6f %.6f %.6f %.6f]",
                translation.x(), translation.y(), translation.z(),
                rotation.w(), rotation.x(), rotation.y(), rotation.z()
            );
        };

        read_body_lidar_extrinsic();

        const auto odom_info = makeInformationMatrix(
            pgo_odom_info_diag_,
            pgo_odom_edge_weight_,
            "pgo_odom_info_diag"
        );
        const auto loop_info = makeInformationMatrix(
            pgo_loop_info_diag_,
            pgo_loop_edge_weight_,
            "pgo_loop_info_diag"
        );
        RCLCPP_INFO(
            get_logger(),
            "PGO info diag order=[x y z roll pitch yaw], odom=[%.3f %.3f %.3f %.3f %.3f %.3f], "
            "loop=[%.3f %.3f %.3f %.3f %.3f %.3f]",
            odom_info(0, 0), odom_info(1, 1), odom_info(2, 2),
            odom_info(3, 3), odom_info(4, 4), odom_info(5, 5),
            loop_info(0, 0), loop_info(1, 1), loop_info(2, 2),
            loop_info(3, 3), loop_info(4, 4), loop_info(5, 5)
        );
        RCLCPP_INFO(
            get_logger(),
            "PGO loop weighting: dynamic=%d score_ref=%.3f score_floor=%.3f "
            "scale=[%.3f %.3f] robust_kernel=%d robust_delta=%.3f",
            pgo_loop_info_dynamic_enable_ ? 1 : 0,
            pgo_loop_info_score_ref_,
            pgo_loop_info_score_floor_,
            pgo_loop_info_min_scale_,
            pgo_loop_info_max_scale_,
            pgo_loop_robust_kernel_enable_ ? 1 : 0,
            pgo_loop_robust_kernel_delta_
        );
        RCLCPP_INFO(
            get_logger(),
            "Loop GICP submap: enabled=%d keyframes=%d leaf=%.3f",
            loop_gicp_use_submap_ ? 1 : 0,
            loop_gicp_submap_keyframes_,
            loop_gicp_submap_leaf_size_
        );
        RCLCPP_INFO(
            get_logger(),
            "Loop GICP selection cost: score + %.3f*odom_delta_t + %.3f*odom_delta_r_deg",
            loop_gicp_selection_correction_trans_weight_,
            loop_gicp_selection_correction_rot_weight_
        );
        RCLCPP_INFO(
            get_logger(),
            "Loop edge sparsification: min_current_gap=%d min_travel_gap=%.3f",
            loop_edge_min_current_gap_,
            loop_edge_min_travel_gap_
        );
        RCLCPP_INFO(
            get_logger(),
            "Cart Context: enabled=%d unit=[%.3f %.3f] range=[%.1f %.1f] "
            "voxel=%.3f align_key=%d align_ratio=%.3f "
            "retrieval=%d retrieval_top_k=%d retrieval_max_dist=%.3f "
            "iris_top_k=%d verify_top_k=%d cart_thresh=%.3f",
            cart_enable_ ? 1 : 0,
            cart_x_unit_m_,
            cart_y_unit_m_,
            cart_x_max_m_,
            cart_y_max_m_,
            cart_voxel_leaf_m_,
            cart_use_align_key_ ? 1 : 0,
            cart_align_search_ratio_,
            lcd_retrieval_enable_ ? 1 : 0,
            lcd_retrieval_top_k_,
            lcd_retrieval_max_distance_,
            cart_candidate_top_k_,
            loop_descriptor_verify_top_k_,
            cart_distance_thresh_
        );
        RCLCPP_INFO(
            get_logger(),
            "Keyframe exclusion box: enabled=%d x=[%.3f %.3f] y=[%.3f %.3f] z=[%.3f %.3f]",
            keyframe_exclusion_box_enable_ ? 1 : 0,
            keyframe_exclusion_min_x_,
            keyframe_exclusion_max_x_,
            keyframe_exclusion_min_y_,
            keyframe_exclusion_max_y_,
            keyframe_exclusion_min_z_,
            keyframe_exclusion_max_z_
        );
    }

    void LoopDetectorNode::callbackKeyFrame(
        const small_point_lio_pgo::msg::KeyFrame::SharedPtr msg
    ) {

        try {

        if (!msg) return;
        ++ received_keyframes_;

        // 位姿来源（直接 odom）：msg->pose 是输入关键帧携带的 T_odom_body。
        // 本节点不通过 TF 读取车体位姿，此处尚未应用 map->odom 修正。
        if (!msg->header.frame_id.empty() &&
            msg->header.frame_id != odom_frame_) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Keyframe pose frame is '%s', expected odom_frame '%s'; "
                "treating the pose as odom regardless",
                msg->header.frame_id.c_str(),
                odom_frame_.c_str()
            );
        }

        // 1. 先做关键帧门控：位移/角度/时间间隔/点数不足的候选不进入后端。
        if (!shouldAcceptKeyFrameCandidate(*msg)) {

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skipped keyframe candidate: candidate_id=%u stored=%lu",
                msg->id,
                stored_keyframes_
            );
            return;
        }

        // 保留输入 keyframe 的原始 id。map_node 按这个 id 存点云，
        // PGO 发布 optimized_keyframes 时也必须使用同一套 id，否则优化位姿
        // 会套到错误的点云上，地图会在首次 PGO 后大幅错位。
        small_point_lio_pgo::msg::KeyFrame accepted_msg = *msg;
        const rclcpp::Time timing_stamp(accepted_msg.header.stamp);

        if (gravity_align_enable_ &&
            (gravity_align_apply_to_descriptor_ ||
             gravity_align_apply_to_pgo_ ||
             gravity_align_apply_to_map_) &&
            !gravityAlignmentReady()) {

            size_t align_samples = 0;
            double align_span = 0.0;
            {
                std::lock_guard<std::mutex> lock(gravity_align_mutex_);
                align_samples = gravity_align_sample_count_;
                align_span = std::max(
                    0.0,
                    gravity_align_latest_stamp_ - gravity_align_first_stamp_
                );
            }
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for gravity alignment before accepting keyframes: id=%u "
                "source=%s samples=%zu span=%.3f/%.3f sec",
                accepted_msg.id,
                gravity_align_source_.c_str(),
                align_samples,
                align_span,
                gravity_align_sample_sec_
            );
            return;
        }

        // 2. 可选地裁掉指定空间盒内的点，再转换成后端内部 LoopKeyFrame。
        if (!filterKeyFrameMsgCloud(accepted_msg)) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Rejected keyframe after exclusion-box filtering: id=%u",
                accepted_msg.id
            );
            return;
        }

        LoopKeyFrame keyframe;
        if (!convertKeyFrameMsg(accepted_msg, keyframe)) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Rejected invalid keyframe message: id=%u",
                accepted_msg.id
            );
            return;
        }

        // Keep the forwarded keyframe in the frontend odom frame. A global
        // gravity/PGO correction belongs in map->odom and in the optimized
        // keyframe snapshot; putting it into this raw map input would make
        // the map and TF disagree before the first PGO snapshot.

        // 3. 为当前关键帧计算 LiDAR-Iris 和 Cart Context。描述子失败时，
        // 当前帧不会进入回环库，也不会作为 PGO 节点。
        const auto descriptor_start = SteadyClock::now();
        if (!computeIrisDescriptor(keyframe)) {

            ++ iris_failures_;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed to compute LiDAR-Iris descriptor: id=%u cloud_size=%zu",
                keyframe.id,
                keyframe.cloud ? keyframe.cloud->size() : 0U
            );
            return;
        }
        if (!computeCartDescriptor(keyframe)) {

            ++ iris_failures_;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed to compute Cart Context descriptor: id=%u cloud_size=%zu",
                keyframe.id,
                keyframe.cloud ? keyframe.cloud->size() : 0U
            );
            return;
        }
        const double descriptor_ms =
            elapsedMs(descriptor_start, SteadyClock::now());

        if (pub_filtered_keyframe_)
            pub_filtered_keyframe_->publish(accepted_msg);

        keyframe.travel_distance = accumulated_travel_distance_;
        if (!keyframes_.empty()) {

            const Eigen::Vector3d previous(
                keyframes_.back().pose.position.x,
                keyframes_.back().pose.position.y,
                keyframes_.back().pose.position.z
            );
            const Eigen::Vector3d current(
                keyframe.pose.position.x,
                keyframe.pose.position.y,
                keyframe.pose.position.z
            );
            const double step = (current - previous).norm();
            if (std::isfinite(step))
                keyframe.travel_distance += step;
        }

        const bool pgo_node_added =
            pgo_enable_ && addKeyFrameToPoseGraph(keyframe);

        // 4. LCD 先按描述子选出候选，再用 GICP 验证，最后只保留
        // 选择代价最低的一个候选进入 loop edge 阶段。
        bool added_loop_edge = false;
        LoopCandidate accepted_candidate;
        bool has_accepted_candidate = false;
        double loop_detect_ms = 0.0;
        if (loop_enable_) {

            const auto loop_start = SteadyClock::now();
            std::vector<LoopCandidate> candidates;
            float best_distance = std::numeric_limits<float>::infinity();
            float best_cart_distance = std::numeric_limits<float>::infinity();
            int eligible_count = 0;
            if (detectLoopCandidates(
                    keyframe,
                    candidates,
                    best_distance,
                    best_cart_distance,
                    eligible_count)) {

                size_t verified_count = 0U;
                for (auto &candidate : candidates) {

                    const bool verified =
                        !loop_gicp_enable_ ||
                        verifyLoopCandidateByGicp(keyframe, candidate);
                    if (!verified)
                        continue;
                    ++ verified_count;

                    if (!has_accepted_candidate ||
                        candidate.gicp_selection_cost <
                            accepted_candidate.gicp_selection_cost ||
                        (!std::isfinite(accepted_candidate.gicp_selection_cost) &&
                         candidate.gicp_score < accepted_candidate.gicp_score)) {

                        accepted_candidate = candidate;
                        has_accepted_candidate = true;
                    }
                }

                if (has_accepted_candidate) {

                    ++ candidate_hits_;
                    RCLCPP_INFO(
                        get_logger(),
                        "Loop candidate selected: current=%u history=%u iris=%.4f "
                        "cart=%.4f cart_shift=%d cart_lat=%.3f "
                        "yaw_bias=%d gicp_enable=%d gicp_score=%.4f cost=%.4f "
                        "corr_t=%.3f corr_r=%.2f descriptor_candidates=%zu "
                        "gicp_verified=%zu",
                        accepted_candidate.current_id,
                        accepted_candidate.history_id,
                        accepted_candidate.iris_distance,
                        accepted_candidate.cart_distance,
                        accepted_candidate.cart_shift_cols,
                        accepted_candidate.cart_lateral_offset_m,
                        accepted_candidate.yaw_bias,
                        loop_gicp_enable_ ? 1 : 0,
                        accepted_candidate.gicp_score,
                        accepted_candidate.gicp_selection_cost,
                        accepted_candidate.gicp_correction_trans,
                        accepted_candidate.gicp_correction_rot_deg,
                        candidates.size(),
                        verified_count
                    );
                } else {

                    ++ candidate_misses_;
                }
            } else {

                ++ candidate_misses_;
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Loop candidate miss: current=%u eligible=%d best_iris=%.4f best_cart=%.4f "
                    "iris_thresh=%.4f cart_thresh=%.4f stored=%zu received=%lu iris_fail=%lu "
                    "hits=%lu misses=%lu",
                    keyframe.id,
                    eligible_count,
                    best_distance,
                    best_cart_distance,
                    loop_iris_distance_thresh_,
                    cart_distance_thresh_,
                    keyframes_.size(),
                    received_keyframes_,
                    iris_failures_,
                    candidate_hits_,
                    candidate_misses_
                );
            }
            loop_detect_ms = elapsedMs(loop_start, SteadyClock::now());
        }
        const double lcd_ms = descriptor_ms + loop_detect_ms;

        // 5. 当前关键帧在检测结束后才入库，避免本帧参与自己的历史候选搜索。
        storeKeyFrame(std::move(keyframe));
        ++ stored_keyframes_;

        if (has_accepted_candidate) {

            loop_candidates_.push_back(accepted_candidate);
            if (pgo_node_added && pgo_enable_) {

                // 6. 对已经通过 GICP 的候选再做稀疏化，避免同一片区域
                // 连续添加大量相似 loop edge。
                int current_gap = 0;
                double travel_gap = 0.0;
                // 使用帧间距离/ keyframe 数量决定是否添加该 loop edge
                if (shouldAddLoopEdgeToPoseGraph(
                        accepted_candidate,
                        current_gap,
                        travel_gap
                    )
                ) {

                    added_loop_edge =
                        addLoopCandidateToPoseGraph(accepted_candidate);
                    if (added_loop_edge) {

                        has_last_added_loop_edge_ = true;
                        last_added_loop_current_id_ =
                            accepted_candidate.current_id;
                        const auto *current =
                            findKeyFrame(accepted_candidate.current_id);
                        if (current)
                            last_added_loop_current_travel_distance_ =
                                current->travel_distance;
                    }
                } else {

                    RCLCPP_INFO(
                        get_logger(),
                        "Skipped loop edge by sparsification: current=%u history=%u "
                        "current_gap=%d/%d travel_gap=%.3f/%.3f",
                        accepted_candidate.current_id,
                        accepted_candidate.history_id,
                        current_gap,
                        loop_edge_min_current_gap_,
                        travel_gap,
                        loop_edge_min_travel_gap_
                    );
                }
            }
        }

        // 7. PGO 默认只在新增 loop edge 后触发；优化结果随后发布给
        // path 和 map_node 重建使用。
        double pgo_ms = 0.0;
        if (pgo_enable_)
            pgo_ms = optimizePoseGraphIfNeeded(added_loop_edge);

        publishTiming(lcd_ms, pgo_ms, timing_stamp);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Loop detector status: received=%lu stored=%lu graph_nodes=%zu "
            "graph_edges=%zu candidates=%zu iris_fail=%lu lcd_ms=%.3f pgo_ms=%.3f",
            received_keyframes_,
            stored_keyframes_,
            pose_graph_.nodeCount(),
            pose_graph_.edgeCount(),
            loop_candidates_.size(),
            iris_failures_,
            lcd_ms,
            pgo_ms
        );
        publishLoopMarkers();
        if (pgo_enable_)
            publishOptimizedPath();
        } catch (const cv::Exception &e) {

            ++ iris_failures_;
            RCLCPP_ERROR(
                get_logger(),
                "OpenCV exception in loop detector keyframe callback: id=%u what=%s",
                msg ? msg->id : 0U,
                e.what()
            );
        } catch (const std::exception &e) {

            RCLCPP_ERROR(
                get_logger(),
                "Exception in loop detector keyframe callback: id=%u what=%s",
                msg ? msg->id : 0U,
                e.what()
            );
        }
    }

    size_t LoopDetectorNode::pointCount(
        const sensor_msgs::msg::PointCloud2 &cloud
    ) {

        return static_cast<size_t>(cloud.width) *
            static_cast<size_t>(cloud.height);
    }

    std::pair<double, double> LoopDetectorNode::poseDelta(
        const geometry_msgs::msg::Pose &from,
        const geometry_msgs::msg::Pose &to
    ) const {

        const Eigen::Vector3d p_from(
            from.position.x,
            from.position.y,
            from.position.z
        );
        const Eigen::Vector3d p_to(
            to.position.x,
            to.position.y,
            to.position.z
        );

        Eigen::Quaterniond q_from(
            from.orientation.w,
            from.orientation.x,
            from.orientation.y,
            from.orientation.z
        );
        Eigen::Quaterniond q_to(
            to.orientation.w,
            to.orientation.x,
            to.orientation.y,
            to.orientation.z
        );

        if (q_from.norm() < 1e-9 || q_to.norm() < 1e-9)
            return {
                std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()
            };
        q_from.normalize();
        q_to.normalize();

        Eigen::Quaterniond dq = (q_from.conjugate() * q_to).normalized();
        if (dq.w() < 0.0)
            dq.coeffs() *= -1.0;

        return {
            (p_to - p_from).norm(),
            Eigen::AngleAxisd(dq).angle()
        };
    }

    bool LoopDetectorNode::shouldAcceptKeyFrameCandidate(
        const small_point_lio_pgo::msg::KeyFrame &msg
    ) const {

        if (msg.cloud.data.empty() ||
            pointCount(msg.cloud) < static_cast<size_t>(min_cloud_points_))
            return false;

        if (!std::isfinite(msg.pose.position.x) ||
            !std::isfinite(msg.pose.position.y) ||
            !std::isfinite(msg.pose.position.z) ||
            !std::isfinite(msg.pose.orientation.w) ||
            !std::isfinite(msg.pose.orientation.x) ||
            !std::isfinite(msg.pose.orientation.y) ||
            !std::isfinite(msg.pose.orientation.z))
            return false;

        if (!has_last_accepted_keyframe_)
            return true;

        double dt = std::numeric_limits<double>::infinity();
        if (std::isfinite(min_kf_interval_sec_) &&
            min_kf_interval_sec_ > 0.0) {

            dt =
                (rclcpp::Time(msg.header.stamp) -
                 last_accepted_keyframe_stamp_).seconds();
            if (std::isfinite(dt) && dt >= 0.0 &&
                dt < min_kf_interval_sec_)
                return false;
        }

        // 位姿来源（直接 odom）：两个操作数都是关键帧消息内的
        // T_odom_body，不通过 TF 读取，也不含 map->odom 修正。若拿前一帧的
        // 重力对齐 pose 和当前 raw pose 比较，固定对齐量会被误算成运动。
        const auto delta = poseDelta(
            last_accepted_keyframe_raw_pose_,
            msg.pose
        );
        if (!std::isfinite(delta.first) || !std::isfinite(delta.second))
            return false;

        const bool accepted =
            delta.first >= kf_trans_thresh_ ||
            delta.second >= kf_rot_thresh_;
        if (accepted) {

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Keyframe gate accepted: id=%u dt=%.3f trans=%.3f/%.3f "
                "rot_deg=%.2f/%.2f",
                msg.id,
                dt,
                delta.first,
                kf_trans_thresh_,
                delta.second * 180.0 / M_PI,
                kf_rot_thresh_ * 180.0 / M_PI
            );
        }
        return accepted;
    }

    bool LoopDetectorNode::filterKeyFrameMsgCloud(
        small_point_lio_pgo::msg::KeyFrame &msg
    ) const {

        if (!keyframe_exclusion_box_enable_)
            return true;

        auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(msg.cloud, *cloud_in);
        if (cloud_in->empty())
            return false;

        auto cloud_body = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        const std::string &frame_id = msg.cloud.header.frame_id;
        if (frame_id == lidar_frame_) {

            pcl::transformPointCloud(
                *cloud_in,
                *cloud_body,
                T_body_lidar_.matrix().cast<float>()
            );
        } else {

            if (!frame_id.empty() && frame_id != body_frame_) {

                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Keyframe exclusion input frame '%s' is neither '%s' nor '%s'; treating it as body frame",
                    frame_id.c_str(),
                    body_frame_.c_str(),
                    lidar_frame_.c_str()
                );
            }
            cloud_body = cloud_in;
        }

        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        filtered->reserve(cloud_body->size());
        for (const auto &point : cloud_body->points) {

            if (!std::isfinite(point.x) ||
                !std::isfinite(point.y) ||
                !std::isfinite(point.z))
                continue;

            const bool inside_box =
                point.x >= keyframe_exclusion_min_x_ &&
                point.x <= keyframe_exclusion_max_x_ &&
                point.y >= keyframe_exclusion_min_y_ &&
                point.y <= keyframe_exclusion_max_y_ &&
                point.z >= keyframe_exclusion_min_z_ &&
                point.z <= keyframe_exclusion_max_z_;
            if (!inside_box)
                filtered->push_back(point);
        }

        filtered->width = static_cast<uint32_t>(filtered->size());
        filtered->height = 1;
        filtered->is_dense = false;
        if (filtered->size() < static_cast<size_t>(min_cloud_points_))
            return false;

        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered, filtered_msg);
        filtered_msg.header = msg.cloud.header;
        filtered_msg.header.frame_id = body_frame_;
        msg.cloud = std::move(filtered_msg);

        const size_t removed = cloud_body->size() - filtered->size();
        if (removed > 0U) {

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Keyframe exclusion box removed %zu/%zu points: id=%u box x=[%.2f %.2f] y=[%.2f %.2f] z=[%.2f %.2f]",
                removed,
                cloud_body->size(),
                msg.id,
                keyframe_exclusion_min_x_,
                keyframe_exclusion_max_x_,
                keyframe_exclusion_min_y_,
                keyframe_exclusion_max_y_,
                keyframe_exclusion_min_z_,
                keyframe_exclusion_max_z_
            );
        }

        return true;
    }

    /**
     * @brief IMU 数据回调函数
     * @param msg imu 数据消息
     * @return void
     *
     * 主要用于 LCD 时对齐重力
     */
    void LoopDetectorNode::callbackImu(
        const sensor_msgs::msg::Imu::SharedPtr msg
    ) {

        if (!msg || !gravity_align_enable_ ||
            gravity_align_source_ != "imu_average")
            return;

        const Eigen::Vector3d acc(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );
        if (!acc.allFinite() || acc.norm() < 1e-9)
            return;

        double stamp =
            msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (!std::isfinite(stamp) || stamp <= 0.0)
            stamp = now().seconds();

        Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
        size_t sample_count = 0;
        double sample_span = 0.0;
        bool should_finalize = false;
        {
            std::lock_guard<std::mutex> lock(gravity_align_mutex_);
            if (gravity_align_ready_)
                return;
            if (!gravity_align_started_) {

                gravity_align_started_ = true;
                gravity_align_first_stamp_ = stamp;
                gravity_align_latest_stamp_ = stamp;
                gravity_align_acc_mean_.setZero();
                gravity_align_sample_count_ = 0;
                RCLCPP_INFO(
                    get_logger(),
                    "Gravity alignment IMU averaging started: topic=%s sample_sec=%.3f",
                    gravity_align_imu_topic_.c_str(),
                    gravity_align_sample_sec_
                );
            }

            ++ gravity_align_sample_count_;
            gravity_align_latest_stamp_ = stamp;
            gravity_align_acc_mean_ +=
                (acc - gravity_align_acc_mean_) /
                static_cast<double>(gravity_align_sample_count_);
            sample_count = gravity_align_sample_count_;
            acc_mean = gravity_align_acc_mean_;
            sample_span = std::max(
                0.0,
                gravity_align_latest_stamp_ - gravity_align_first_stamp_
            );
            should_finalize = sample_span >= gravity_align_sample_sec_;
        }

        if (should_finalize) {

            finalizeGravityAlignmentFromAcceleration(
                acc_mean,
                sample_count,
                sample_span,
                "imu_average"
            );
        }
    }

    bool LoopDetectorNode::gravityAlignmentReady() const {

        if (!gravity_align_enable_)
            return true;
        std::lock_guard<std::mutex> lock(gravity_align_mutex_);
        return gravity_align_ready_;
    }

    // 对齐变换
    Eigen::Isometry3d LoopDetectorNode::gravityAlignmentTransform() const {

        std::lock_guard<std::mutex> lock(gravity_align_mutex_);
        return T_gravity_align_;
    }

    // 根据标准值对齐重力
    void LoopDetectorNode::configureManualGravityAlignment() {

        const Eigen::Matrix3d R_align =
            rotationFromRpyDeg(gravity_align_manual_rpy_deg_);
        if (!R_align.allFinite()) {

            RCLCPP_WARN(
                get_logger(),
                "gravity_align_manual_rpy_deg is invalid; disabling gravity alignment"
            );
            gravity_align_enable_ = false;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(gravity_align_mutex_);
            T_gravity_align_ = Eigen::Isometry3d::Identity();
            T_gravity_align_.linear() = R_align;
            gravity_align_ready_ = true;
        }

        const Eigen::Vector3d rpy = rpyDegFromRotation(R_align);
        RCLCPP_INFO(
            get_logger(),
            "Gravity alignment manual_rpy ready: rpy_deg=[%.3f %.3f %.3f]",
            rpy.x(), rpy.y(), rpy.z()
        );
    }

    /**
     * @brief 使用初始累计平均值做重力对齐变换
     * @param acc_mean 加速度累计平均值
     * @param sample_count 采样数量
     * @param sample_span_sec 采样持续时间
     * @param source 模式
     * @return bool 是否成功
     */
    bool LoopDetectorNode::finalizeGravityAlignmentFromAcceleration(
        const Eigen::Vector3d &acc_mean,
        const size_t sample_count,
        const double sample_span_sec,
        const char *source
    ) {

        const double acc_norm = acc_mean.norm();
        if (!acc_mean.allFinite() || acc_norm < 1e-9) {

            RCLCPP_WARN(
                get_logger(),
                "Gravity alignment failed: invalid acc_mean=[%.6f %.6f %.6f]",
                acc_mean.x(), acc_mean.y(), acc_mean.z()
            );
            return false;
        }

        const Eigen::Vector3d measured = acc_mean / acc_norm;
        const Eigen::Vector3d target = Eigen::Vector3d::UnitZ();
        const Eigen::Vector3d axis = measured.cross(target);
        const double axis_norm = axis.norm();
        const double cos_angle =
            std::clamp(measured.dot(target), -1.0, 1.0);
        Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();
        if (axis_norm < 1e-9) {

            if (cos_angle < 0.0) {

                R_align = Eigen::AngleAxisd(
                    M_PI,
                    Eigen::Vector3d::UnitX()
                ).toRotationMatrix();
            }
        } else {

            const Eigen::Matrix3d K =
                (Eigen::Matrix3d() <<
                    0.0, -axis.z(), axis.y(),
                    axis.z(), 0.0, -axis.x(),
                    -axis.y(), axis.x(), 0.0
                ).finished();
            R_align = Eigen::Matrix3d::Identity() +
                K +
                K * K * ((1.0 - cos_angle) / (axis_norm * axis_norm));
        }
        const Eigen::Vector3d rpy = rpyDegFromRotation(R_align);
        const double tilt_deg = std::acos(cos_angle) * 180.0 / M_PI;

        {
            std::lock_guard<std::mutex> lock(gravity_align_mutex_);
            if (gravity_align_ready_)
                return true;
            T_gravity_align_ = Eigen::Isometry3d::Identity();
            T_gravity_align_.linear() = R_align;
            gravity_align_ready_ = true;
        }

        RCLCPP_INFO(
            get_logger(),
            "Gravity alignment ready: source=%s samples=%zu span=%.3f "
            "acc_mean_raw=[%.6f %.6f %.6f] acc_norm_raw=%.6f "
            "acc_norm_scaled=%.6f tilt_deg=%.3f align_rpy_deg=[%.3f %.3f %.3f]",
            source ? source : "unknown",
            sample_count,
            sample_span_sec,
            acc_mean.x(), acc_mean.y(), acc_mean.z(),
            acc_norm,
            acc_norm * gravity_align_acc_scale_,
            tilt_deg,
            rpy.x(), rpy.y(), rpy.z()
        );
        return true;
    }

    // 对齐位姿
    Eigen::Isometry3d LoopDetectorNode::alignedPose(
        const geometry_msgs::msg::Pose &raw_pose
    ) const {

        const Eigen::Isometry3d raw = poseToIsometry(raw_pose);
        if (!gravity_align_enable_ || !raw.matrix().allFinite())
            return raw;
        return gravityAlignmentTransform() * raw;
    }

    // 将 Keyframe 的点云也进行重力对齐
    pcl::PointCloud<pcl::PointXYZ>::Ptr LoopDetectorNode::makeDescriptorCloud(
        const LoopKeyFrame &keyframe
    ) const {

        return makeDescriptorCloud(keyframe, keyframe.cloud);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr LoopDetectorNode::makeDescriptorCloud(
        const LoopKeyFrame &keyframe,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud
    ) const {

        if (!source_cloud || source_cloud->empty())
            return source_cloud;
        if (!gravity_align_enable_ ||
            !gravity_align_apply_to_descriptor_ ||
            !gravityAlignmentReady())
            return source_cloud;

        const Eigen::Isometry3d T_aligned_world_body =
            alignedPose(keyframe.raw_pose);
        if (!T_aligned_world_body.matrix().allFinite())
            return source_cloud;

        Eigen::Matrix4d R_descriptor = Eigen::Matrix4d::Identity();
        R_descriptor.block<3, 3>(0, 0) =
            T_aligned_world_body.rotation() *
            T_body_lidar_.rotation();

        auto cloud_aligned = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(
            *source_cloud,
            *cloud_aligned,
            R_descriptor.cast<float>()
        );
        cloud_aligned->width = static_cast<uint32_t>(cloud_aligned->size());
        cloud_aligned->height = 1;
        cloud_aligned->is_dense = false;
        return cloud_aligned;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr LoopDetectorNode::buildIrisSubmapCloud(
        const LoopKeyFrame &anchor,
        size_t &frames_used,
        size_t &input_points,
        double &span_m
    ) const {

        frames_used = 0U;
        input_points = 0U;
        span_m = 0.0;
        if (!anchor.cloud || anchor.cloud->empty())
            return anchor.cloud;

        auto submap = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        submap->reserve(
            anchor.cloud->size() *
            static_cast<size_t>(std::max(1, iris_submap_keyframes_))
        );

        // 位姿来源（直接 odom）：Iris 子地图使用 raw_pose，也就是原始
        // T_odom_body 做帧间相对变换；这里不查询 TF，不应用 map->odom。
        const Eigen::Isometry3d T_world_anchor_body =
            poseToIsometry(anchor.raw_pose);
        const Eigen::Isometry3d T_world_anchor_lidar =
            T_world_anchor_body * T_body_lidar_;
        if (!T_world_anchor_lidar.matrix().allFinite())
            return anchor.cloud;

        *submap += *anchor.cloud;
        frames_used = 1U;
        input_points = anchor.cloud->size();

        const int history_limit = std::max(0, iris_submap_keyframes_ - 1);
        int history_used = 0;
        for (
            auto it = keyframes_.rbegin();
            it != keyframes_.rend() && history_used < history_limit;
            ++ it
        ) {

            if (!it->cloud || it->cloud->empty())
                continue;

            const Eigen::Isometry3d T_world_frame_body =
                poseToIsometry(it->raw_pose);
            const Eigen::Isometry3d T_world_frame_lidar =
                T_world_frame_body * T_body_lidar_;
            if (!T_world_frame_lidar.matrix().allFinite())
                continue;

            const Eigen::Isometry3d T_anchor_lidar_frame_lidar =
                T_world_anchor_lidar.inverse() * T_world_frame_lidar;
            if (!T_anchor_lidar_frame_lidar.matrix().allFinite())
                continue;

            pcl::PointCloud<pcl::PointXYZ> transformed;
            pcl::transformPointCloud(
                *it->cloud,
                transformed,
                T_anchor_lidar_frame_lidar.matrix().cast<float>()
            );
            *submap += transformed;
            ++ history_used;
            ++ frames_used;
            input_points += it->cloud->size();
            span_m = std::max(
                span_m,
                T_anchor_lidar_frame_lidar.translation().norm()
            );
        }

        submap->width = static_cast<uint32_t>(submap->size());
        submap->height = 1;
        submap->is_dense = false;
        if (iris_submap_voxel_leaf_m_ <= 0.0 || submap->empty())
            return submap;

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setLeafSize(
            static_cast<float>(iris_submap_voxel_leaf_m_),
            static_cast<float>(iris_submap_voxel_leaf_m_),
            static_cast<float>(iris_submap_voxel_leaf_m_)
        );
        voxel_filter.setInputCloud(submap);
        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        voxel_filter.filter(*filtered);
        filtered->width = static_cast<uint32_t>(filtered->size());
        filtered->height = 1;
        filtered->is_dense = false;
        return filtered;
    }

    Eigen::Matrix3d LoopDetectorNode::rotationFromRpyDeg(
        const std::vector<double> &rpy_deg
    ) {

        if (rpy_deg.size() != 3)
            return Eigen::Matrix3d::Identity();
        const double roll = rpy_deg[0] * M_PI / 180.0;
        const double pitch = rpy_deg[1] * M_PI / 180.0;
        const double yaw = rpy_deg[2] * M_PI / 180.0;
        return (
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        ).toRotationMatrix();
    }

    Eigen::Vector3d LoopDetectorNode::rpyDegFromRotation(
        const Eigen::Matrix3d &R
    ) {

        const double sy =
            std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
        const bool singular = sy < 1e-6;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        if (!singular) {

            roll = std::atan2(R(2, 1), R(2, 2));
            pitch = std::atan2(-R(2, 0), sy);
            yaw = std::atan2(R(1, 0), R(0, 0));
        } else {

            roll = std::atan2(-R(1, 2), R(1, 1));
            pitch = std::atan2(-R(2, 0), sy);
            yaw = 0.0;
        }
        return Eigen::Vector3d(
            roll * 180.0 / M_PI,
            pitch * 180.0 / M_PI,
            yaw * 180.0 / M_PI
        );
    }

    bool LoopDetectorNode::convertKeyFrameMsg(
        const small_point_lio_pgo::msg::KeyFrame &msg,
        LoopKeyFrame &keyframe
    ) const {

        auto cloud_body = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(msg.cloud, *cloud_body);

        if (cloud_body->empty()) return false;

        auto cloud_lidar = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        const std::string &frame_id = msg.cloud.header.frame_id;
        if (frame_id == lidar_frame_) {

            cloud_lidar = cloud_body;
        } else {

            if (!frame_id.empty() && frame_id != body_frame_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Keyframe cloud frame '%s' is neither '%s' nor '%s'; treating it as body frame",
                    frame_id.c_str(),
                    body_frame_.c_str(),
                    lidar_frame_.c_str()
                );
            }
            pcl::transformPointCloud(
                *cloud_body,
                *cloud_lidar,
                T_lidar_body_.matrix().cast<float>()
            );
        }

        if (cloud_lidar->empty()) return false;

        keyframe.id = msg.id;
        keyframe.stamp = rclcpp::Time(msg.header.stamp);
        // 位姿来源（直接 odom）：raw_pose 永远保留消息里的 T_odom_body。
        // pose 最多左乘固定重力对齐变换，仍不是通过动态 map->odom TF 查询
        // 得到的 T_map_body；真正的 map 位姿只在 PGO 优化结果中产生。
        keyframe.raw_pose = msg.pose;
        keyframe.pose = gravity_align_enable_ &&
            (gravity_align_apply_to_pgo_ || gravity_align_apply_to_map_)
                ? isometryToPose(alignedPose(msg.pose))
                : msg.pose;
        keyframe.cloud = cloud_lidar;
        keyframe.descriptor_cloud = makeDescriptorCloud(keyframe);
        return true;
    }

    std::vector<LoopDetectorNode::RetrievalHit> LoopDetectorNode::retrieveByCartKey(
        const LoopKeyFrame &current,
        int &eligible_count
    ) const {

        std::vector<LoopDetectorNode::RetrievalHit> hits;
        for (const auto &history : keyframes_) {

            if (!isCandidateAllowed(current, history))
                continue;

            ++ eligible_count;

            if (!current.has_cart_descriptor ||
                !history.has_cart_descriptor)
                continue;

            const Eigen::VectorXf &current_key =
                current.cart_descriptor.retrieval_key;
            const Eigen::VectorXf &history_key =
                history.cart_descriptor.retrieval_key;
            const float dist = std::min(
                retrievalKeyDistance(current_key, history_key),
                retrievalKeyReverseDistance(current_key, history_key)
            );

            if (!std::isfinite(dist))
                continue;
            if (lcd_retrieval_max_distance_ > 0.0 &&
                dist > lcd_retrieval_max_distance_)
                continue;

            hits.push_back({&history, dist});
        }
        std::sort(
            hits.begin(), hits.end(),
            [](
                const LoopDetectorNode::RetrievalHit &a,
                const LoopDetectorNode::RetrievalHit &b
            ) {
                return a.distance < b.distance;
            }
        );
        if (static_cast<int>(hits.size()) > lcd_retrieval_top_k_)
            hits.resize(static_cast<size_t>(lcd_retrieval_top_k_));
        return hits;
    }

    float LoopDetectorNode::retrievalKeyDistance(
        const Eigen::VectorXf &a,
        const Eigen::VectorXf &b
    ) const {

        if (a.size() == 0 || a.size() != b.size())
            return std::numeric_limits<float>::infinity();
        return (a - b).norm();
    }

    float LoopDetectorNode::retrievalKeyReverseDistance(
        const Eigen::VectorXf &a,
        const Eigen::VectorXf &b
    ) const {

        if (a.size() == 0 || a.size() != b.size())
            return std::numeric_limits<float>::infinity();

        float sum = 0.0F;
        const int size = static_cast<int>(a.size());
        for (int i = 0; i < size; ++i) {

            const float diff = a(size - 1 - i) - b(i);
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }

    bool LoopDetectorNode::computeIrisDescriptor(
        LoopKeyFrame &keyframe
    ) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
            keyframe.descriptor_cloud
            ? keyframe.descriptor_cloud
            : keyframe.cloud;
        size_t submap_frames = 1U;
        size_t submap_input_points =
            keyframe.cloud
            ? keyframe.cloud->size()
            : 0U;
        double submap_span_m = 0.0;
        if (iris_submap_enable_) {

            const auto submap = buildIrisSubmapCloud(
                keyframe,
                submap_frames,
                submap_input_points,
                submap_span_m
            );
            if (submap && !submap->empty())
                cloud = makeDescriptorCloud(keyframe, submap);
        }
        if (!lidar_iris_ || !cloud || cloud->empty())
            return false;

        if (iris_submap_enable_) {

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Iris submap: current=%u frames=%zu/%d input_points=%zu "
                "filtered_points=%zu span_m=%.3f voxel_leaf=%.3f",
                keyframe.id,
                submap_frames,
                iris_submap_keyframes_,
                submap_input_points,
                cloud->size(),
                submap_span_m,
                iris_submap_voxel_leaf_m_
            );
        }

        keyframe.iris_image = LidarIris::GetIris(*cloud);
        if (keyframe.iris_image.empty())
            return false;

        keyframe.iris_descriptor = lidar_iris_->GetFeature(keyframe.iris_image);
        return !keyframe.iris_descriptor.img.empty() &&
            !keyframe.iris_descriptor.T.empty() &&
            !keyframe.iris_descriptor.M.empty();
    }

    bool LoopDetectorNode::computeCartDescriptor(
        LoopKeyFrame &keyframe
    ) const {

        keyframe.has_cart_descriptor = false;
        if (!cart_enable_)
            return true;
        const auto &cloud = keyframe.descriptor_cloud
            ? keyframe.descriptor_cloud
            : keyframe.cloud;
        if (!cart_context_ || !cloud || cloud->empty())
            return false;

        keyframe.cart_descriptor =
            cart_context_->makeDescriptor(*cloud);
        const bool valid =
            keyframe.cart_descriptor.image.rows() == cart_context_->numX() &&
            keyframe.cart_descriptor.image.cols() == cart_context_->numY() &&
            keyframe.cart_descriptor.image.size() > 0 &&
            keyframe.cart_descriptor.image.cwiseAbs().maxCoeff() > 1e-6F;
        keyframe.has_cart_descriptor = valid;
        return valid;
    }

    bool LoopDetectorNode::detectLoopCandidate(
        const LoopKeyFrame &current,
        LoopCandidate &candidate
    ) const {

        float best_distance = std::numeric_limits<float>::infinity();
        float best_cart_distance = std::numeric_limits<float>::infinity();
        int eligible_count = 0;
        return detectLoopCandidate(
            current,
            candidate,
            best_distance,
            best_cart_distance,
            eligible_count
        );
    }

    bool LoopDetectorNode::detectLoopCandidate(
        const LoopKeyFrame &current,
        LoopCandidate &candidate,
        float &best_distance,
        float &best_cart_distance,
        int &eligible_count
    ) const {

        std::vector<LoopCandidate> candidates;
        if (!detectLoopCandidates(
                current,
                candidates,
                best_distance,
                best_cart_distance,
                eligible_count) ||
            candidates.empty())
            return false;

        candidate = candidates.front();
        return true;
    }

    bool LoopDetectorNode::detectLoopCandidates(
        const LoopKeyFrame &current,
        std::vector<LoopCandidate> &candidates,
        float &best_distance,
        float &best_cart_distance,
        int &eligible_count
    ) const {

        candidates.clear();
        best_distance = std::numeric_limits<float>::infinity();
        best_cart_distance = std::numeric_limits<float>::infinity();
        eligible_count = 0;

        struct IrisHit {

            const LoopKeyFrame *history = nullptr;
            float iris_distance = std::numeric_limits<float>::infinity();
            int yaw_bias = 0;
        };

        std::vector<IrisHit> iris_hits;
        iris_hits.reserve(static_cast<size_t>(cart_candidate_top_k_));

        // 第零阶段：遍历历史关键帧，用 retrieval key 做粗召回 search_pool。
        // min keyframe gap 和 travel distance 在 isCandidateAllowed() 中过滤。
        // retrieval 分支只把 top-k 历史帧送入昂贵的 LiDAR-Iris 精排。
        std::vector<const LoopKeyFrame *> search_pool;
        if (lcd_retrieval_enable_ && cart_enable_) {

            const auto retrieval_hits =
                retrieveByCartKey(current, eligible_count);
            search_pool.reserve(retrieval_hits.size());
            for (const auto &hit : retrieval_hits) {

                if (hit.history)
                    search_pool.push_back(hit.history);
            }
        } else {

            search_pool.reserve(keyframes_.size());
            for (const auto &history : keyframes_) {

                if (!isCandidateAllowed(current, history))
                    continue;
                ++eligible_count;
                search_pool.push_back(&history);
            }
        }
        // 第一阶段：遍历粗召回的 search_pool 进行 iris 比对
        for (const auto &history : search_pool) {

            if (!history)
                continue;

            int yaw_bias = 0;
            float dis = lidar_iris_->Compare(
                current.iris_descriptor,
                history->iris_descriptor,
                &yaw_bias
            );

            if (!std::isfinite(dis))
                continue;

            if (dis < best_distance)
                best_distance = dis;

            if (dis >= loop_iris_distance_thresh_)
                continue;

            iris_hits.push_back(IrisHit{history, dis, yaw_bias});
        }

        std::sort(
            iris_hits.begin(),
            iris_hits.end(),
            [](const IrisHit &a, const IrisHit &b) {
                return a.iris_distance < b.iris_distance;
            }
        );
        if (static_cast<int>(iris_hits.size()) > cart_candidate_top_k_) {

            iris_hits.resize(static_cast<size_t>(cart_candidate_top_k_));
        }

        std::ostringstream cart_debug_stream;
        int cart_debug_rank = 0;
        for (const IrisHit &hit : iris_hits) {

            // 第二阶段：可选 Cart Context 复核。正常描述子和 180 度翻转
            // 描述子各比一次，避免相反朝向经过同一区域时被 Cart 误筛掉。
            if (!hit.history)
                continue;

            float cart_distance = std::numeric_limits<float>::infinity();
            int cart_shift_cols = 0;
            double cart_lateral_offset_m = 0.0;

            if (cart_enable_) {

                if (!cart_context_ ||
                    !current.has_cart_descriptor ||
                    !hit.history->has_cart_descriptor) {

                    cart_debug_stream
                        << "#" << cart_debug_rank++
                        << " id=" << hit.history->id
                        << " iris=" << hit.iris_distance
                        << " cart=nan valid=0 pass=0 ";
                    continue;
                }

                const CartContext::MatchResult cart_result_normal =
                    cart_context_->compare(
                        current.cart_descriptor,
                        hit.history->cart_descriptor
                    );
                const CartContext::Descriptor current_cart_flipped =
                    cart_context_->flipped180(current.cart_descriptor);
                const CartContext::MatchResult cart_result_reverse =
                    cart_context_->compare(
                        current_cart_flipped,
                        hit.history->cart_descriptor
                    );

                const bool cart_normal_valid =
                    cart_result_normal.valid &&
                    std::isfinite(cart_result_normal.distance);
                const bool cart_reverse_valid =
                    cart_result_reverse.valid &&
                    std::isfinite(cart_result_reverse.distance);

                CartContext::MatchResult cart_result = cart_result_normal;
                bool cart_reverse_used = false;
                if (cart_reverse_valid &&
                    (!cart_normal_valid ||
                     cart_result_reverse.distance < cart_result_normal.distance)) {

                    cart_result = cart_result_reverse;
                    cart_reverse_used = true;
                }

                if (!cart_result.valid ||
                    !std::isfinite(cart_result.distance)) {

                    cart_debug_stream
                        << "#" << cart_debug_rank++
                        << " id=" << hit.history->id
                        << " iris=" << hit.iris_distance
                        << " cart=nan valid=0 pass=0 rev=0 ";
                    continue;
                }

                cart_distance = cart_result.distance;
                if (cart_distance < best_cart_distance)
                    best_cart_distance = cart_distance;
                cart_shift_cols = cart_result.shift_cols;
                cart_lateral_offset_m = cart_result.lateral_offset_m;
                const bool cart_pass = cart_distance <= cart_distance_thresh_;
                cart_debug_stream
                    << "#" << cart_debug_rank++
                    << " id=" << hit.history->id
                    << " iris=" << hit.iris_distance
                    << " cart=" << cart_distance
                    << " shift=" << cart_shift_cols
                    << " lat=" << cart_lateral_offset_m
                    << " rev=" << (cart_reverse_used ? 1 : 0)
                    << " valid=1 pass=" << (cart_pass ? 1 : 0) << " ";
                if (!cart_pass)
                    continue;
            }

            LoopCandidate candidate;
            candidate.current_id = current.id;
            candidate.history_id = hit.history->id;
            candidate.iris_distance = hit.iris_distance;
            candidate.cart_distance = cart_distance;
            candidate.cart_shift_cols = cart_shift_cols;
            candidate.cart_lateral_offset_m = cart_lateral_offset_m;
            candidate.yaw_bias = hit.yaw_bias;
            candidates.push_back(candidate);
        }

        if (cart_enable_ && !iris_hits.empty()) {

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Loop descriptor top-k: current=%u iris_hits=%zu cart_candidates=%zu "
                "cart_thresh=%.4f [%s]",
                current.id,
                iris_hits.size(),
                candidates.size(),
                cart_distance_thresh_,
                cart_debug_stream.str().c_str()
            );
        }

        if (candidates.empty())
            return false;

        // 第三阶段：按 Cart/iris 分数排序，只把 top-k 送去 GICP，
        // 控制每帧后端计算量。
        std::sort(
            candidates.begin(),
            candidates.end(),
            [this](const LoopCandidate &a, const LoopCandidate &b) {
                if (cart_enable_) {

                    const bool a_cart = std::isfinite(a.cart_distance);
                    const bool b_cart = std::isfinite(b.cart_distance);
                    if (a_cart != b_cart)
                        return a_cart;
                    if (a_cart &&
                        std::abs(a.cart_distance - b.cart_distance) > 1e-4F)
                        return a.cart_distance < b.cart_distance;
                }
                if (std::abs(a.iris_distance - b.iris_distance) > 1e-4F)
                    return a.iris_distance < b.iris_distance;
                return a.history_id < b.history_id;
            }
        );
        if (static_cast<int>(candidates.size()) > loop_descriptor_verify_top_k_)
            candidates.resize(static_cast<size_t>(loop_descriptor_verify_top_k_));

        return true;
    }

    bool LoopDetectorNode::appendKeyFrameToSubmap(
        const LoopKeyFrame &anchor,
        const LoopKeyFrame &frame,
        pcl::PointCloud<pcl::PointXYZ> &submap
    ) const {

        if (!frame.cloud || frame.cloud->empty())
            return false;

        // 位姿来源（后端工作坐标系）：anchor.pose/frame.pose 来自 raw odom，
        // 可包含固定重力对齐，但没有应用动态 map->odom，也没有查询 TF。
        const Eigen::Isometry3d T_world_anchor_body =
            poseToIsometry(anchor.pose);
        const Eigen::Isometry3d T_world_frame_body =
            poseToIsometry(frame.pose);
        if (!T_world_anchor_body.matrix().allFinite() ||
            !T_world_frame_body.matrix().allFinite())
            return false;

        const Eigen::Isometry3d T_world_anchor_lidar =
            T_world_anchor_body * T_body_lidar_;
        const Eigen::Isometry3d T_world_frame_lidar =
            T_world_frame_body * T_body_lidar_;
        if (!T_world_anchor_lidar.matrix().allFinite() ||
            !T_world_frame_lidar.matrix().allFinite())
            return false;

        const Eigen::Isometry3d T_anchor_lidar_frame_lidar =
            T_world_anchor_lidar.inverse() * T_world_frame_lidar;
        if (!T_anchor_lidar_frame_lidar.matrix().allFinite())
            return false;

        pcl::PointCloud<pcl::PointXYZ> transformed;
        pcl::transformPointCloud(
            *frame.cloud,
            transformed,
            T_anchor_lidar_frame_lidar.matrix().cast<float>()
        );
        submap += transformed;
        return true;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr LoopDetectorNode::buildLoopGicpSubmap(
        const LoopKeyFrame &anchor
    ) const {

        // 以 anchor 的 lidar 坐标系为局部坐标，累积 anchor 及其前序关键帧。
        // 这样 loop 验证可以使用局部子图，而不是单帧稀疏点云。
        auto submap = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        const int max_frames = std::max(1, loop_gicp_submap_keyframes_);

        int used = 0;
        if (appendKeyFrameToSubmap(anchor, anchor, *submap))
            ++used;

        int anchor_index = -1;
        for (size_t i = 0; i < keyframes_.size(); ++i) {

            if (keyframes_[i].id == anchor.id) {

                anchor_index = static_cast<int>(i);
                break;
            }
        }

        int index = anchor_index >= 0
            ? anchor_index - 1
            : static_cast<int>(keyframes_.size()) - 1;

        while (used < max_frames && index >= 0) {

            if (appendKeyFrameToSubmap(anchor, keyframes_[static_cast<size_t>(index)], *submap))
                ++used;
            --index;
        }

        if (submap->empty())
            return submap;

        submap->width = static_cast<uint32_t>(submap->size());
        submap->height = 1;
        submap->is_dense = false;

        if (loop_gicp_submap_leaf_size_ > 0.0) {

            // 子图过密时先降采样，降低 GICP 计算量并稳定配准耗时。
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setLeafSize(
                static_cast<float>(loop_gicp_submap_leaf_size_),
                static_cast<float>(loop_gicp_submap_leaf_size_),
                static_cast<float>(loop_gicp_submap_leaf_size_)
            );
            auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            voxel_filter.setInputCloud(submap);
            voxel_filter.filter(*filtered);
            filtered->width = static_cast<uint32_t>(filtered->size());
            filtered->height = 1;
            filtered->is_dense = false;
            return filtered;
        }

        return submap;
    }

    /**
     * @brief 使用 GICP 匹配潜在回环帧筛选真正回环帧
     * @param current 目前的点云帧
     * @param candidate 潜在回环点云帧
     * @return bool 是否成功
     */
    bool LoopDetectorNode::verifyLoopCandidateByGicp(
        const LoopKeyFrame &current,
        LoopCandidate &candidate
    ) {

        // GICP 验证的目标是把描述子候选变成可用于 PGO 的 SE3 测量：
        // current 点云作为 source，history 点云作为 target，输出 history<-current。
        const auto *history = findKeyFrame(candidate.history_id);
        if (!history || !current.cloud || current.cloud->empty() ||
            !history->cloud || history->cloud->empty())
            return false;

        // 位姿来源（后端工作坐标系）：GICP 初值由关键帧 pose 直接计算，
        // 不是从 TF 读取的 T_map_body。两帧使用同一坐标系，取相对量时
        // 固定的坐标系对齐会相消。
        const Eigen::Matrix4d T_world_current = poseToMatrix(current.pose);
        const Eigen::Matrix4d T_world_history = poseToMatrix(history->pose);
        if (!T_world_current.allFinite() || !T_world_history.allFinite())
            return false;

        auto yaw_from_matrix = [](const Eigen::Matrix4d &T) {

            return std::atan2(T(1, 0), T(0, 0));
        };

        auto normalize_deg = [](double deg) {

            while (deg > 180.0) deg -= 360.0;
            while (deg <= -180.0) deg += 360.0;
            return deg;
        };

        auto make_yaw_delta_init = [](
            const Eigen::Matrix4d &base,
            const double yaw_delta_rad
        ) {

            Eigen::Matrix4d init = base;
            const Eigen::AngleAxisd yaw_delta(
                yaw_delta_rad,
                Eigen::Vector3d::UnitZ()
            );
            init.block<3, 3>(0, 0) =
                yaw_delta.toRotationMatrix() * base.block<3, 3>(0, 0);
            return init;
        };

        auto make_lateral_delta_init = [](
            const Eigen::Matrix4d &base,
            const double lateral_m
        ) {

            Eigen::Matrix4d init = base;
            init(1, 3) += lateral_m;
            return init;
        };

        auto correction_delta = [](
            const Eigen::Matrix4d &init,
            const Eigen::Matrix4d &final
        ) {

            const Eigen::Matrix4d correction = init.inverse() * final;
            const double corr_t = correction.block<3, 1>(0, 3).norm();
            Eigen::Matrix3d corr_R = correction.block<3, 3>(0, 0);
            Eigen::Quaterniond corr_q(corr_R);
            corr_q.normalize();
            double corr_r_deg =
                Eigen::AngleAxisd(corr_q).angle() * 180.0 / M_PI;
            if (corr_r_deg > 180.0)
                corr_r_deg = 360.0 - corr_r_deg;
            return std::make_pair(corr_t, corr_r_deg);
        };

        // body 下 history -> current 的相对变化
        const Eigen::Isometry3d body_history_from_current =
            matrixToIsometry(T_world_history.inverse() * T_world_current);
        if (!body_history_from_current.matrix().allFinite())
            return false;

        // 三明治乘法转化为 lidar 下 history -> current
        const Eigen::Isometry3d lidar_history_from_current =
            T_lidar_body_ * body_history_from_current * T_body_lidar_;
        if (!lidar_history_from_current.matrix().allFinite())
            return false;

        // 可选使用关键帧子图替代单帧点云，提高闭环配准的约束密度。
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = current.cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = history->cloud;
        if (loop_gicp_use_submap_) {

            source_cloud = buildLoopGicpSubmap(current);
            target_cloud = buildLoopGicpSubmap(*history);
            if (!source_cloud || source_cloud->empty() ||
                !target_cloud || target_cloud->empty()) {

                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Loop GICP submap is empty: current=%u history=%u source=%zu target=%zu",
                    candidate.current_id,
                    candidate.history_id,
                    source_cloud ? source_cloud->size() : 0U,
                    target_cloud ? target_cloud->size() : 0U
                );
                return false;
            }
        }

        // 这里的 gicp 主要是对齐两个直接从 lidar 上获取的点云（一个是 history 一个是 current）
        const Eigen::Matrix4d init_guess = lidar_history_from_current.matrix();
        const double odom_yaw = yaw_from_matrix(init_guess);
        const double iris_yaw =
            static_cast<double>(candidate.yaw_bias) * M_PI / 180.0;
        const Eigen::Matrix4d init_iris_pos =
            make_yaw_delta_init(init_guess, iris_yaw);
        const Eigen::Matrix4d init_iris_neg =
            make_yaw_delta_init(init_guess, -iris_yaw);

        // 每个 trial 是一次不同初值下的 GICP。最终按 score 和
        // 相对里程计初值的修正幅度共同选最可信的 trial。
        struct GicpTrial {

            std::string label;
            GicpResult result;
            double correction_trans = std::numeric_limits<double>::infinity();
            double correction_rot_deg = std::numeric_limits<double>::infinity();
            double odom_delta_trans = std::numeric_limits<double>::infinity();
            double odom_delta_rot_deg = std::numeric_limits<double>::infinity();
            double selection_cost = std::numeric_limits<double>::infinity();
            bool accepted = false;
        };

        auto evaluate_trial = [&](GicpTrial trial) {

            // bounded score 只统计最大对应距离内的点，但小范围偶然重合仍可能
            // 得到低分，因此还必须要求真实收敛、足够覆盖率和合理修正幅度。
            trial.accepted =
                trial.result.success &&
                trial.result.converged &&
                std::isfinite(trial.result.score) &&
                trial.result.score < loop_gicp_score_thresh_ &&
                trial.result.num_inliers >=
                    static_cast<size_t>(loop_gicp_min_inliers_) &&
                std::isfinite(trial.result.inlier_ratio) &&
                trial.result.inlier_ratio >= loop_gicp_min_inlier_ratio_ &&
                std::isfinite(trial.result.registration_error) &&
                trial.result.hessian.allFinite() &&
                std::isfinite(trial.correction_trans) &&
                trial.correction_trans <= loop_gicp_max_correction_trans_ &&
                std::isfinite(trial.correction_rot_deg) &&
                trial.correction_rot_deg <= loop_gicp_max_correction_rot_deg_;
            if (trial.accepted) {

                trial.selection_cost =
                    trial.result.score +
                    loop_gicp_selection_correction_trans_weight_ *
                        trial.odom_delta_trans +
                    loop_gicp_selection_correction_rot_weight_ *
                        trial.odom_delta_rot_deg;
            }
            return trial;
        };

        struct InitGuess {

            std::string label;
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        };

        std::vector<InitGuess> init_guesses;
        init_guesses.push_back({"odom", init_guess});
        init_guesses.push_back({"iris+", init_iris_pos});
        init_guesses.push_back({"iris-", init_iris_neg});

        // Cart Context 如果估计了横向偏移，就组合进 GICP 初值搜索，
        // 增加窄通道/平行结构场景下配准收敛的机会。
        const bool has_cart_lateral =
            cart_enable_ &&
            std::isfinite(candidate.cart_lateral_offset_m) &&
            std::abs(candidate.cart_lateral_offset_m) > 1e-6;
        if (has_cart_lateral) {

            const double cart_lat = candidate.cart_lateral_offset_m;
            init_guesses.push_back({
                "cart+",
                make_lateral_delta_init(init_guess, cart_lat)
            });
            init_guesses.push_back({
                "cart-",
                make_lateral_delta_init(init_guess, -cart_lat)
            });
            init_guesses.push_back({
                "iris+cart+",
                make_lateral_delta_init(init_iris_pos, cart_lat)
            });
            init_guesses.push_back({
                "iris+cart-",
                make_lateral_delta_init(init_iris_pos, -cart_lat)
            });
            init_guesses.push_back({
                "iris-cart+",
                make_lateral_delta_init(init_iris_neg, cart_lat)
            });
            init_guesses.push_back({
                "iris-cart-",
                make_lateral_delta_init(init_iris_neg, -cart_lat)
            });
        }

        std::vector<GicpTrial> trials;
        trials.reserve(init_guesses.size());
        for (const auto &init : init_guesses) {

            // align() 返回的 transform 是最终 source->target 测量；
            // correction_delta 用来判断本次配准是否偏离初值过大。
            const GicpResult result =
                gicp_matcher_.align(source_cloud, target_cloud, init.transform);
            const auto corr = result.success
                ? correction_delta(init.transform, result.transform)
                : std::make_pair(
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity());
            const auto odom_delta = result.success
                ? correction_delta(init_guess, result.transform)
                : std::make_pair(
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity());
            trials.push_back(
                evaluate_trial(GicpTrial{
                    init.label,
                    result,
                    corr.first,
                    corr.second,
                    odom_delta.first,
                    odom_delta.second,
                    std::numeric_limits<double>::infinity(),
                    false
                })
            );
        }

        std::ostringstream bounded_score_stream;
        std::ostringstream raw_score_stream;
        std::ostringstream cost_stream;
        std::ostringstream corr_t_stream;
        std::ostringstream corr_r_stream;
        std::ostringstream odom_delta_t_stream;
        std::ostringstream odom_delta_r_stream;
        std::ostringstream accepted_stream;
        std::ostringstream quality_stream;
        for (const auto &trial : trials) {

            bounded_score_stream
                << trial.label << " " << trial.result.score << " ";
            raw_score_stream
                << trial.label << " " << trial.result.raw_score << " ";
            cost_stream
                << trial.label << " " << trial.selection_cost << " ";
            corr_t_stream
                << trial.label << " " << trial.correction_trans << " ";
            corr_r_stream
                << trial.label << " " << trial.correction_rot_deg << " ";
            odom_delta_t_stream
                << trial.label << " " << trial.odom_delta_trans << " ";
            odom_delta_r_stream
                << trial.label << " " << trial.odom_delta_rot_deg << " ";
            accepted_stream
                << trial.label << " " << (trial.accepted ? 1 : 0) << " ";
            quality_stream
                << trial.label
                << "(conv=" << (trial.result.converged ? 1 : 0)
                << ",iter=" << trial.result.iterations
                << ",inliers=" << trial.result.num_inliers
                << ",ratio=" << trial.result.inlier_ratio
                << ",error=" << trial.result.registration_error
                << ") ";
        }

        RCLCPP_INFO(
            get_logger(),
            "Loop init debug: current=%u history=%u iris=%.4f cart=%.4f \n"
            "cart_shift=%d cart_lat=%.3f yaw_bias=%d \n"
            "submap=%d points=[%zu %zu] \n"
            "odom_yaw=%.2f iris_yaw+=%.2f iris_yaw-=%.2f \n"
            "bounded_score=[%s] \n raw_score=[%s] \n cost=[%s] \n"
            "corr_t=[%s] \n corr_r=[%s] \n"
            "odom_delta_t=[%s] \n odom_delta_r=[%s] \n accepted=[%s] \n"
            "quality=[%s]",
            candidate.current_id,
            candidate.history_id,
            candidate.iris_distance,
            candidate.cart_distance,
            candidate.cart_shift_cols,
            candidate.cart_lateral_offset_m,
            candidate.yaw_bias,
            loop_gicp_use_submap_ ? 1 : 0,
            source_cloud ? source_cloud->size() : 0U,
            target_cloud ? target_cloud->size() : 0U,
            normalize_deg(odom_yaw * 180.0 / M_PI),
            normalize_deg(iris_yaw * 180.0 / M_PI),
            normalize_deg(-iris_yaw * 180.0 / M_PI),
            bounded_score_stream.str().c_str(),
            raw_score_stream.str().c_str(),
            cost_stream.str().c_str(),
            corr_t_stream.str().c_str(),
            corr_r_stream.str().c_str(),
            odom_delta_t_stream.str().c_str(),
            odom_delta_r_stream.str().c_str(),
            accepted_stream.str().c_str(),
            quality_stream.str().c_str()
        );

        const GicpTrial *best_trial = nullptr;
        for (const auto &trial : trials) {

            // 只在通过 bounded score、overlap 和 correction gate 的 trial 中选最小代价。
            if (!trial.accepted)
                continue;
            if (!best_trial ||
                trial.selection_cost < best_trial->selection_cost)
                best_trial = &trial;
        }

        if (!best_trial) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Loop GICP rejected: current=%u history=%u iris=%.4f cart=%.4f "
                "accepted=[%s] bounded_score=[%s] raw_score=[%s] "
                "cost=[%s] quality=[%s]",
                candidate.current_id,
                candidate.history_id,
                candidate.iris_distance,
                candidate.cart_distance,
                accepted_stream.str().c_str(),
                bounded_score_stream.str().c_str(),
                raw_score_stream.str().c_str(),
                cost_stream.str().c_str(),
                quality_stream.str().c_str()
            );
            return false;
        }

        const double corr_t = best_trial->correction_trans;
        const double corr_r_deg = best_trial->correction_rot_deg;

        candidate.gicp_score = best_trial->result.score;
        candidate.gicp_selection_cost = best_trial->selection_cost;
        candidate.gicp_correction_trans = corr_t;
        candidate.gicp_correction_rot_deg = corr_r_deg;
        candidate.gicp_inlier_ratio = best_trial->result.inlier_ratio;
        candidate.gicp_registration_error =
            best_trial->result.registration_error;
        candidate.gicp_iterations = best_trial->result.iterations;
        candidate.gicp_num_inliers = best_trial->result.num_inliers;
        candidate.source_to_target = best_trial->result.transform;
        candidate.gicp_verified = true;

        RCLCPP_INFO(
            get_logger(),
            "GICP candidate accepted: current=%u history=%u init=%s iris=%.4f "
            "cart=%.4f cart_shift=%d cart_lat=%.3f "
            "submap=%d points=[%zu %zu] bounded_score=%.4f raw_score=%.4f "
            "cost=%.4f corr_t=%.3f corr_r=%.2f "
            "converged=%d iter=%zu inliers=%zu ratio=%.3f error=%.3f",
            candidate.current_id,
            candidate.history_id,
            best_trial->label.c_str(),
            candidate.iris_distance,
            candidate.cart_distance,
            candidate.cart_shift_cols,
            candidate.cart_lateral_offset_m,
            loop_gicp_use_submap_ ? 1 : 0,
            source_cloud ? source_cloud->size() : 0U,
            target_cloud ? target_cloud->size() : 0U,
            candidate.gicp_score,
            best_trial->result.raw_score,
            best_trial->selection_cost,
            corr_t,
            corr_r_deg,
            best_trial->result.converged ? 1 : 0,
            best_trial->result.iterations,
            best_trial->result.num_inliers,
            best_trial->result.inlier_ratio,
            best_trial->result.registration_error
        );

        return true;
    }

    bool LoopDetectorNode::isCandidateAllowed(
        const LoopKeyFrame &current,
        const LoopKeyFrame &history
    ) const {

        const int64_t id_gap =
            std::llabs(
                static_cast<int64_t>(current.id) -
                static_cast<int64_t>(history.id)
            );
        if (id_gap < static_cast<int64_t>(loop_min_keyframe_gap_))
            return false;

        const double travel_gap =
            current.travel_distance - history.travel_distance;
        if (!std::isfinite(travel_gap) ||
            travel_gap < loop_min_travel_distance_)
            return false;

        return !current.iris_descriptor.T.empty() &&
            !current.iris_descriptor.M.empty() &&
            !history.iris_descriptor.T.empty() &&
            !history.iris_descriptor.M.empty();
    }

    const LoopDetectorNode::LoopKeyFrame *LoopDetectorNode::findKeyFrame(
        const uint32_t id
    ) const {

        for (const auto &keyframe : keyframes_)
            if (keyframe.id == id)
                return &keyframe;
        return nullptr;
    }

    Eigen::Matrix4d LoopDetectorNode::poseToMatrix(
        const geometry_msgs::msg::Pose &pose
    ) const {

        Eigen::Quaterniond q(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );
        if (!q.coeffs().allFinite() || q.norm() < 1e-6)
            return Eigen::Matrix4d::Constant(
                std::numeric_limits<double>::quiet_NaN());
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T(0, 3) = pose.position.x;
        T(1, 3) = pose.position.y;
        T(2, 3) = pose.position.z;
        return T;
    }

    Eigen::Isometry3d LoopDetectorNode::poseToIsometry(
        const geometry_msgs::msg::Pose &pose
    ) const {

        return matrixToIsometry(poseToMatrix(pose));
    }

    /**
     * @brief 等距变换矩阵
     * @param 要变换的矩阵
     * @return Eigen::Isometry3d 刚体变换矩阵
     *
     * 具体来说，就是[R t]，方式矩阵不合法
     *             [0 1]
     */
    Eigen::Isometry3d LoopDetectorNode::matrixToIsometry(
        const Eigen::Matrix4d &matrix
    ) const {

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        if (!matrix.allFinite())
            return Eigen::Isometry3d(
                Eigen::Matrix4d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));

        Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
        if (!q.coeffs().allFinite() || q.norm() < 1e-6)
            return Eigen::Isometry3d(
                Eigen::Matrix4d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
        q.normalize();

        T.linear() = q.toRotationMatrix();
        T.translation() = matrix.block<3, 1>(0, 3);
        return T;
    }

    geometry_msgs::msg::Pose LoopDetectorNode::isometryToPose(
        const Eigen::Isometry3d &pose
    ) const {

        geometry_msgs::msg::Pose msg;
        msg.position.x = pose.translation().x();
        msg.position.y = pose.translation().y();
        msg.position.z = pose.translation().z();

        Eigen::Quaterniond q(pose.rotation());
        q.normalize();
        msg.orientation.w = q.w();
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        return msg;
    }

    Eigen::Matrix<double, 6, 6> LoopDetectorNode::makeInformationMatrix(
        const std::vector<double> &diag,
        const double fallback_weight,
        const char *param_name
    ) const {

        // 优先使用 6 维对角信息矩阵；旧的标量权重只作为配置缺失
        // 或非法时的兼容 fallback。
        Eigen::Matrix<double, 6, 6> information =
            Eigen::Matrix<double, 6, 6>::Zero();

        if (diag.size() == 6) {

            bool valid = true;
            for (const double value : diag) {

                if (!std::isfinite(value) || value <= 0.0) {

                    valid = false;
                    break;
                }
            }

            if (valid) {

                for (int i = 0; i < 6; ++i)
                    information(i, i) = diag[static_cast<size_t>(i)];
                return information;
            }

            RCLCPP_WARN(
                get_logger(),
                "%s must contain 6 positive finite values; falling back to scalar weight",
                param_name ? param_name : "pgo_info_diag"
            );
        } else if (!diag.empty()) {

            RCLCPP_WARN(
                get_logger(),
                "%s must contain 6 values [x,y,z,roll,pitch,yaw], got %zu; "
                "falling back to scalar weight",
                param_name ? param_name : "pgo_info_diag",
                diag.size()
            );
        }

        const double safe_weight =
            std::isfinite(fallback_weight) && fallback_weight > 0.0
                ? fallback_weight
                : 1.0;
        information.setIdentity();
        information *= safe_weight;
        return information;
    }

    std::pair<double, double> LoopDetectorNode::relativeDelta(
        const Eigen::Isometry3d &a,
        const Eigen::Isometry3d &b
    ) const {

        const Eigen::Isometry3d delta = a.inverse() * b;
        const double trans = delta.translation().norm();
        Eigen::Quaterniond q(delta.rotation());
        q.normalize();
        double rot_deg = Eigen::AngleAxisd(q).angle() * 180.0 / M_PI;
        if (rot_deg > 180.0)
            rot_deg = 360.0 - rot_deg;
        return {trans, rot_deg};
    }

    Eigen::Isometry3d LoopDetectorNode::makeLoopMeasurement(
        const Eigen::Isometry3d &T_world_history,
        const Eigen::Isometry3d &T_world_current,
        const Eigen::Isometry3d &full_measurement,
        Eigen::Vector3d &world_translation_correction,
        Eigen::Vector3d &world_rotation_correction_rpy_deg
    ) const {

        // 变换链：world<-history 乘 history<-current，得到 GICP 推断的
        // world<-current；再与前端的 world<-current 比较，可直接看出
        // GICP 想给地图施加多少平移和 roll/pitch/yaw 修正。
        const Eigen::Isometry3d T_world_current_from_gicp =
            T_world_history * full_measurement;
        world_translation_correction =
            T_world_current_from_gicp.translation() -
            T_world_current.translation();
        const Eigen::Matrix3d R_world_correction =
            T_world_current_from_gicp.rotation() *
            T_world_current.rotation().transpose();
        world_rotation_correction_rpy_deg =
            rpyDegFromRotation(R_world_correction);

        if (pgo_loop_measurement_mode_ == "full_se3")
            return full_measurement;

        // 坡道上的局部点云容易让 GICP 在 z/roll/pitch 上退化。默认模式只取
        // GICP 的世界系 x/y 和绕重力轴 yaw，沿用 LIO/IMU 给出的高度与坡度。
        Eigen::Isometry3d T_world_current_projected = T_world_current;
        T_world_current_projected.translation().x() =
            T_world_current_from_gicp.translation().x();
        T_world_current_projected.translation().y() =
            T_world_current_from_gicp.translation().y();
        T_world_current_projected.translation().z() =
            T_world_current.translation().z();

        const double yaw_correction =
            std::atan2(R_world_correction(1, 0), R_world_correction(0, 0));
        Eigen::Quaterniond projected_rotation(
            Eigen::AngleAxisd(yaw_correction, Eigen::Vector3d::UnitZ()) *
            T_world_current.rotation()
        );
        projected_rotation.normalize();
        T_world_current_projected.linear() =
            projected_rotation.toRotationMatrix();

        return T_world_history.inverse() * T_world_current_projected;
    }

    bool LoopDetectorNode::addKeyFrameToPoseGraph(
        const LoopKeyFrame &keyframe
    ) {

        // 位姿来源（后端工作坐标系）：keyframe.pose 直接来自 odom，可能已
        // 做固定重力对齐，但没有经动态 map->odom TF 修正。PGO 优化成功后，
        // PoseGraphNode::pose 才被解释为优化后的 T_map_body。
        // PGO 图和关键帧库使用同一套 backend id。新节点加入后，
        // 如果存在前一关键帧，就添加连续里程计边。
        const Eigen::Isometry3d current_pose = poseToIsometry(keyframe.pose);
        if (!current_pose.matrix().allFinite())
            return false;

        PoseGraphNode node;
        node.id = keyframe.id;
        node.stamp = keyframe.stamp.seconds();
        node.pose = current_pose;

        const bool added = pose_graph_.addNode(node);
        if (!added)
            return false;

        if (keyframes_.empty())
            return true;

        const LoopKeyFrame &previous = keyframes_.back();
        const Eigen::Isometry3d previous_pose = poseToIsometry(previous.pose);
        if (!previous_pose.matrix().allFinite())
            return false;

        const Eigen::Isometry3d relative_pose =
            previous_pose.inverse() * current_pose;
        return pose_graph_.addOdomEdge(
            previous.id,
            keyframe.id,
            relative_pose,
            makeInformationMatrix(
                pgo_odom_info_diag_,
                pgo_odom_edge_weight_,
                "pgo_odom_info_diag"
            )
        );
    }

    bool LoopDetectorNode::shouldAddLoopEdgeToPoseGraph(
        const LoopCandidate &candidate,
        int &current_gap,
        double &travel_gap
    ) const {

        current_gap = std::numeric_limits<int>::max();
        travel_gap = std::numeric_limits<double>::infinity();

        if (!has_last_added_loop_edge_)
            return true;

        const int64_t raw_current_gap =
            static_cast<int64_t>(candidate.current_id) -
            static_cast<int64_t>(last_added_loop_current_id_);
        if (raw_current_gap > static_cast<int64_t>(std::numeric_limits<int>::max())) {

            current_gap = std::numeric_limits<int>::max();
        } else if (raw_current_gap < static_cast<int64_t>(std::numeric_limits<int>::min())) {

            current_gap = std::numeric_limits<int>::min();
        } else {

            current_gap = static_cast<int>(raw_current_gap);
        }

        if (loop_edge_min_current_gap_ > 0 &&
            raw_current_gap >= 0 &&
            raw_current_gap < loop_edge_min_current_gap_)
            return false;

        const auto *current = findKeyFrame(candidate.current_id);
        if (current) {

            travel_gap =
                current->travel_distance -
                last_added_loop_current_travel_distance_;
        }

        if (loop_edge_min_travel_gap_ > 0.0 &&
            std::isfinite(travel_gap) &&
            travel_gap < loop_edge_min_travel_gap_)
            return false;

        return true;
    }

    bool LoopDetectorNode::addLoopCandidateToPoseGraph(
        const LoopCandidate &candidate
    ) {

        if (!candidate.gicp_verified ||
            !candidate.source_to_target.allFinite())
            return false;

        // GICP 在 lidar 坐标下验证回环；PGO 节点姿态在 body 坐标下，
        // 所以先用外参三明治乘法把 history_lidar<-current_lidar 转回 body。
        const Eigen::Isometry3d history_lidar_from_current_lidar =
            matrixToIsometry(candidate.source_to_target);
        if (!history_lidar_from_current_lidar.matrix().allFinite())
            return false;

        const Eigen::Isometry3d history_body_from_current_body =
            T_body_lidar_ *
            history_lidar_from_current_lidar *
            T_lidar_body_;
        if (!history_body_from_current_body.matrix().allFinite())
            return false;

        const auto *history = findKeyFrame(candidate.history_id);
        const auto *current = findKeyFrame(candidate.current_id);
        if (!history || !current)
            return false;

        // 位姿来源（后端工作坐标系）：这里直接读取两关键帧保存的 pose 来
        // 构造 odom 预测相对量，不查询 TF，也不应用当前 map->odom。
        const Eigen::Isometry3d T_world_history =
            poseToIsometry(history->pose);
        const Eigen::Isometry3d T_world_current =
            poseToIsometry(current->pose);
        if (!T_world_history.matrix().allFinite() ||
            !T_world_current.matrix().allFinite())
            return false;

        const Eigen::Isometry3d odom_history_from_current =
            T_world_history.inverse() * T_world_current;
        const auto direct_delta =
            relativeDelta(odom_history_from_current, history_body_from_current_body);
        const auto inverse_delta =
            relativeDelta(
                odom_history_from_current,
                history_body_from_current_body.inverse()
            );

        Eigen::Vector3d world_translation_correction =
            Eigen::Vector3d::Zero();
        Eigen::Vector3d world_rotation_correction_rpy_deg =
            Eigen::Vector3d::Zero();
        // “找对历史地点”只说明 LCD 正确，不代表 GICP 的完整 6DoF 都可信。
        // makeLoopMeasurement() 按配置选择完整测量或重力保持测量后再交给 g2o。
        const Eigen::Isometry3d loop_measurement = makeLoopMeasurement(
            T_world_history,
            T_world_current,
            history_body_from_current_body,
            world_translation_correction,
            world_rotation_correction_rpy_deg
        );
        if (!loop_measurement.matrix().allFinite())
            return false;
        const auto used_delta =
            relativeDelta(odom_history_from_current, loop_measurement);

        // loop 信息矩阵先取基础 diag，再按 GICP score 动态缩放：
        // score 越小认为配准越可信，loop 约束越强。
        Eigen::Matrix<double, 6, 6> loop_information =
            makeInformationMatrix(
                pgo_loop_info_diag_,
                pgo_loop_edge_weight_,
                "pgo_loop_info_diag"
            );
        double loop_info_scale = 1.0;
        if (pgo_loop_info_dynamic_enable_) {

            const double score_for_scale =
                std::isfinite(candidate.gicp_score)
                    ? std::max(candidate.gicp_score, pgo_loop_info_score_floor_)
                    : pgo_loop_info_score_floor_;
            loop_info_scale = std::clamp(
                pgo_loop_info_score_ref_ / score_for_scale,
                pgo_loop_info_min_scale_,
                pgo_loop_info_max_scale_
            );
            loop_information *= loop_info_scale;
        }

        RCLCPP_INFO(
            get_logger(),
            "Loop edge debug: history=%u current=%u gicp_score=%.4f iris=%.4f "
            "loop_info_scale=%.3f robust_kernel_enable=%d robust_kernel_delta=%.3f "
            "loop_info_diag_after_scale=[%.3f %.3f %.3f %.3f %.3f %.3f] "
            "odom_vs_direct_dp=%.3f dr=%.2f odom_vs_inverse_dp=%.3f dr=%.2f "
            "measurement_mode=%s odom_vs_used_dp=%.3f dr=%.2f "
            "gicp_world_dt=[%.3f %.3f %.3f] gicp_world_drpy_deg=[%.2f %.2f %.2f] "
            "gicp_inliers=%zu ratio=%.3f error=%.3f "
            "direct_t=[%.3f %.3f %.3f] used_t=[%.3f %.3f %.3f]",
            candidate.history_id,
            candidate.current_id,
            candidate.gicp_score,
            candidate.iris_distance,
            loop_info_scale,
            pgo_loop_robust_kernel_enable_ ? 1 : 0,
            pgo_loop_robust_kernel_delta_,
            loop_information(0, 0),
            loop_information(1, 1),
            loop_information(2, 2),
            loop_information(3, 3),
            loop_information(4, 4),
            loop_information(5, 5),
            direct_delta.first,
            direct_delta.second,
            inverse_delta.first,
            inverse_delta.second,
            pgo_loop_measurement_mode_.c_str(),
            used_delta.first,
            used_delta.second,
            world_translation_correction.x(),
            world_translation_correction.y(),
            world_translation_correction.z(),
            world_rotation_correction_rpy_deg.x(),
            world_rotation_correction_rpy_deg.y(),
            world_rotation_correction_rpy_deg.z(),
            candidate.gicp_num_inliers,
            candidate.gicp_inlier_ratio,
            candidate.gicp_registration_error,
            history_body_from_current_body.translation().x(),
            history_body_from_current_body.translation().y(),
            history_body_from_current_body.translation().z(),
            loop_measurement.translation().x(),
            loop_measurement.translation().y(),
            loop_measurement.translation().z()
        );

        return pose_graph_.addLoopEdge(
            candidate.history_id,
            candidate.current_id,
            loop_measurement,
            loop_information,
            candidate.gicp_score,
            pgo_loop_robust_kernel_enable_,
            pgo_loop_robust_kernel_delta_
        );
    }

    double LoopDetectorNode::optimizePoseGraphIfNeeded(
        const bool has_new_loop
    ) {

        // 默认只在新增 loop edge 后优化。这样普通 odom 边持续入图，
        // 但不会每个关键帧都触发一次全图优化。
        if (!pgo_enable_)
            return 0.0;
        if (pgo_optimize_on_loop_ && !has_new_loop)
            return 0.0;

        const auto pgo_start = SteadyClock::now();
        const PoseGraphOptimizationSummary summary = pose_graph_.optimize();
        const double pgo_ms = elapsedMs(pgo_start, SteadyClock::now());
        if (!summary.success) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "PGO skipped/failed: %s nodes=%d edges=%d pgo_ms=%.3f",
                summary.message.c_str(),
                summary.node_count,
                summary.edge_count,
                pgo_ms
            );
            return pgo_ms;
        }

        RCLCPP_INFO(
            get_logger(),
            "PGO optimized: iter=%d nodes=%d edges=%d chi2 %.3f -> %.3f pgo_ms=%.3f",
            summary.iterations,
            summary.node_count,
            summary.edge_count,
            summary.initial_chi2,
            summary.final_chi2,
            pgo_ms
        );
        has_pgo_optimized_snapshot_ = true;
        updateMapToOdomFromOptimizedGraph();
        publishOptimizedKeyFrames();
        publishMapToOdom();
        return pgo_ms;
    }

    void LoopDetectorNode::updateMapToOdomFromOptimizedGraph() {

        if (!map_to_odom_tf_enabled_ || !has_pgo_optimized_snapshot_ ||
            pose_graph_.empty())
            return;

        // Use the newest common keyframe as the live anchor. The optimized
        // pose is in map and raw_pose is in the frontend odom frame.
        const PoseGraphNode *latest_node = nullptr;
        for (const auto &node : pose_graph_.nodes()) {

            if (!latest_node || node.stamp >= latest_node->stamp)
                latest_node = &node;
        }
        if (!latest_node)
            return;

        const auto *keyframe = findKeyFrame(latest_node->id);
        if (!keyframe)
            return;

        // 位姿来源（混合求解）：latest_node->pose 是 PGO 输出的 T_map_body；
        // raw_pose 是消息中直接保存的 T_odom_body。这里没有查询 TF，而是
        // 用二者主动求出将要发布的 T_map_odom。
        const Eigen::Isometry3d map_from_body = latest_node->pose;
        const Eigen::Isometry3d odom_from_body =
            poseToIsometry(keyframe->raw_pose);
        if (!map_from_body.matrix().allFinite() ||
            !odom_from_body.matrix().allFinite())
            return;

        const Eigen::Isometry3d map_from_odom =
            map_from_body * odom_from_body.inverse();
        if (!map_from_odom.matrix().allFinite())
            return;

        {
            std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
            map_from_odom_ = map_from_odom;
            map_to_odom_anchor_id_ = latest_node->id;
            map_to_odom_has_anchor_ = true;
        }

        Eigen::Quaterniond rotation(map_from_odom.rotation());
        rotation.normalize();
        double rotation_deg =
            Eigen::AngleAxisd(rotation).angle() * 180.0 / M_PI;
        if (rotation_deg > 180.0)
            rotation_deg = 360.0 - rotation_deg;

        RCLCPP_INFO(
            get_logger(),
            "Updated map->odom correction: anchor=%u t=[%.3f %.3f %.3f] "
            "rot=%.3fdeg",
            latest_node->id,
            map_from_odom.translation().x(),
            map_from_odom.translation().y(),
            map_from_odom.translation().z(),
            rotation_deg
        );
    }

    void LoopDetectorNode::publishMapToOdom() const {

        if (!map_to_odom_tf_enabled_ || !tf_broadcaster_)
            return;

        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = odom_frame_;

        Eigen::Isometry3d cached_map_from_odom;
        uint32_t anchor_id = 0;
        bool has_anchor = false;
        {
            std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
            cached_map_from_odom = map_from_odom_;
            anchor_id = map_to_odom_anchor_id_;
            has_anchor = map_to_odom_has_anchor_;
        }
        const bool correction_active =
            apply_map_to_odom_correction_ && has_anchor;
        const Eigen::Isometry3d map_from_odom = correction_active
            ? cached_map_from_odom
            : Eigen::Isometry3d::Identity();
        if (!map_from_odom.matrix().allFinite())
            return;

        tf.transform.translation.x = map_from_odom.translation().x();
        tf.transform.translation.y = map_from_odom.translation().y();
        tf.transform.translation.z = map_from_odom.translation().z();
        Eigen::Quaterniond rotation(map_from_odom.rotation());
        rotation.normalize();
        tf.transform.rotation.w = rotation.w();
        tf.transform.rotation.x = rotation.x();
        tf.transform.rotation.y = rotation.y();
        tf.transform.rotation.z = rotation.z();

        // This is a current, continuously valid frame relationship. Using
        // the old keyframe stamp would make consumers query beyond the TF
        // buffer after the robot has moved on.
        tf.header.stamp = now();

        tf_broadcaster_->sendTransform(tf);

        double rotation_deg = 0.0;
        rotation_deg =
            Eigen::AngleAxisd(map_from_odom.rotation()).angle() *
            180.0 / M_PI;
        if (rotation_deg > 180.0)
            rotation_deg = 360.0 - rotation_deg;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Published map->odom: correction_enabled=%d active=%d anchor=%u "
            "t=[%.3f %.3f %.3f] "
            "rot=%.3fdeg map=%s odom=%s",
            apply_map_to_odom_correction_ ? 1 : 0,
            correction_active ? 1 : 0,
            anchor_id,
            map_from_odom.translation().x(),
            map_from_odom.translation().y(),
            map_from_odom.translation().z(),
            rotation_deg,
            map_frame_.c_str(),
            odom_frame_.c_str()
        );
    }

    void LoopDetectorNode::publishOptimizedKeyFrames() const {

        // 发送完整优化关键帧列表给 map_node；map_node 用它重建 /global_map，
        // 并对尚未优化的尾部关键帧可选应用最近一次 PGO correction。
        if (!pub_optimized_keyframes_ || pose_graph_.empty())
            return;

        small_point_lio_pgo::msg::OptimizedKeyFrames msg;
        msg.header.frame_id = map_frame_;
        msg.ids.reserve(pose_graph_.nodeCount());
        msg.poses.reserve(pose_graph_.nodeCount());

        double latest_stamp = 0.0;
        for (const auto &node : pose_graph_.nodes()) {
            if (!node.pose.matrix().allFinite())
                continue;

            // 位姿来源（PGO/map）：node.pose 是 g2o 优化后直接保存的
            // T_map_body，不是通过 TF 再转换一次的 odom pose。
            msg.ids.push_back(node.id);
            msg.poses.push_back(isometryToPose(node.pose));
            if (std::isfinite(node.stamp) && node.stamp > latest_stamp)
                latest_stamp = node.stamp;
        }

        if (latest_stamp > 0.0) {

            const auto stamp_ns =
                static_cast<int64_t>(latest_stamp * 1000000000.0);
            msg.header.stamp.sec =
                static_cast<int32_t>(stamp_ns / 1000000000LL);
            msg.header.stamp.nanosec =
                static_cast<uint32_t>(stamp_ns % 1000000000LL);
        } else {

            msg.header.stamp = now();
        }

        if (msg.ids.empty())
            return;

        pub_optimized_keyframes_->publish(msg);
    }

    void LoopDetectorNode::storeKeyFrame(
        LoopKeyFrame &&keyframe
    ) {

        accumulated_travel_distance_ = keyframe.travel_distance;
        // shouldAcceptKeyFrameCandidate() receives msg.pose before gravity
        // alignment, so retain the matching raw pose for the next gate check.
        last_accepted_keyframe_raw_pose_ = keyframe.raw_pose;
        last_accepted_keyframe_stamp_ = keyframe.stamp;
        has_last_accepted_keyframe_ = true;
        keyframes_.push_back(std::move(keyframe));
    }

    void LoopDetectorNode::publishLoopMarkers() const {

        visualization_msgs::msg::MarkerArray markers;
        const rclcpp::Time stamp = now();

        auto make_base_marker = [&](const int id, const int type, const std::string &ns) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = map_frame_;
            marker.ns = ns;
            marker.id = id;
            marker.type = type;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            return marker;
        };

        auto to_point = [](const geometry_msgs::msg::Pose &pose) {
            geometry_msgs::msg::Point point;
            point.x = pose.position.x;
            point.y = pose.position.y;
            point.z = pose.position.z;
            return point;
        };

        auto pose_for_id = [this](
            const uint32_t id,
            geometry_msgs::msg::Pose &pose
        ) {

            if (has_pgo_optimized_snapshot_) {

                Eigen::Isometry3d optimized_pose;
                if (pose_graph_.getPose(id, optimized_pose) &&
                    optimized_pose.matrix().allFinite()) {

                    // 位姿来源（PGO/map）：直接读取图中的 T_map_body，
                    // 不通过 TF 将 odom pose 再转换一次。
                    pose = isometryToPose(optimized_pose);
                    return true;
                }
            }

            const auto *keyframe = findKeyFrame(id);
            if (!keyframe)
                return false;

            // 位姿来源（回退路径）：首次 PGO 前读取 raw T_odom_body，此时
            // map->odom 是单位变换；PGO 后正常应由上面的 graph map pose 命中。
            // 这里同样没有执行 TF 查询。
            pose = has_pgo_optimized_snapshot_
                ? keyframe->pose
                : keyframe->raw_pose;
            return poseToIsometry(pose).matrix().allFinite();
        };

        auto nodes = make_base_marker(
            0,
            visualization_msgs::msg::Marker::SPHERE_LIST,
            "loop_detector_keyframe_nodes"
        );
        nodes.scale.x = 0.15;
        nodes.scale.y = 0.15;
        nodes.scale.z = 0.15;
        nodes.color.r = 0.10F;
        nodes.color.g = 0.85F;
        nodes.color.b = 1.00F;
        nodes.color.a = 1.00F;
        nodes.points.reserve(keyframes_.size());
        for (const auto &keyframe : keyframes_) {

            geometry_msgs::msg::Pose pose;
            if (pose_for_id(keyframe.id, pose))
                nodes.points.push_back(to_point(pose));
        }

        auto odom_edges = make_base_marker(
            1,
            visualization_msgs::msg::Marker::LINE_LIST,
            "loop_detector_odom_edges"
        );
        odom_edges.scale.x = 0.030;
        odom_edges.color.r = 0.25F;
        odom_edges.color.g = 0.45F;
        odom_edges.color.b = 1.00F;
        odom_edges.color.a = 0.80F;
        if (keyframes_.size() >= 2)
            odom_edges.points.reserve((keyframes_.size() - 1U) * 2U);
        for (size_t i = 1; i < keyframes_.size(); ++ i) {

            geometry_msgs::msg::Pose from_pose;
            geometry_msgs::msg::Pose to_pose;
            if (!pose_for_id(keyframes_[i - 1].id, from_pose) ||
                !pose_for_id(keyframes_[i].id, to_pose))
                continue;

            odom_edges.points.push_back(to_point(from_pose));
            odom_edges.points.push_back(to_point(to_pose));
        }

        auto loop_edges = make_base_marker(
            2,
            visualization_msgs::msg::Marker::LINE_LIST,
            "loop_detector_candidate_edges"
        );
        loop_edges.scale.x = 0.08;
        loop_edges.color.r = 1.00F;
        loop_edges.color.g = 0.80F;
        loop_edges.color.b = 0.00F;
        loop_edges.color.a = 1.00F;
        loop_edges.points.reserve(loop_candidates_.size() * 2U);
        for (const auto &candidate : loop_candidates_) {

            geometry_msgs::msg::Pose current_pose;
            geometry_msgs::msg::Pose history_pose;
            if (!pose_for_id(candidate.current_id, current_pose) ||
                !pose_for_id(candidate.history_id, history_pose))
                continue;

            loop_edges.points.push_back(to_point(history_pose));
            loop_edges.points.push_back(to_point(current_pose));
        }

        markers.markers.push_back(std::move(nodes));
        markers.markers.push_back(std::move(odom_edges));
        markers.markers.push_back(std::move(loop_edges));
        pub_loop_markers_->publish(markers);
    }

    void LoopDetectorNode::publishOptimizedPath() const {

        if (!pub_optimized_path_ || pose_graph_.empty())
            return;

        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = map_frame_;
        path.poses.reserve(pose_graph_.nodeCount());

        for (const auto &node : pose_graph_.nodes()) {

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            if (has_pgo_optimized_snapshot_) {

                if (!node.pose.matrix().allFinite())
                    continue;
                // 位姿来源（PGO/map）：直接读取 g2o 图中的 T_map_body。
                pose.pose = isometryToPose(node.pose);
            } else {

                const auto *keyframe = findKeyFrame(node.id);
                if (!keyframe)
                    continue;
                // 位姿来源（直接 odom）：首次 PGO 前读取 T_odom_body；由于
                // 此时 map->odom 为单位变换，可安全以 map frame 发布显示。
                pose.pose = keyframe->raw_pose;
            }
            path.poses.push_back(std::move(pose));
        }

        pub_optimized_path_->publish(path);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Published optimized_path: poses=%zu nodes=%zu edges=%zu",
            path.poses.size(),
            pose_graph_.nodeCount(),
            pose_graph_.edgeCount()
        );
    }

    void LoopDetectorNode::publishTiming(
        const double lcd_ms,
        const double pgo_ms,
        const rclcpp::Time &stamp
    ) const {

        if (!pub_timing_)
            return;

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = map_frame_;
        msg.vector.x = lcd_ms;
        msg.vector.y = pgo_ms;
        msg.vector.z = lcd_ms + pgo_ms;
        pub_timing_->publish(msg);

        RCLCPP_INFO(
            get_logger(),
            "LCD/PGO timing: lcd_ms=%.3f pgo_ms=%.3f total_ms=%.3f",
            msg.vector.x,
            msg.vector.y,
            msg.vector.z
        );
    }

} // namespace small_point_lio_pgo
