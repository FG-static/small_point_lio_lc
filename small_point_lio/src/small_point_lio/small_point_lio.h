/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "estimator.h"
#include "parameters.h"
#include "preprocess.h"
#include <pch.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>

namespace small_point_lio {

    class SmallPointLioTestPeer;

    class SmallPointLio {
    private:
        friend class SmallPointLioTestPeer;

        /// Accumulates scan-to-map update quality over one complete LiDAR packet.
        struct PacketUpdateStatistics {
            uint32_t attempted = 0;///< Point residuals submitted to the ESKF.
            uint32_t accepted = 0; ///< Point residuals accepted by the ESKF.
            double residual_squared_sum = 0.0;///< Sum used to compute packet RMS.
            double residual_max_abs = 0.0;   ///< Largest accepted absolute residual.

            void reset();
        };

        /// Associates a published correction sequence with the cumulative
        /// frontend correction at that packet boundary for later map rebasing.
        struct CorrectionSnapshot {
            uint64_t sequence = 0;///< Published correction sequence.
            Eigen::Isometry3d cumulative_correction =
                    Eigen::Isometry3d::Identity();///< corrected * epoch_predicted^-1.
        };

        /// Records a point inserted into the live iVox so points newer than an
        /// asynchronous rebuild's cutoff can be replayed before map activation.
        struct JournalPoint {
            uint64_t sequence = 0;///< Monotonic insertion order.
            double timestamp = 0.0;///< Point acquisition time, in seconds.
            Eigen::Vector3f point = Eigen::Vector3f::Zero();///< Estimator-odom point, in metres.
        };

        /// Work item handed from the frontend thread to the background iVox builder.
        struct MapBuildRequest {
            common::LocalTrackingMapUpdate update;///< Versioned cloud and build metadata.
            uint64_t reset_generation = 0;///< Rejects work queued before a frontend reset.
        };

        /// Completed background build staged until a safe LiDAR packet boundary.
        struct BuiltMapUpdate {
            common::LocalTrackingMapUpdate metadata;///< Metadata copied from the request.
            std::shared_ptr<SmallIVox> ivox;///< Fully populated replacement tracking map.
            uint64_t reset_generation = 0;///< Generation fence paired with the request.
        };

        Parameters parameters;
        Preprocess preprocess;
        Estimator estimator;
        rclcpp::Logger logger;
        double time_current = 0.0;
        std::vector<Eigen::Vector3f> pointcloud_odom_frame;
        std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> pointcloud_callback;
        std::function<void(const common::Odometry &odometry)> odometry_callback;
        std::function<void(const common::ScanToMapCorrection &correction)>
                scan_to_map_correction_callback;
        bool is_init = false;

        // The epoch shadow supplies a correction-free local motion chain for
        // fixed-lag odometry edges. The packet shadow is refreshed at every
        // completed LiDAR packet so its delta excludes only that packet's
        // point updates while retaining all earlier state corrections.
        eskf prediction_only_kf;
        eskf packet_prediction_kf;
        bool prediction_epoch_valid = false;
        std::deque<double> packet_end_boundaries;
        double last_registered_packet_end = -1.0;
        PacketUpdateStatistics packet_statistics;
        uint64_t correction_sequence = 0;
        std::deque<CorrectionSnapshot> correction_history;

        std::atomic<uint64_t> tracking_map_version{0};
        uint64_t active_map_source_correction_sequence = 0;
        uint64_t point_insertion_sequence = 0;
        std::deque<JournalPoint> tail_point_journal;
        double journal_pruned_through_timestamp =
                -std::numeric_limits<double>::infinity();

        std::mutex map_builder_mutex;
        std::condition_variable_any map_builder_cv;
        std::optional<MapBuildRequest> queued_map_build;
        std::optional<BuiltMapUpdate> built_map_update;
        std::jthread map_builder_thread;
        std::atomic<uint64_t> reset_generation{0};

    public:
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> Q;

        explicit SmallPointLio(rclcpp::Node &node);

        ~SmallPointLio();

        void reset();

        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        void on_imu_callback(const common::ImuMsg &imu_msg);

        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback);

        void set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback);

        void set_scan_to_map_correction_callback(
                const std::function<void(const common::ScanToMapCorrection &correction)> &callback);

        void queue_local_tracking_map(common::LocalTrackingMapUpdate update);

        [[nodiscard]] uint64_t get_tracking_map_version() const;

    private:
        void publish_odometry(double timestamp);

        void register_packet_boundary(const std::vector<common::Point> &pointcloud);

        void finalize_ready_packet_boundaries();

        [[nodiscard]] bool packet_boundary_ready(double boundary) const;

        void finalize_packet(double boundary);

        void reset_prediction_epoch();

        [[nodiscard]] static Eigen::Isometry3d state_pose(const state &x);

        [[nodiscard]] static common::Pose3d common_pose(
                const Eigen::Isometry3d &pose);

        [[nodiscard]] Eigen::Isometry3d cumulative_correction() const;

        void append_tail_point(double timestamp, const Eigen::Vector3f &point);

        void prune_tail_point_journal(double newest_timestamp);

        void map_builder_loop(std::stop_token stop_token);

        void try_apply_built_map();

        [[nodiscard]] bool correction_for_sequence(
                uint64_t sequence,
                Eigen::Isometry3d &correction) const;

        [[nodiscard]] static double rotation_angle(
                const Eigen::Matrix3d &rotation);
    };

}// namespace small_point_lio
