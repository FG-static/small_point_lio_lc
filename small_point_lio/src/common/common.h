/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace common {

    struct Odometry {
        double timestamp;                // Unit: s
        Eigen::Vector3d position;        // Unit: m
        Eigen::Vector3d velocity;        // Unit: m/s
        Eigen::Quaterniond orientation;  // Unit: rad
        Eigen::Vector3d angular_velocity;// Unit: rad/s
    };

    struct ImuMsg {
        double timestamp;                   // Unit: s
        Eigen::Vector3d linear_acceleration;// Unit: g
        Eigen::Vector3d angular_velocity;   // Unit: rad/s
    };

    struct Point {
        double timestamp;        // Unit: s
        Eigen::Vector3f position;// Unit: m
    };

    /// ROS-independent rigid-pose value used by the frontend feedback path.
    /// The owning message defines the coordinate frame; translation is in metres.
    struct Pose3d {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();///< Translation in the owning frame.
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();///< Unit rotation quaternion.
    };

    /// Packet-boundary scan-to-map evidence exported by the Point-LIO frontend.
    /// It keeps both prediction baselines and the corrected ESKF pose so the
    /// backend can distinguish accumulated drift from a single-packet update.
    struct ScanToMapCorrection {
        double timestamp = 0.0;///< LiDAR packet boundary time, in seconds.
        uint64_t sequence = 0; ///< Monotonic frontend correction identifier.
        uint64_t tracking_map_version = 0;///< iVox version used for this update.
        Pose3d packet_predicted_pose;///< IMU-only prediction from the preceding packet boundary.
        Pose3d epoch_predicted_pose; ///< IMU-only prediction from the current map epoch.
        Pose3d corrected_pose;       ///< ESKF pose after this packet's point updates.
        uint32_t attempted_point_updates = 0;///< Point residuals offered to the ESKF.
        uint32_t accepted_point_updates = 0; ///< Point residuals accepted by the ESKF.
        double residual_rms = 0.0;          ///< RMS over accepted point residuals.
        double residual_max_abs = 0.0;      ///< Largest accepted absolute residual.
        uint64_t active_map_source_correction_sequence = 0;///< Evidence sequence that produced the active iVox.
    };

    /// Versioned tracking-map replacement delivered from the local backend to
    /// the frontend's asynchronous iVox builder.
    struct LocalTrackingMapUpdate {
        double cutoff_timestamp = 0.0;///< Rebuilt-map horizon; later journal points must be replayed.
        uint64_t source_correction_sequence = 0;///< Evidence used to build and rebase this map.
        uint64_t source_tracking_map_version = 0;///< Active iVox version on which the build started.
        uint64_t target_tracking_map_version = 0;///< Version assigned after an atomic map swap.
        uint64_t pgo_graph_version = 0;///< Global graph snapshot incorporated by the build.
        uint32_t anchor_candidate_id = 0;///< Fixed oldest node of the local optimization window.
        std::vector<Eigen::Vector3f> points_odom;///< Map points in the estimator's odom basis, in metres.
    };

}// namespace common
