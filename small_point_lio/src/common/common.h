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

    /// 前端在一个 LiDAR packet 边界导出的 scan-to-map 证据。
    /// 同时保存两个 IMU-only 基线和本 packet 结束后的 ESKF 位姿，
    /// 供后端区分单 packet 修正与累计漂移。
    struct ScanToMapCorrection {
        double timestamp = 0.0;///< packet 边界时间，单位秒。
        uint64_t sequence = 0; ///< 本次运行内第几个前端 correction。
        uint64_t tracking_map_version = 0;///< 产生本 correction 时使用的 iVox 版本。
        Pose3d packet_predicted_pose;///< 从上一个 packet 边界仅用 IMU 推进的位姿。
        Pose3d epoch_predicted_pose; ///< 当前地图版本启用以来仅用 IMU 推进的位姿。
        Pose3d corrected_pose;       ///< 本 packet 点更新完成后的 ESKF 位姿。
        uint32_t attempted_point_updates = 0;///< 提交给 ESKF 的点面残差数。
        uint32_t accepted_point_updates = 0; ///< 被 ESKF 接受的点面残差数。
        double residual_rms = 0.0;          ///< 被接受残差的 RMS。
        double residual_max_abs = 0.0;      ///< 被接受残差的最大绝对值。
        uint64_t active_map_source_correction_sequence = 0;///< 当前 iVox 来源 correction 序号。
    };

    /// 后端发给前端异步 iVox 构建器的带版本地图替换请求。
    struct LocalTrackingMapUpdate {
        double cutoff_timestamp = 0.0;///< 地图点云覆盖到的时间；之后的 tail 点需要重放。
        uint64_t source_correction_sequence = 0;///< 构建并重定位该地图所依据的 correction 序号。
        uint64_t source_tracking_map_version = 0;///< 构建开始时前端正在使用的 iVox 版本。
        uint64_t target_tracking_map_version = 0;///< 地图切换成功后应成为的版本。
        uint64_t pgo_graph_version = 0;///< 构建时采用的全局 PGO 图快照版本。
        uint32_t anchor_candidate_id = 0;///< 局部窗口固定锚点的关键帧 ID。
        std::string trigger_reason;///< 触发原因：pgo、瞬时修正或累计修正。
        std::vector<Eigen::Vector3f> points_odom;///< 以估计器 odom 基准表达的地图点。
    };

}// namespace common
