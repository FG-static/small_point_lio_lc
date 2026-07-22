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

        /// 统计一个完整 LiDAR packet 内的 scan-to-map 更新质量。
        struct PacketUpdateStatistics {
            uint32_t attempted = 0;///< 提交给 ESKF 的点残差数。
            uint32_t accepted = 0; ///< 被 ESKF 接受的点残差数。
            double residual_squared_sum = 0.0;///< 用于计算 packet RMS 的残差平方和。
            double residual_max_abs = 0.0;   ///< 已接受残差的最大绝对值。

            void reset();
        };

        /// 将 correction 序号与该 packet 边界的累计修正绑定，供地图重定位和 lag 检查使用。
        struct CorrectionSnapshot {
            uint64_t sequence = 0;///< Published correction sequence.
            Eigen::Isometry3d cumulative_correction =
                    Eigen::Isometry3d::Identity();///< corrected * epoch_predicted^-1.
        };

        /// 记录进入当前 iVox 的点，使异步重建期间的新点可在切图前重放。
        struct JournalPoint {
            uint64_t sequence = 0;///< Monotonic insertion order.
            double timestamp = 0.0;///< Point acquisition time, in seconds.
            Eigen::Vector3f point = Eigen::Vector3f::Zero();///< Estimator-odom point, in metres.
        };

        /// 前端线程交给后台 iVox 构建线程的任务。
        struct MapBuildRequest {
            common::LocalTrackingMapUpdate update;///< Versioned cloud and build metadata.
            uint64_t reset_generation = 0;///< Rejects work queued before a frontend reset.
        };

        /// 后台已经构建完成、等待安全 packet 边界应用的替换地图。
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

        // prediction_only_kf：当前地图 epoch 以来只接收 IMU 的基准。
        // packet_prediction_kf：上一个 packet 边界以来只接收 IMU 的基准。
        // 两者都不接收点更新，用于分别计算累计修正和瞬时修正。
        eskf prediction_only_kf;
        eskf packet_prediction_kf;
        bool prediction_epoch_valid = false;///< 两个 IMU-only 基准是否已经初始化。
        std::deque<double> packet_end_boundaries;///< 等待数据消费完成的 packet 结束时间。
        double last_registered_packet_end = -1.0;///< 最近登记的 packet 结束时间。
        PacketUpdateStatistics packet_statistics;///< 当前 packet 的点更新统计。
        uint64_t correction_sequence = 0;///< 前端 correction 的运行内单调编号。
        std::deque<CorrectionSnapshot> correction_history;///< 按序保存近期累计修正快照。

        std::atomic<uint64_t> tracking_map_version{0};///< 当前正在使用的 iVox 版本。
        uint64_t active_map_source_correction_sequence = 0;///< 当前 iVox 来源 correction 序号。
        uint64_t point_insertion_sequence = 0;///< tail journal 中点的插入顺序编号。
        std::deque<JournalPoint> tail_point_journal;///< 异步建图期间暂存的新点。
        double journal_pruned_through_timestamp =
                -std::numeric_limits<double>::infinity();///< journal 已经裁剪到的最晚时间。

        std::mutex map_builder_mutex;///< 保护地图构建队列和构建结果。
        std::condition_variable_any map_builder_cv;///< 唤醒后台地图构建线程。
        std::optional<MapBuildRequest> queued_map_build;///< 尚未开始的最新地图任务。
        std::optional<BuiltMapUpdate> built_map_update;///< 已完成但尚未应用的最新地图。
        std::jthread map_builder_thread;///< 后台 iVox 构建线程。
        std::atomic<uint64_t> reset_generation{0};///< 前端运行代次，用于使 reset 前任务失效。

    public:
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> Q;

        explicit SmallPointLio(rclcpp::Node &node);

        ~SmallPointLio();

        /// 重置滤波器、地图、版本计数和异步任务代次。
        void reset();

        /// 接收一个 LiDAR packet，并登记其结束时间。
        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        /// 接收一个 IMU 样本并驱动后续时间序列处理。
        void on_imu_callback(const common::ImuMsg &imu_msg);

        /// 消费当前缓存中的 IMU/LiDAR 数据，并尝试完成 packet 边界。
        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback);

        void set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback);

        /// 设置 correction 发布回调，由 ROS 节点转换为消息。
        void set_scan_to_map_correction_callback(
                const std::function<void(const common::ScanToMapCorrection &correction)> &callback);

        /// 接收后端地图并排队异步重建前端 iVox。
        void queue_local_tracking_map(common::LocalTrackingMapUpdate update);

        /// 返回当前前端正在使用的 iVox 版本。
        [[nodiscard]] uint64_t get_tracking_map_version() const;

    private:
        void publish_odometry(double timestamp);

        /// 登记当前回调内所有点的最大时间作为 packet 结束时间。
        void register_packet_boundary(const std::vector<common::Point> &pointcloud);

        /// 只在相关点云和 IMU 已消费完后完成等待中的 packet。
        void finalize_ready_packet_boundaries();

        /// 判断 boundary 之前的稀疏点、稠密点和 IMU 是否都已消费。
        [[nodiscard]] bool packet_boundary_ready(double boundary) const;

        /// 生成一个 packet correction，并刷新 packet prediction 基准。
        void finalize_packet(double boundary);

        /// 用当前 ESKF 状态重置 correction-free epoch 基准。
        void reset_prediction_epoch();

        [[nodiscard]] static Eigen::Isometry3d state_pose(const state &x);

        [[nodiscard]] static common::Pose3d common_pose(
                const Eigen::Isometry3d &pose);

        /// 计算当前 ESKF 相对于 epoch prediction 的累计修正。
        [[nodiscard]] Eigen::Isometry3d cumulative_correction() const;

        /// 将新处理的点追加到 tail journal，并执行容量/时间裁剪。
        void append_tail_point(double timestamp, const Eigen::Vector3f &point);

        /// 裁剪过旧或过量的 tail journal 点。
        void prune_tail_point_journal(double newest_timestamp);

        /// 后台构造替换 iVox，不直接触碰前端当前地图。
        void map_builder_loop(std::stop_token stop_token);

        /// 在 packet 边界校验并安全切换后台构建完成的地图。
        void try_apply_built_map();

        /// 从 correction history 查找指定序号的累计修正。
        [[nodiscard]] bool correction_for_sequence(
                uint64_t sequence,
                Eigen::Isometry3d &correction) const;

        [[nodiscard]] static double rotation_angle(
                const Eigen::Matrix3d &rotation);
    };

}// namespace small_point_lio
