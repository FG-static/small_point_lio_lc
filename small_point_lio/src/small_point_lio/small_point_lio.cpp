/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"

namespace small_point_lio {

    namespace {

        constexpr double kTimestampEpsilon = 1e-7;
        constexpr size_t kCorrectionHistoryCapacity = 4096;
        constexpr size_t kIvoxCapacity = 1000000;

    }// namespace

    void SmallPointLio::PacketUpdateStatistics::reset() {
        attempted = 0;
        accepted = 0;
        residual_squared_sum = 0.0;
        residual_max_abs = 0.0;
    }

    SmallPointLio::SmallPointLio(rclcpp::Node &node)
        : logger(node.get_logger()) {
        // init param
        parameters.read_parameters(node);
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T.cast<state::value_type>();
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R.cast<state::value_type>();
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T.cast<state::value_type>();
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R.cast<state::value_type>();
        }
        Q = estimator.process_noise_cov();
        estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;

        // init data
        reset();

        if (parameters.local_map_feedback_enable) {
            map_builder_thread = std::jthread(
                    [this](const std::stop_token stop_token) {
                        map_builder_loop(stop_token);
                    });
        }
    }

    SmallPointLio::~SmallPointLio() {
        if (map_builder_thread.joinable()) {
            map_builder_thread.request_stop();
            map_builder_cv.notify_all();
            // Join before member destruction: a worker already building an
            // iVox can still read reset_generation after stop is requested.
            map_builder_thread.join();
        }
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        is_init = false;
        prediction_epoch_valid = false;
        packet_end_boundaries.clear();
        last_registered_packet_end = -1.0;
        packet_statistics.reset();
        correction_sequence = 0;
        correction_history.clear();
        tracking_map_version.store(0, std::memory_order_release);
        active_map_source_correction_sequence = 0;
        point_insertion_sequence = 0;
        tail_point_journal.clear();
        journal_pruned_through_timestamp =
                -std::numeric_limits<double>::infinity();
        pointcloud_odom_frame.clear();

        reset_generation.fetch_add(1, std::memory_order_acq_rel);
        {
            std::lock_guard<std::mutex> lock(map_builder_mutex);
            queued_map_build.reset();
            built_map_update.reset();
        }
        map_builder_cv.notify_all();
    }

    // LiDAR 包回调
    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        if (is_init && parameters.local_map_feedback_enable) {
            register_packet_boundary(pointcloud);
        }
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        // we need to init small point lio
        if (!is_init) {
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // init map
                for (const auto &point: preprocess.point_deque) {
                    estimator.ivox->add_point(point.position);
                }
                // fix gravity direction
                if (parameters.fix_gravity_direction) {
                    estimator.kf.x.gravity = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (const auto &imu_msg: preprocess.imu_deque) {
                        estimator.kf.x.gravity += imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    state::value_type scale = -static_cast<state::value_type>(parameters.gravity.norm()) / estimator.kf.x.gravity.norm();
                    estimator.kf.x.gravity *= scale;
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                }
                estimator.kf.x.acceleration = -estimator.kf.x.gravity;
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                estimator.kf.init_timestamp(time_current);
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                packet_end_boundaries.clear();
                last_registered_packet_end = -1.0;
                is_init = true;
                if (parameters.local_map_feedback_enable) {
                    reset_prediction_epoch();
                }
            }
            return;
        }

        // judge we should do point update or imu update
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // collect odom frame pointcloud
                Eigen::Matrix<state::value_type, 3, 1> dense_point_imu_frame;
                if (parameters.extrinsic_est_en) {
                    dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                } else {
                    dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                }
                pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());

                preprocess.dense_point_deque.pop_front();
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // point update
                if (point_lidar_frame.timestamp < time_current) {
                    preprocess.point_deque.pop_front();
                    finalize_ready_packet_boundaries();
                    continue;
                }
                time_current = point_lidar_frame.timestamp;

                // predict
                estimator.kf.predict_state(time_current);
                if (prediction_epoch_valid) {
                    prediction_only_kf.predict_state(time_current);
                    packet_prediction_kf.predict_state(time_current);
                }

                // update
                estimator.point_lidar_frame = point_lidar_frame.position;
                state::value_type residual = 0.0;
                ++packet_statistics.attempted;
                if (estimator.kf.update_point(&residual)) {
                    ++packet_statistics.accepted;
                    const double residual_abs = std::abs(static_cast<double>(residual));
                    packet_statistics.residual_squared_sum += residual_abs * residual_abs;
                    packet_statistics.residual_max_abs = std::max(
                            packet_statistics.residual_max_abs, residual_abs);
                }

                // publish odometry
                if (parameters.publish_odometry_without_downsample) {
                    publish_odometry(time_current);
                }

                // map incremental
                estimator.ivox->add_point(estimator.point_odom_frame);
                append_tail_point(time_current, estimator.point_odom_frame);

                preprocess.point_deque.pop_front();
            } else {
                // imu update
                if (imu_msg.timestamp < time_current) {
                    preprocess.imu_deque.pop_front();
                    finalize_ready_packet_boundaries();
                    continue;
                }
                time_current = imu_msg.timestamp;

                // predict
                estimator.kf.predict_state(time_current);
                estimator.kf.predict_cov(time_current, Q);
                if (prediction_epoch_valid) {
                    prediction_only_kf.predict_state(time_current);
                    prediction_only_kf.predict_cov(time_current, Q);
                    packet_prediction_kf.predict_state(time_current);
                    packet_prediction_kf.predict_cov(time_current, Q);
                }

                // update
                estimator.angular_velocity = imu_msg.angular_velocity.cast<state::value_type>();
                estimator.linear_acceleration = imu_msg.linear_acceleration.cast<state::value_type>();
                estimator.kf.update_imu();
                if (prediction_epoch_valid) {
                    // Both shadows retain the estimator's IMU measurement
                    // callback but own their state/covariance and never receive
                    // point updates.
                    prediction_only_kf.update_imu();
                    packet_prediction_kf.update_imu();
                }

                preprocess.imu_deque.pop_front();
            }

            finalize_ready_packet_boundaries();
        }

        finalize_ready_packet_boundaries();

        if (is_publish_odometry) {
            if (!parameters.publish_odometry_without_downsample) {
                publish_odometry(time_current);
            }
            if (!pointcloud_odom_frame.empty()) {
                if (pointcloud_callback) {
                    pointcloud_callback(pointcloud_odom_frame);
                }
                pointcloud_odom_frame.clear();
            }
        }
    }

    void SmallPointLio::set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback) {
        this->pointcloud_callback = pointcloud_callback;
    }

    void SmallPointLio::set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    void SmallPointLio::set_scan_to_map_correction_callback(
            const std::function<void(const common::ScanToMapCorrection &correction)> &callback) {
        scan_to_map_correction_callback = callback;
    }

    void SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            common::Odometry odometry;
            odometry.timestamp = timestamp;
            odometry.position = estimator.kf.x.position.cast<double>();
            odometry.velocity = estimator.kf.x.velocity.cast<double>();
            odometry.orientation = estimator.kf.x.rotation.cast<double>();
            odometry.angular_velocity = estimator.kf.x.omg.cast<double>();
            odometry_callback(odometry);
        }
    }

    // 每来一个LiDAR包就记录一下包边界时间戳
    void SmallPointLio::register_packet_boundary(
            const std::vector<common::Point> &pointcloud) {
        double packet_end = -std::numeric_limits<double>::infinity();
        for (const auto &point: pointcloud) {
            if (std::isfinite(point.timestamp)) {
                packet_end = std::max(packet_end, point.timestamp); // record the end-cloud_point stamp be the packet_end stamp.
            }
        }
        if (!std::isfinite(packet_end)) {
            return;
        }
        if (last_registered_packet_end >= 0.0 &&
            packet_end <= last_registered_packet_end + kTimestampEpsilon) {
            RCLCPP_WARN(
                    logger,
                    "Ignoring non-increasing LiDAR packet boundary %.9f after %.9f",
                    packet_end, last_registered_packet_end);
            return;
        }
        packet_end_boundaries.push_back(packet_end);
        last_registered_packet_end = packet_end;
    }

    // boundary 记录的是一段时间内的所有 LiDAR 包的结束时间戳
    bool SmallPointLio::packet_boundary_ready(const double boundary) const {
        // 累计时间超过 kTimestampEpsilon 阈值
        // 稀疏、稠密点云 + imu数据均已消费完毕
        const bool points_ready = preprocess.point_deque.empty() ||
                                  preprocess.point_deque.front().timestamp >
                                          boundary + kTimestampEpsilon;
        const bool dense_points_ready = preprocess.dense_point_deque.empty() ||
                                        preprocess.dense_point_deque.front().timestamp >
                                                boundary + kTimestampEpsilon;
        const bool imu_advanced = !preprocess.imu_deque.empty() &&
                                  preprocess.imu_deque.front().timestamp >
                                          boundary + kTimestampEpsilon;
        return points_ready && dense_points_ready && imu_advanced;
    }

    void SmallPointLio::finalize_ready_packet_boundaries() {
        if (!parameters.local_map_feedback_enable || !prediction_epoch_valid) {
            return;
        }

        while (!packet_end_boundaries.empty()) {
            const double boundary = packet_end_boundaries.front();
            if (!packet_boundary_ready(boundary)) {
                break;
            }

            // 三个kf不断积分
            if (time_current < boundary) {
                estimator.kf.predict_state(boundary);
                prediction_only_kf.predict_state(boundary);
                packet_prediction_kf.predict_state(boundary);
                time_current = boundary;
            }

            finalize_packet(boundary);
            packet_end_boundaries.pop_front();
            try_apply_built_map();
        }
    }

    void SmallPointLio::finalize_packet(const double boundary) {
        const Eigen::Isometry3d corrected = state_pose(estimator.kf.x);
        const Eigen::Isometry3d epoch_predicted = state_pose(prediction_only_kf.x);
        const Eigen::Isometry3d packet_predicted =
                state_pose(packet_prediction_kf.x);
        // 有关 scantomap 的矫正量，如果没有进行 scantomap 则为单位阵
        const Eigen::Isometry3d current_cumulative =
                corrected * epoch_predicted.inverse();

        common::ScanToMapCorrection correction;
        correction.timestamp = boundary;
        correction.sequence = ++correction_sequence;
        correction.tracking_map_version =
                tracking_map_version.load(std::memory_order_acquire);
        correction.packet_predicted_pose = common_pose(packet_predicted);
        correction.epoch_predicted_pose = common_pose(epoch_predicted);
        correction.corrected_pose = common_pose(corrected);
        correction.attempted_point_updates = packet_statistics.attempted;
        correction.accepted_point_updates = packet_statistics.accepted;
        if (packet_statistics.accepted > 0) {
            correction.residual_rms = std::sqrt(
                    packet_statistics.residual_squared_sum /
                    static_cast<double>(packet_statistics.accepted));
        }
        correction.residual_max_abs = packet_statistics.residual_max_abs;
        correction.active_map_source_correction_sequence =
                active_map_source_correction_sequence;

        correction_history.push_back(
                CorrectionSnapshot{correction.sequence, current_cumulative});
        while (correction_history.size() > kCorrectionHistoryCapacity) {
            correction_history.pop_front();
        }

        // 发送矫正 msg 给后端（给 local_map_feedback_node ）
        if (scan_to_map_correction_callback) {
            scan_to_map_correction_callback(correction);
        }

        // The next packet starts from the fully corrected state at this exact
        // boundary. Copying the whole filter (not only its SE(3) pose) retains
        // earlier point updates to velocity, biases, gravity, and covariance.
        packet_prediction_kf = estimator.kf;
        packet_statistics.reset();
    }

    void SmallPointLio::reset_prediction_epoch() {
        prediction_only_kf = estimator.kf;
        packet_prediction_kf = estimator.kf;
        prediction_epoch_valid = true;
        packet_statistics.reset();
        correction_history.clear();
    }

    Eigen::Isometry3d SmallPointLio::state_pose(const state &x) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = x.rotation.cast<double>();
        pose.translation() = x.position.cast<double>();
        return pose;
    }

    common::Pose3d SmallPointLio::common_pose(const Eigen::Isometry3d &pose) {
        common::Pose3d result;
        result.position = pose.translation();
        result.orientation = Eigen::Quaterniond(pose.rotation()).normalized();
        return result;
    }

    Eigen::Isometry3d SmallPointLio::cumulative_correction() const {
        if (!prediction_epoch_valid) {
            return Eigen::Isometry3d::Identity();
        }
        return state_pose(estimator.kf.x) *
               state_pose(prediction_only_kf.x).inverse();
    }

    void SmallPointLio::append_tail_point(
            const double timestamp,
            const Eigen::Vector3f &point) {
        if (!parameters.local_map_feedback_enable || !point.allFinite()) {
            return;
        }
        tail_point_journal.push_back(
                JournalPoint{++point_insertion_sequence, timestamp, point});
        prune_tail_point_journal(timestamp);
    }

    void SmallPointLio::prune_tail_point_journal(const double newest_timestamp) {
        const double oldest_allowed =
                newest_timestamp - parameters.local_map_tail_journal_duration_sec;
        while (!tail_point_journal.empty() &&
               (tail_point_journal.size() >
                        parameters.local_map_tail_journal_max_points ||
                tail_point_journal.front().timestamp < oldest_allowed)) {
            journal_pruned_through_timestamp = std::max(
                    journal_pruned_through_timestamp,
                    tail_point_journal.front().timestamp);
            tail_point_journal.pop_front();
        }
    }

    void SmallPointLio::queue_local_tracking_map(
            common::LocalTrackingMapUpdate update) {
        if (!parameters.local_map_feedback_enable) {
            return;
        }
        if (!std::isfinite(update.cutoff_timestamp) ||
            update.points_odom.empty() ||
            update.target_tracking_map_version !=
                    update.source_tracking_map_version + 1) {
            RCLCPP_WARN(logger, "Rejected malformed local tracking-map update");
            return;
        }
        const uint64_t active_version =
                tracking_map_version.load(std::memory_order_acquire);
        if (update.source_tracking_map_version != active_version) {
            RCLCPP_WARN(
                    logger,
                    "Rejected local map for stale source version %lu (active=%lu)",
                    static_cast<unsigned long>(update.source_tracking_map_version),
                    static_cast<unsigned long>(active_version));
            return;
        }

        MapBuildRequest request;
        request.update = std::move(update);
        request.reset_generation = reset_generation.load(std::memory_order_acquire);
        {
            std::lock_guard<std::mutex> lock(map_builder_mutex);
            if (!queued_map_build ||
                request.update.source_correction_sequence >=
                        queued_map_build->update.source_correction_sequence) {
                queued_map_build = std::move(request);
            }
        }
        // 唤醒地图重建线程
        map_builder_cv.notify_all();
    }

    void SmallPointLio::map_builder_loop(const std::stop_token stop_token) {
        while (!stop_token.stop_requested()) {
            MapBuildRequest request;
            {
                std::unique_lock<std::mutex> lock(map_builder_mutex);
                map_builder_cv.wait(
                        lock,
                        stop_token,
                        [this]() { return queued_map_build.has_value(); });
                if (stop_token.stop_requested()) {
                    return;
                }
                request = std::move(*queued_map_build);
                queued_map_build.reset();
            }

            // 重建iVox地图
            auto new_ivox = std::make_shared<SmallIVox>(
                    static_cast<float>(parameters.map_resolution),
                    kIvoxCapacity);
            new_ivox->add_points(request.update.points_odom);
            if (new_ivox->size() < parameters.init_map_size) {
                RCLCPP_WARN(
                        logger,
                        "Discarded rebuilt iVox with only %zu occupied voxels",
                        new_ivox->size());
                continue;
            }
            if (request.reset_generation !=
                reset_generation.load(std::memory_order_acquire)) {
                continue;
            }

            BuiltMapUpdate built;
            built.metadata = std::move(request.update);
            built.ivox = std::move(new_ivox);
            built.reset_generation = request.reset_generation;
            {
                std::lock_guard<std::mutex> lock(map_builder_mutex);
                if (!built_map_update ||
                    built.metadata.source_correction_sequence >=
                            built_map_update->metadata.source_correction_sequence) {
                    built_map_update = std::move(built);
                }
            }
        }
    }

    void SmallPointLio::try_apply_built_map() {
        std::optional<BuiltMapUpdate> candidate;
        {
            std::lock_guard<std::mutex> lock(map_builder_mutex);
            if (!built_map_update) {
                return;
            }
            candidate = std::move(built_map_update);
            built_map_update.reset();
        }

        const uint64_t active_version =
                tracking_map_version.load(std::memory_order_acquire);
        if (candidate->reset_generation !=
                    reset_generation.load(std::memory_order_acquire) ||
            candidate->metadata.source_tracking_map_version != active_version ||
            candidate->metadata.target_tracking_map_version != active_version + 1) {
            RCLCPP_WARN(logger, "Discarded stale rebuilt iVox version");
            return;
        }

        Eigen::Isometry3d source_correction;
        if (!correction_for_sequence(
                    candidate->metadata.source_correction_sequence,
                    source_correction)) {
            RCLCPP_WARN(
                    logger,
                    "Discarded rebuilt iVox: correction sequence %lu is no longer available",
                    static_cast<unsigned long>(
                            candidate->metadata.source_correction_sequence));
            return;
        }

        // Reject every rebuilt map, including PGO-triggered maps, when the
        // frontend correction has moved too far since the build snapshot.
        const Eigen::Isometry3d lag =
                cumulative_correction() * source_correction.inverse();
        const double lag_translation = lag.translation().norm();
        const double lag_rotation_deg =
                rotation_angle(lag.rotation()) * 180.0 / M_PI;
        if (lag_translation >
                    parameters.local_map_max_apply_lag_translation_m ||
            lag_rotation_deg >
                    parameters.local_map_max_apply_lag_rotation_deg) {
            RCLCPP_WARN(
                    logger,
                    "Discarded stale rebuilt iVox: post-snapshot correction "
                    "translation=%.3fm rotation=%.3fdeg",
                    lag_translation,
                    lag_rotation_deg);
            return;
        }

        if (candidate->metadata.cutoff_timestamp + kTimestampEpsilon <
            journal_pruned_through_timestamp) {
            RCLCPP_WARN(
                    logger,
                    "Discarded rebuilt iVox: tail journal no longer covers cutoff %.9f",
                    candidate->metadata.cutoff_timestamp);
            return;
        }

        size_t replayed_points = 0;
        for (const auto &journal_point: tail_point_journal) {
            if (journal_point.timestamp >
                candidate->metadata.cutoff_timestamp + kTimestampEpsilon) {
                candidate->ivox->add_point(journal_point.point);
                ++replayed_points;
            }
        }

        estimator.ivox = std::move(candidate->ivox);
        active_map_source_correction_sequence =
                candidate->metadata.source_correction_sequence;
        tracking_map_version.store(
                candidate->metadata.target_tracking_map_version,
                std::memory_order_release);
        reset_prediction_epoch();

        RCLCPP_INFO(
                logger,
                "Applied local tracking map version %lu (pgo=%lu anchor=%u "
                "voxels=%zu replayed_tail_points=%zu)",
                static_cast<unsigned long>(
                        candidate->metadata.target_tracking_map_version),
                static_cast<unsigned long>(candidate->metadata.pgo_graph_version),
                candidate->metadata.anchor_candidate_id,
                estimator.ivox->size(),
                replayed_points);
    }

    bool SmallPointLio::correction_for_sequence(
            const uint64_t sequence,
            Eigen::Isometry3d &correction) const {
        for (auto iter = correction_history.rbegin();
             iter != correction_history.rend(); ++iter) {
            if (iter->sequence == sequence) {
                correction = iter->cumulative_correction;
                return true;
            }
        }
        return false;
    }

    double SmallPointLio::rotation_angle(const Eigen::Matrix3d &rotation) {
        const double cosine = std::clamp(
                (rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
        return std::acos(cosine);
    }

    uint64_t SmallPointLio::get_tracking_map_version() const {
        return tracking_map_version.load(std::memory_order_acquire);
    }

}// namespace small_point_lio
