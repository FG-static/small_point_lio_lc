#include "small_point_lio_pgo/local_map_feedback_node.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_set>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>

namespace small_point_lio_pgo {

    namespace {

        constexpr double kTimeEpsilon = 1e-7;
        constexpr size_t kCorrectionBufferMaxMessages = 4096;

        Eigen::Matrix<double, 6, 6> informationFromDiagonal(
                const std::vector<double> &diagonal
        ) {

            Eigen::Matrix<double, 6, 6> information =
                    Eigen::Matrix<double, 6, 6>::Identity();
            if (diagonal.size() != 6)
                return information;
            for (int index = 0; index < 6; ++ index) {

                const double value = diagonal[static_cast<size_t>(index)];
                information(index, index) =
                    std::isfinite(value) && value > 0.0 ? value : 1.0;
            }
            return information;
        }

    }  // namespace

    size_t LocalMapFeedbackNode::VoxelKeyHash::operator()(
            const VoxelKey &key
    ) const {

        const auto mix = [](uint64_t value) {
            value ^= value >> 30U;
            value *= 0xbf58476d1ce4e5b9ULL;
            value ^= value >> 27U;
            value *= 0x94d049bb133111ebULL;
            return value ^ (value >> 31U);
        };
        const uint64_t hx = mix(static_cast<uint64_t>(key.x));
        const uint64_t hy = mix(static_cast<uint64_t>(key.y));
        const uint64_t hz = mix(static_cast<uint64_t>(key.z));
        return static_cast<size_t>(hx ^ (hy << 1U) ^ (hz << 7U));
    }

    LocalMapFeedbackNode::LocalMapFeedbackNode()
        : Node("small_point_lio_local_map_feedback"
    ) {

        load_parameters();

        map_publisher_ = create_publisher<
                small_point_lio_interfaces::msg::LocalTrackingMap>(
                local_map_topic_, rclcpp::QoS(1).reliable());
        correction_subscription_ = create_subscription<
                small_point_lio_interfaces::msg::ScanToMapCorrection>(
                correction_topic_,
                rclcpp::QoS(100).reliable(),
                [this](small_point_lio_interfaces::msg::
                               ScanToMapCorrection::ConstSharedPtr msg) {
                    correction_callback(std::move(msg));
                });
        candidate_subscription_ = create_subscription<
                small_point_lio_pgo::msg::KeyFrame>(
                candidate_topic_,
                rclcpp::QoS(100).reliable(),
                [this](small_point_lio_pgo::msg::KeyFrame::ConstSharedPtr msg) {
                    candidate_callback(std::move(msg));
                });
        optimized_subscription_ = create_subscription<
                small_point_lio_pgo::msg::OptimizedKeyFrames>(
                optimized_topic_,
                rclcpp::QoS(1).reliable().transient_local(),
                [this](small_point_lio_pgo::msg::
                               OptimizedKeyFrames::ConstSharedPtr msg) {
                    optimized_callback(std::move(msg));
                });

        // 局部地图优化+重建线程
        worker_thread_ = std::thread([this]() { worker_loop(); });

        RCLCPP_INFO(
                get_logger(),
                "Local map feedback: correction=%s candidates=%s "
                "optimized=%s output=%s active_window=%d history_radius=%.1fm",
                correction_topic_.c_str(),
                candidate_topic_.c_str(),
                optimized_topic_.c_str(),
                local_map_topic_.c_str(),
                active_window_keyframes_,
                history_search_radius_m_);
    }

    LocalMapFeedbackNode::~LocalMapFeedbackNode() {

        {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            stop_worker_ = true;
            queued_job_.reset();
        }
        worker_cv_.notify_all();
        if (worker_thread_.joinable())
            worker_thread_.join();
    }

    void LocalMapFeedbackNode::load_parameters() {

        const auto load_string =
            [this](
                const std::string &name,
                std::string &value
            ) {
                declare_parameter(name, value);
                get_parameter(name, value);
            };
        const auto load_int = [this](const std::string &name, int &value) {
            declare_parameter(name, value);
            get_parameter(name, value);
        };
        const auto load_double =
            [this](
                const std::string &name,
                double &value
            ) {
                declare_parameter(name, value);
                get_parameter(name, value);
            };
        const auto load_bool =
            [this](
                const std::string &name,
                bool &value
            ) {
                declare_parameter(name, value);
                get_parameter(name, value);
            };

        load_string("correction_topic", correction_topic_);
        load_string("candidate_topic", candidate_topic_);
        load_string("optimized_topic", optimized_topic_);
        load_string("local_map_topic", local_map_topic_);
        load_string("odom_frame", odom_frame_);
        load_int("evidence_window_packets", evidence_window_packets_);
        load_double(
                "evidence_window_duration_sec",
                evidence_window_duration_sec_);
        load_int("evidence_min_packets", evidence_min_packets_);
        load_int(
                "evidence_min_valid_updates",
                evidence_min_valid_updates_);
        load_double(
                "evidence_min_duration_sec",
                evidence_min_duration_sec_);
        load_double(
                "evidence_min_valid_ratio",
                evidence_min_valid_ratio_);
        load_double(
                "cumulative_trigger_translation_m",
                cumulative_trigger_translation_m_);
        load_double(
                "cumulative_trigger_rotation_deg",
                cumulative_trigger_rotation_deg_);
        load_double(
                "instant_trigger_translation_m",
                instant_trigger_translation_m_);
        load_double(
                "instant_trigger_rotation_deg",
                instant_trigger_rotation_deg_);
        load_int(
                "instant_trigger_min_valid_updates",
                instant_trigger_min_valid_updates_);
        load_double(
                "max_safe_correction_translation_m",
                max_safe_correction_translation_m_);
        load_double(
                "max_safe_correction_rotation_deg",
                max_safe_correction_rotation_deg_);
        load_double("optimization_cooldown_sec", optimization_cooldown_sec_);
        load_double("map_apply_timeout_sec", map_apply_timeout_sec_);
        load_double(
                "candidate_correction_max_delay_sec",
                candidate_correction_max_delay_sec_);
        load_double(
                "correction_buffer_duration_sec",
                correction_buffer_duration_sec_);
        load_int(
                "candidate_database_max_keyframes",
                candidate_database_max_keyframes_);
        load_int("active_window_keyframes", active_window_keyframes_);
        load_int("min_active_keyframes", min_active_keyframes_);
        load_double("history_search_radius_m", history_search_radius_m_);
        load_int("history_max_keyframes", history_max_keyframes_);
        load_double("max_local_translation_threshold", max_local_translation_threshold_);
        load_double("max_local_rotation_deg_threshold", max_local_rotation_deg_threshold_);
        load_double("map_voxel_leaf_size_m", map_voxel_leaf_size_m_);
        load_int("map_max_points", map_max_points_);
        load_int("map_min_points", map_min_points_);
        load_int("local_graph_max_iterations", local_graph_max_iterations_);
        declare_parameter("local_odom_info_diag", local_odom_info_diag_);
        get_parameter("local_odom_info_diag", local_odom_info_diag_);
        load_double(
                "pgo_deformation_trigger_translation_m",
                pgo_deformation_trigger_translation_m_);
        load_double(
                "pgo_deformation_trigger_rotation_deg",
                pgo_deformation_trigger_rotation_deg_);
        load_bool("trigger_pgo_enable", trigger_pgo_enable_);
        load_bool("trigger_instant_enable", trigger_instant_enable_);
        load_bool("trigger_normal_enable", trigger_normal_enable_);

        evidence_window_packets_ = std::max(1, evidence_window_packets_);
        evidence_window_duration_sec_ =
                std::max(0.0, evidence_window_duration_sec_);
        evidence_min_packets_ = std::max(1, evidence_min_packets_);
        evidence_min_valid_updates_ =
                std::max(1, evidence_min_valid_updates_);
        evidence_min_duration_sec_ =
                std::max(0.0, evidence_min_duration_sec_);
        evidence_min_valid_ratio_ =
                std::clamp(evidence_min_valid_ratio_, 0.0, 1.0);
        instant_trigger_min_valid_updates_ =
                std::max(1, instant_trigger_min_valid_updates_);
        candidate_database_max_keyframes_ =
                std::max(10, candidate_database_max_keyframes_);
        active_window_keyframes_ = std::max(2, active_window_keyframes_);
        min_active_keyframes_ = std::clamp(
                min_active_keyframes_, 2, active_window_keyframes_);
        history_max_keyframes_ = std::max(0, history_max_keyframes_);
        max_local_translation_threshold_ = std::max(0.0, max_local_translation_threshold_);
        max_local_rotation_deg_threshold_ = std::max(0.0, max_local_rotation_deg_threshold_);
        map_voxel_leaf_size_m_ = std::max(0.01, map_voxel_leaf_size_m_);
        map_max_points_ = std::max(100, map_max_points_);
        map_min_points_ = std::clamp(map_min_points_, 1, map_max_points_);
        local_graph_max_iterations_ =
                std::max(1, local_graph_max_iterations_);
        if (local_odom_info_diag_.size() != 6) {
            local_odom_info_diag_ =
                    {300.0, 300.0, 150.0, 120.0, 120.0, 300.0};
        }
    }

    // 收到若干个 LiDAR 包内有关 scantomap 的矫正信息
    void LocalMapFeedbackNode::correction_callback(
        small_point_lio_interfaces::msg::ScanToMapCorrection::
            ConstSharedPtr msg
    ) {

        if (!msg || msg->header.frame_id != odom_frame_)
            return;

        CorrectionRecord record;
        record.stamp = stamp_to_seconds(msg->header.stamp);
        record.sequence = msg->sequence;
        record.tracking_map_version = msg->tracking_map_version;
        record.active_map_source_correction_sequence =
                msg->active_map_source_correction_sequence;
        record.packet_predicted = pose_to_isometry(msg->packet_predicted_pose);
        record.epoch_predicted = pose_to_isometry(msg->epoch_predicted_pose);
        record.corrected = pose_to_isometry(msg->corrected_pose);
        record.attempted_updates = msg->attempted_point_updates;
        record.accepted_updates = msg->accepted_point_updates;
        record.residual_rms = msg->residual_rms;
        record.residual_max_abs = msg->residual_max_abs;
        if (!std::isfinite(record.stamp) ||
            !finite_pose(record.packet_predicted) ||
            !finite_pose(record.epoch_predicted) ||
            !finite_pose(record.corrected))
            return;

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (has_correction_ &&
            record.stamp + kTimeEpsilon < last_correction_stamp_)
            reset_runtime_state_locked("correction time moved backwards");

        if (!has_correction_)
            active_tracking_map_version_ = record.tracking_map_version;
        else if (
            record.tracking_map_version <
                active_tracking_map_version_
        ) return;
        else if (
            record.tracking_map_version >
                active_tracking_map_version_
        ) {

            acknowledge_map_version_locked(
                record.tracking_map_version,
                record.active_map_source_correction_sequence
            );
        }

        has_correction_ = true;
        last_correction_stamp_ = record.stamp;
        corrections_.push_back(record);
        while (
            !corrections_.empty() && (
                    corrections_.size() > kCorrectionBufferMaxMessages ||
                        record.stamp - corrections_.front().stamp >
                            correction_buffer_duration_sec_
                )
        ) corrections_.pop_front();

        // candidate 和 correction 是异步到达数据流，两个无论哪个先来最后都做一次联合
        associate_candidate_predictions_locked();
        schedule_if_needed_locked(record);
    }

    void LocalMapFeedbackNode::candidate_callback(
            small_point_lio_pgo::msg::KeyFrame::ConstSharedPtr msg
    ) {

        if (!msg || msg->header.frame_id != odom_frame_)
            return;

        CandidateRecord candidate;
        candidate.id = msg->id;
        candidate.stamp = stamp_to_seconds(msg->header.stamp);
        candidate.raw_pose = pose_to_isometry(msg->pose);
        candidate.cloud_body =
            std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(msg->cloud, *candidate.cloud_body);
        if (!std::isfinite(candidate.stamp) ||
            !finite_pose(candidate.raw_pose) || candidate.cloud_body->empty())
            return;

        candidate.cloud_body->erase(
            std::remove_if(
                candidate.cloud_body->begin(),
                candidate.cloud_body->end(),
                [](const pcl::PointXYZ &point) {
                    return !std::isfinite(point.x) ||
                        !std::isfinite(point.y) ||
                        !std::isfinite(point.z);
                }),
            candidate.cloud_body->end()
        );
        if (candidate.cloud_body->empty())
            return;

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!candidates_.empty() &&
            candidate.stamp + kTimeEpsilon < candidates_.back().stamp &&
            candidate.id <= candidates_.back().id)
            reset_runtime_state_locked("candidate time/id moved backwards");

        for (auto &stored: candidates_) {

            if (stored.id == candidate.id) {

                candidate.has_epoch_prediction = stored.has_epoch_prediction;
                candidate.prediction_map_version =
                    stored.prediction_map_version;
                candidate.epoch_prediction = stored.epoch_prediction;
                stored = std::move(candidate);
                associate_candidate_prediction_locked(stored);
                return;
            }
        }

        associate_candidate_prediction_locked(candidate);
        candidates_.push_back(std::move(candidate));
        while (candidates_.size() >
            static_cast<size_t>(candidate_database_max_keyframes_)
        ) {

            local_pose_overrides_.erase(candidates_.front().id);
            candidates_.pop_front();
        }
    }

    void LocalMapFeedbackNode::optimized_callback(
        small_point_lio_pgo::msg::OptimizedKeyFrames::ConstSharedPtr msg
    ) {

        if (!msg || msg->ids.size() != msg->poses.size() ||
            msg->ids.empty())
            return;

        std::unordered_map<uint32_t, Eigen::Isometry3d> optimized;
        optimized.reserve(msg->ids.size());
        for (size_t index = 0; index < msg->ids.size(); ++ index) {

            const Eigen::Isometry3d pose = pose_to_isometry(msg->poses[index]);
            if (finite_pose(pose))
                optimized[msg->ids[index]] = pose;
        }
        if (optimized.empty()) return;

        std::lock_guard<std::mutex> lock(state_mutex_);
        const uint64_t incoming_version =
            msg->graph_version > 0
                ? msg->graph_version
                : pgo_graph_version_ + 1;
        if (incoming_version <= pgo_graph_version_)
            return;

        const bool significant =
            pgo_deformation_significant_locked(
                optimized_poses_, optimized
            );
        optimized_poses_ = std::move(optimized);
        pgo_graph_version_ = incoming_version;
        pgo_rebuild_pending_ = pgo_rebuild_pending_ || significant; // 标记表示pgo大幅度变动过

        RCLCPP_INFO(
                get_logger(),
                "Received PGO graph version %lu (%zu poses, local_deformation=%d)",
                static_cast<unsigned long>(pgo_graph_version_),
                optimized_poses_.size(),
                significant ? 1 : 0);

        // pgo 处理完后进行一次可重建判断
        if (has_correction_ && !corrections_.empty())
            schedule_if_needed_locked(corrections_.back());
    }

    void LocalMapFeedbackNode::reset_runtime_state_locked(
            const std::string &reason
    ) {

        ++ runtime_generation_;
        corrections_.clear();
        candidates_.clear();
        local_pose_overrides_.clear();
        optimized_poses_.clear();
        pgo_graph_version_ = 0;
        applied_pgo_graph_version_ = 0;
        active_tracking_map_version_ = 0;
        last_correction_stamp_ = -1.0;
        has_correction_ = false;
        pgo_rebuild_pending_ = false;
        build_in_progress_ = false;
        awaiting_map_apply_ = false;
        awaiting_target_version_ = 0;
        pending_pose_commit_.reset();
        {
            std::lock_guard<std::mutex> worker_lock(worker_mutex_);
            queued_job_.reset();
        }
        RCLCPP_WARN(get_logger(), "Reset local feedback state: %s", reason.c_str());
    }

    void LocalMapFeedbackNode::acknowledge_map_version_locked(
        const uint64_t active_version,
        const uint64_t active_map_source_correction_sequence
    ) {

        const bool expected = awaiting_map_apply_ &&
                              active_version == awaiting_target_version_ &&
                              pending_pose_commit_ &&
                              pending_pose_commit_->target_tracking_map_version ==
                                      active_version &&
                              pending_pose_commit_->source_tracking_map_version + 1 ==
                                      active_version &&
                              active_map_source_correction_sequence ==
                                      pending_pose_commit_
                                              ->source_correction_sequence;
        if (expected) {

            for (const auto &[id, pose]: pending_pose_commit_->poses) {

                local_pose_overrides_[id] = LocalPoseOverride{
                    active_version,
                    pending_pose_commit_->pgo_graph_version,
                    pose
                };
            }
            applied_pgo_graph_version_ =
                    pending_pose_commit_->pgo_graph_version;
            last_map_applied_time_ = SteadyClock::now();
            RCLCPP_INFO(
                    get_logger(),
                    "Frontend acknowledged local tracking-map version %lu",
                    static_cast<unsigned long>(active_version));
        } else {

            local_pose_overrides_.clear();
            RCLCPP_WARN(
                    get_logger(),
                    "Observed unrequested tracking-map version change %lu -> %lu "
                    "(applied source correction=%lu)",
                    static_cast<unsigned long>(active_tracking_map_version_),
                    static_cast<unsigned long>(active_version),
                    static_cast<unsigned long>(
                            active_map_source_correction_sequence));
        }

        active_tracking_map_version_ = active_version;
        awaiting_map_apply_ = false;
        awaiting_target_version_ = 0;
        pending_pose_commit_.reset();
        corrections_.clear();
        pgo_rebuild_pending_ =
                pgo_graph_version_ > applied_pgo_graph_version_;
        for (auto &candidate: candidates_) {

            if (candidate.prediction_map_version != active_version)
                candidate.has_epoch_prediction = false;
        }
    }

    // @brief 遍历所有候选关键帧，找到时间上最接近矫正记录的关键帧
    void LocalMapFeedbackNode::associate_candidate_predictions_locked() {

        for (auto iter = candidates_.rbegin(); iter != candidates_.rend(); ++ iter) {

            if (has_correction_ &&
                    last_correction_stamp_ - iter->stamp >
                        correction_buffer_duration_sec_)
                break;
            associate_candidate_prediction_locked(*iter);
        }
    }

    bool LocalMapFeedbackNode::associate_candidate_prediction_locked(
            CandidateRecord &candidate
    ) {

        const CorrectionRecord *best = nullptr;
        double best_delay = std::numeric_limits<double>::infinity();
        for (auto iter = corrections_.rbegin(); iter != corrections_.rend(); ++ iter) {

            const double delay = std::abs(iter->stamp - candidate.stamp);
            if (delay < best_delay) {

                best = &*iter;
                best_delay = delay;
            }
            if (iter->stamp + candidate_correction_max_delay_sec_ <
                candidate.stamp)
                break;

        }
        if (!best || best_delay > candidate_correction_max_delay_sec_)
            return false;
        candidate.has_epoch_prediction = true;
        candidate.prediction_map_version = best->tracking_map_version;
        candidate.epoch_prediction = best->epoch_predicted;
        return true;
    }

    // @brief 使用最新矫正记录收集需优化证据
    // @param latest 最新的矫正记录
    // @return LocalMapFeedbackNode::EvidenceSummary 矫正质量摘要
    LocalMapFeedbackNode::EvidenceSummary
        LocalMapFeedbackNode::evidence_summary_locked(
            const CorrectionRecord &latest
    ) const {

        EvidenceSummary summary;
        double oldest_stamp = latest.stamp;
        for (auto iter = corrections_.rbegin(); iter != corrections_.rend(); ++ iter) {

            if (iter->tracking_map_version != latest.tracking_map_version)
                continue;
            // 证据链包数够了 / 时间超阈值就退出
            if (summary.packet_count >=
                    static_cast<size_t>(evidence_window_packets_) ||
                latest.stamp - iter->stamp > evidence_window_duration_sec_)
                break;
            ++ summary.packet_count;
            summary.attempted_updates += iter->attempted_updates;
            summary.accepted_updates += iter->accepted_updates;
            oldest_stamp = iter->stamp;
        }
        summary.duration_sec = std::max(0.0, latest.stamp - oldest_stamp);
        if (summary.attempted_updates > 0) {

            summary.valid_ratio =
                    static_cast<double>(summary.accepted_updates) /
                    static_cast<double>(summary.attempted_updates);
        }
        return summary;
    }

    // @brief 是否应该重建地图？是则打包为BuildJob丢给工作线程
    bool LocalMapFeedbackNode::schedule_if_needed_locked(
        const CorrectionRecord &latest
    ) {

        const auto now_steady = SteadyClock::now();
        if (awaiting_map_apply_ &&
            std::chrono::duration<double>(now_steady - awaiting_since_).count() >
                    map_apply_timeout_sec_) {

            RCLCPP_WARN(
                    get_logger(),
                    "Timed out waiting for tracking-map version %lu; rearming",
                    static_cast<unsigned long>(awaiting_target_version_));
            if (pending_pose_commit_ &&
                pending_pose_commit_->pgo_graph_version >
                        applied_pgo_graph_version_) // pgo graph version 有变动就说明此次进行了 pgo
                pgo_rebuild_pending_ = true;

            awaiting_map_apply_ = false;
            awaiting_target_version_ = 0;
            pending_pose_commit_.reset();
        }
        if (build_in_progress_ || awaiting_map_apply_)
            return false;

        if (last_map_applied_time_ != SteadyClock::time_point::min() &&
            std::chrono::duration<double>(now_steady - last_map_applied_time_)
                            .count() < optimization_cooldown_sec_)
            return false;

        const Eigen::Isometry3d packet_correction =
                latest.corrected * latest.packet_predicted.inverse();
        const Eigen::Isometry3d cumulative =
                latest.corrected * latest.epoch_predicted.inverse();
        const double packet_translation = packet_correction.translation().norm();
        const double packet_rotation_deg =
                rotation_angle(packet_correction.rotation()) * 180.0 / M_PI;
        const double cumulative_translation = cumulative.translation().norm();
        const double cumulative_rotation_deg =
                rotation_angle(cumulative.rotation()) * 180.0 / M_PI;

        const bool unsafe =
                cumulative_translation > max_safe_correction_translation_m_ ||
                cumulative_rotation_deg > max_safe_correction_rotation_deg_ ||
                packet_translation > max_safe_correction_translation_m_ ||
                packet_rotation_deg > max_safe_correction_rotation_deg_;
        if (unsafe && !pgo_rebuild_pending_) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Not feeding back unsafe scan correction: packet=[%.3fm %.2fdeg] "
                    "cumulative=[%.3fm %.2fdeg]",
                    packet_translation,
                    packet_rotation_deg,
                    cumulative_translation,
                    cumulative_rotation_deg);
            return false;
        }

        const EvidenceSummary evidence = evidence_summary_locked(latest);
        const bool evidence_ready =
                evidence.packet_count >=
                        static_cast<size_t>(evidence_min_packets_) &&
                evidence.accepted_updates >=
                        static_cast<uint64_t>(evidence_min_valid_updates_) &&
                evidence.duration_sec >= evidence_min_duration_sec_ &&
                evidence.valid_ratio >= evidence_min_valid_ratio_;
        const bool normal_trigger =
                evidence_ready &&
                (cumulative_translation >=
                         cumulative_trigger_translation_m_ ||
                 cumulative_rotation_deg >=
                         cumulative_trigger_rotation_deg_);
        const bool instant_trigger =
                latest.accepted_updates >=
                        static_cast<uint32_t>(
                                instant_trigger_min_valid_updates_) &&
                (packet_translation >= instant_trigger_translation_m_ ||
                 packet_rotation_deg >= instant_trigger_rotation_deg_);

        std::string reason;
        if (pgo_rebuild_pending_ && trigger_pgo_enable_) {

            reason = "pgo";
        } else if (instant_trigger && trigger_instant_enable_) {

            reason = "instant_scan_correction";
        } else if (normal_trigger && trigger_normal_enable_) {

            reason = "cumulative_scan_correction";
        } else {

            return false;
        }

        auto job = make_build_job_locked(latest, reason);
        if (!job) {

            RCLCPP_DEBUG(
                    get_logger(),
                    "Feedback trigger %s is waiting for enough synchronized keyframes",
                    reason.c_str());
            return false;
        }

        build_in_progress_ = true;
        if (job->pgo_graph_version >= pgo_graph_version_)
            pgo_rebuild_pending_ = false;
        {
            std::lock_guard<std::mutex> worker_lock(worker_mutex_);
            queued_job_ = std::move(*job);
        }
        worker_cv_.notify_one();

        RCLCPP_INFO(
                get_logger(),
                "Scheduled local feedback (%s): packets=%zu valid=%lu ratio=%.3f "
                "packet=[%.3fm %.2fdeg] cumulative=[%.3fm %.2fdeg]",
                reason.c_str(),
                evidence.packet_count,
                static_cast<unsigned long>(evidence.accepted_updates),
                evidence.valid_ratio,
                packet_translation,
                packet_rotation_deg,
                cumulative_translation,
                cumulative_rotation_deg);
        return true;
    }

    std::optional<LocalMapFeedbackNode::BuildJob>
        LocalMapFeedbackNode::make_build_job_locked(
            const CorrectionRecord &latest,
            const std::string &reason
    ) {

        const auto seeds = seed_poses_locked(
                latest.tracking_map_version,
                optimized_poses_,
                pgo_graph_version_);

        const bool new_pgo_snapshot =
                reason == "pgo" &&
                pgo_graph_version_ > applied_pgo_graph_version_;

        double latest_pgo_stamp =
                -std::numeric_limits<double>::infinity();

        if (new_pgo_snapshot) {

            for (const auto &candidate : candidates_) {

                if (optimized_poses_.find(candidate.id) !=
                        optimized_poses_.end()) {

                    latest_pgo_stamp =
                            std::max(latest_pgo_stamp, candidate.stamp);
                }
            }
        }

        // 构建局部可调整关键帧容器
        std::vector<const CandidateRecord *> active_candidates;
        active_candidates.reserve(static_cast<size_t>(active_window_keyframes_));
        for (auto iter = candidates_.rbegin(); iter != candidates_.rend(); ++ iter) {

            if (iter->stamp > latest.stamp + kTimeEpsilon ||
                !iter->has_epoch_prediction ||
                iter->prediction_map_version != latest.tracking_map_version)
                continue;
            active_candidates.push_back(&*iter);
            if (active_candidates.size() >=
                static_cast<size_t>(active_window_keyframes_))
                break;
        }
        if (active_candidates.size() <
            static_cast<size_t>(min_active_keyframes_))
            return std::nullopt;
        std::reverse(active_candidates.begin(), active_candidates.end());

        BuildJob job;
        job.correction = latest;
        job.runtime_generation = runtime_generation_;
        job.pgo_graph_version = pgo_graph_version_;
        job.trigger_reason = reason;
        std::unordered_set<uint32_t> active_ids;
        active_ids.reserve(active_candidates.size());
        for (const auto *candidate : active_candidates) {

            const auto seed_iter = seeds.find(candidate->id);
            const Eigen::Isometry3d initial = seed_iter != seeds.end()
                                                        ? seed_iter->second
                                                        : candidate->raw_pose;
            if (!finite_pose(initial) ||
                !finite_pose(candidate->epoch_prediction))
                continue;
            FrameSnapshot frame;
            frame.candidate_id = candidate->id;
            frame.stamp = candidate->stamp;
            frame.raw_pose = candidate->raw_pose;
            frame.pgo_covered =
                new_pgo_snapshot &&
                std::isfinite(latest_pgo_stamp) &&
                candidate->stamp <= latest_pgo_stamp + kTimeEpsilon;
            frame.initial_pose = initial;
            frame.epoch_prediction = candidate->epoch_prediction;
            frame.cloud_body = candidate->cloud_body;
            frame.distance_to_current =
                    (initial.translation() - latest.corrected.translation())
                            .norm();
            job.active_frames.push_back(std::move(frame));
            active_ids.insert(candidate->id);
        }
        if (job.active_frames.size() <
            static_cast<size_t>(min_active_keyframes_))
            return std::nullopt;
        if (new_pgo_snapshot) {

            const size_t pgo_covered_count = static_cast<size_t>(
                std::count_if(
                    job.active_frames.begin(),
                    job.active_frames.end(),
                    [](const FrameSnapshot &frame) {
                        return frame.pgo_covered;
                    }));
            double max_local_translation_delta = 0.0;
            double max_local_rotation_deg_delta = 0.0;
            // 最后一个被全局 PGO 覆盖的索引
            int anchor_index = -1;
            for (int index = static_cast<int>(job.active_frames.size()) - 1; index >= 0; -- index) {

                if (job.active_frames[static_cast<size_t>(index)].pgo_covered) {

                    anchor_index = index;
                    break;
                }
            }
            if (anchor_index >= 0) {

                // 通过 frame 得到 位姿
                const auto baseline_pose =
                    [this, &latest](const FrameSnapshot &frame) {

                        const auto iter =
                            local_pose_overrides_.find(frame.candidate_id);
                        if (iter != local_pose_overrides_.end() &&
                            iter->second.tracking_map_version == latest.tracking_map_version &&
                            finite_pose(iter->second.pose))
                            return iter->second.pose;
                        return frame.raw_pose;
                    };

                // 锚点关键帧
                const FrameSnapshot &anchor =
                    job.active_frames[static_cast<size_t>(anchor_index)];

                const Eigen::Isometry3d baseline_anchor =
                    baseline_pose(anchor);
                const Eigen::Isometry3d new_anchor =
                    anchor.initial_pose;

                for (const auto &frame : job.active_frames) {

                    if (!frame.pgo_covered) continue;

                    // 对于滑窗内所有的帧
                    const Eigen::Isometry3d baseline = baseline_pose(frame);
                    // 全局 PGO 优化前的与 anchor 帧间形变量
                    const Eigen::Isometry3d old_relative = baseline_anchor.inverse() * baseline;
                    // 全局 PGO 优化后的与 anchor 帧间形变量
                    const Eigen::Isometry3d new_relative = new_anchor.inverse() * frame.initial_pose;
                    // 如果全局 PGO 对所有帧都是刚体变换，那么 delta 的结果是 0 （意味着帧间几何关系/连续性一致，刚体变换会抵消），否则非 0
                    const Eigen::Isometry3d delta = old_relative.inverse() * new_relative;
                    const double translation_delta = delta.translation().norm();
                    const double rotation_deg_delta = rotation_angle(delta.rotation()) * 180.0 / M_PI;
                    max_local_translation_delta =
                        std::max(max_local_translation_delta, translation_delta);
                    max_local_rotation_deg_delta =
                        std::max(max_local_rotation_deg_delta, rotation_deg_delta);
                    if (translation_delta >=
                                max_local_translation_threshold_ ||
                        rotation_deg_delta >=
                                max_local_rotation_deg_threshold_) {

                        // reanchor 模式是为了修正全局 PGO 的错误优化，大部分情况下 small_correction 已够用
                        job.pgo_reanchor = true;
                    }
                }
            }
            RCLCPP_INFO(
                get_logger(),
                "PGO feedback mode: version=%lu mode=%s "
                "local_delta=[%.3fm %.2fdeg] covered=%zu/%zu",
                static_cast<unsigned long>(job.pgo_graph_version),
                job.pgo_reanchor ? "large_reanchor" : "small_correction",
                max_local_translation_delta,
                max_local_rotation_deg_delta,
                pgo_covered_count,
                job.active_frames.size());
        }
        // 历史关键帧加入冻结容器
        for (const auto &candidate : candidates_) {

            if (candidate.stamp > latest.stamp + kTimeEpsilon ||
                active_ids.find(candidate.id) != active_ids.end())
                continue;
            const auto seed_iter = seeds.find(candidate.id);
            const Eigen::Isometry3d initial = seed_iter != seeds.end()
                                                        ? seed_iter->second
                                                        : candidate.raw_pose;
            const double distance =
                    (initial.translation() - latest.corrected.translation())
                            .norm();
            if (!finite_pose(initial) || distance > history_search_radius_m_)
                continue;
            FrameSnapshot frame;
            frame.candidate_id = candidate.id;
            frame.stamp = candidate.stamp;
            frame.initial_pose = initial;
            frame.cloud_body = candidate.cloud_body;
            frame.distance_to_current = distance;
            job.frozen_history.push_back(std::move(frame));
        }
        std::sort(
                job.frozen_history.begin(),
                job.frozen_history.end(),
                [](const FrameSnapshot &lhs, const FrameSnapshot &rhs) {
                    if (lhs.distance_to_current != rhs.distance_to_current) {
                        return lhs.distance_to_current < rhs.distance_to_current;
                    }
                    return lhs.candidate_id < rhs.candidate_id;
                });
        if (job.frozen_history.size() >
            static_cast<size_t>(history_max_keyframes_)) {
            job.frozen_history.resize(
                    static_cast<size_t>(history_max_keyframes_));
        }
        return job;
    }

    std::unordered_map<uint32_t, Eigen::Isometry3d>
        LocalMapFeedbackNode::seed_poses_locked(
            const uint64_t tracking_map_version,
            const std::unordered_map<uint32_t, Eigen::Isometry3d>
                &optimized_poses,
            const uint64_t optimized_version
    ) const {

        std::unordered_map<uint32_t, Eigen::Isometry3d> seeds;
        seeds.reserve(candidates_.size());

        const CandidateRecord *anchor = nullptr;
        for (const auto &candidate : candidates_) {

            if (optimized_poses.find(candidate.id) == optimized_poses.end())
                continue;
            if (!anchor || candidate.stamp > anchor->stamp)
                anchor = &candidate;
        }

        if (!anchor) {

            for (const auto &candidate : candidates_) {

                const auto override_iter =
                        local_pose_overrides_.find(candidate.id);
                if (override_iter != local_pose_overrides_.end() &&
                    override_iter->second.tracking_map_version ==
                            tracking_map_version) {

                    seeds[candidate.id] = override_iter->second.pose;
                } else {

                    seeds[candidate.id] = candidate.raw_pose;
                }
            }
            return seeds;
        }

        const Eigen::Isometry3d odom_from_map =
                anchor->raw_pose * optimized_poses.at(anchor->id).inverse();
        /// Samples the PGO-derived correction field at an optimized keyframe;
        /// neighbouring samples interpolate corrections for non-PGO candidates.
        struct CorrectionAnchor {
            double stamp = 0.0;///< Optimized keyframe time, in seconds.
            uint32_t id = 0;   ///< Keyframe identifier retained for traceability.
            Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();///< seed * raw_pose^-1.
            Eigen::Isometry3d seed = Eigen::Isometry3d::Identity();///< PGO pose expressed in odom.
        };
        std::vector<CorrectionAnchor> anchors;
        for (const auto &candidate : candidates_) {

            const auto optimized_iter = optimized_poses.find(candidate.id);
            if (optimized_iter == optimized_poses.end())
                continue;
            const Eigen::Isometry3d seed =
                    odom_from_map * optimized_iter->second;
            anchors.push_back(CorrectionAnchor{
                    candidate.stamp,
                    candidate.id,
                    seed * candidate.raw_pose.inverse(),
                    seed});
        }
        std::sort(
                anchors.begin(),
                anchors.end(),
                [](const CorrectionAnchor &lhs, const CorrectionAnchor &rhs) {
                    return lhs.stamp < rhs.stamp;
                });

        const bool new_pgo_snapshot =
                optimized_version > applied_pgo_graph_version_;
        for (const auto &candidate : candidates_) {

            const auto override_iter = local_pose_overrides_.find(candidate.id);
            if (!new_pgo_snapshot &&
                override_iter != local_pose_overrides_.end() &&
                override_iter->second.tracking_map_version ==
                        tracking_map_version) {

                seeds[candidate.id] = override_iter->second.pose;
                continue;
            }

            const auto exact = optimized_poses.find(candidate.id);
            if (exact != optimized_poses.end()) {

                seeds[candidate.id] = odom_from_map * exact->second;
                continue;
            }

            const CorrectionAnchor *before = nullptr;
            const CorrectionAnchor *after = nullptr;
            for (const auto &correction_anchor : anchors) {

                if (correction_anchor.stamp <= candidate.stamp) {

                    before = &correction_anchor;
                }
                if (correction_anchor.stamp >= candidate.stamp) {

                    after = &correction_anchor;
                    break;
                }
            }
            Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
            if (before && after && after->stamp > before->stamp) {

                const double alpha = std::clamp(
                        (candidate.stamp - before->stamp) /
                                (after->stamp - before->stamp),
                        0.0,
                        1.0);
                correction = interpolate_correction(
                        before->correction, after->correction, alpha);
            } else if (before) {

                correction = before->correction;
            } else if (after) {

                correction = after->correction;
            }
            seeds[candidate.id] = correction * candidate.raw_pose;
        }
        return seeds;
    }

    // @brief 判断差异决定是否需要重建
    bool LocalMapFeedbackNode::pgo_deformation_significant_locked(
            const std::unordered_map<uint32_t, Eigen::Isometry3d> &previous,
            const std::unordered_map<uint32_t, Eigen::Isometry3d> &current
    ) const {

        if (previous.empty()) return true;

        const CandidateRecord *anchor = nullptr;
        for (const auto &candidate : candidates_) {

            if (previous.find(candidate.id) == previous.end() ||
                current.find(candidate.id) == current.end())
                continue;
            if (!anchor || candidate.stamp > anchor->stamp)
                anchor = &candidate;
        }
        if (!anchor) return true;

        const Eigen::Isometry3d odom_from_previous =
                anchor->raw_pose * previous.at(anchor->id).inverse();
        const Eigen::Isometry3d odom_from_current =
                anchor->raw_pose * current.at(anchor->id).inverse();
        const Eigen::Vector3d current_position =
                corrections_.empty()
                        ? anchor->raw_pose.translation()
                        : corrections_.back().corrected.translation();

        for (const auto &candidate : candidates_) {

            const auto old_iter = previous.find(candidate.id);
            const auto new_iter = current.find(candidate.id);
            if (new_iter == current.end())
                continue;
            const Eigen::Isometry3d new_local =
                    odom_from_current * new_iter->second;
            if ((new_local.translation() - current_position).norm() >
                history_search_radius_m_) {

                continue;
            }
            if (old_iter == previous.end())
                return true;

            const Eigen::Isometry3d old_local =
                    odom_from_previous * old_iter->second;
            const Eigen::Isometry3d delta =
                    new_local * old_local.inverse();
            const double rotation_deg =
                    rotation_angle(delta.rotation()) * 180.0 / M_PI;
            if (delta.translation().norm() >=
                        pgo_deformation_trigger_translation_m_ ||
                rotation_deg >= pgo_deformation_trigger_rotation_deg_) {

                return true;
            }
        }
        return false;
    }

    void LocalMapFeedbackNode::worker_loop() {

        while (true) {

            BuildJob job;
            {
                std::unique_lock<std::mutex> lock(worker_mutex_);
                worker_cv_.wait(
                        lock,
                        [this]() { return stop_worker_ || queued_job_.has_value(); });
                if (stop_worker_) return;
                job = std::move(*queued_job_);
                queued_job_.reset();
            }

            small_point_lio_interfaces::msg::LocalTrackingMap message;
            PendingPoseCommit commit;
            const bool success = execute_build_job(job, message, commit);
            bool publish = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                // 防止旧代次 job 进入
                const bool current_generation =
                        runtime_generation_ == job.runtime_generation;
                // A rosbag time reset can queue a new-generation job while an
                // old worker is still finishing. The old result must neither
                // clear that job's busy flag nor resurrect its PGO request.
                if (current_generation) {
                    build_in_progress_ = false;
                }
                // Publish only if the job still matches the active runtime,
                // PGO graph, and tracking-map version.
                if (success && current_generation &&
                    job.pgo_graph_version == pgo_graph_version_ &&
                    active_tracking_map_version_ ==
                            job.correction.tracking_map_version &&
                    !awaiting_map_apply_) {

                    awaiting_map_apply_ = true;
                    awaiting_target_version_ =
                            commit.target_tracking_map_version;
                    awaiting_since_ = SteadyClock::now();
                    pending_pose_commit_ = std::move(commit);
                    publish = true;
                } else if (current_generation &&
                           pgo_graph_version_ >
                           applied_pgo_graph_version_) {

                    pgo_rebuild_pending_ = true; // 标记待重建
                }
            }
            // 发布地图给前端
            if (publish) map_publisher_->publish(message);
        }
    }

    // @brief 执行工作
    // @param job 交付的工作
    // @param message 重建后的局部地图
    // @param commit 优化后的位姿
    // @return bool 是否成功
    bool LocalMapFeedbackNode::execute_build_job(
        const BuildJob &job,
        small_point_lio_interfaces::msg::LocalTrackingMap &message,
        PendingPoseCommit &commit
    ) {

        std::vector<Eigen::Isometry3d> active_optimized;
        if (!optimize_active_window(job, active_optimized)) {

            RCLCPP_WARN(
                    get_logger(),
                    "Local window optimization failed for correction %lu",
                    static_cast<unsigned long>(job.correction.sequence));
            return false;
        }

        const auto cloud = build_tracking_cloud(job, active_optimized);
        if (!cloud || cloud->size() < static_cast<size_t>(map_min_points_)) {

            RCLCPP_WARN(
                    get_logger(),
                    "Local tracking-map build produced too few points");
            return false;
        }

        // The rebuilt cloud contains keyframe clouds, not every point up to
        // the correction time. Tail replay must therefore start after the
        // latest keyframe actually included in this cloud.
        double map_cutoff_timestamp = job.active_frames.front().stamp;
        for (const auto &frame : job.active_frames) {

            map_cutoff_timestamp =
                    std::max(map_cutoff_timestamp, frame.stamp);
        }
        for (const auto &frame : job.frozen_history) {

            map_cutoff_timestamp =
                    std::max(map_cutoff_timestamp, frame.stamp);
        }

        // 整理发给前端的地图
        const int64_t stamp_ns = static_cast<int64_t>(
                std::llround(map_cutoff_timestamp * 1e9));
        message.header.stamp.sec =
                static_cast<int32_t>(stamp_ns / 1000000000LL);
        message.header.stamp.nanosec =
                static_cast<uint32_t>(stamp_ns % 1000000000LL);
        message.header.frame_id = odom_frame_;
        message.source_correction_sequence = job.correction.sequence;
        message.source_tracking_map_version =
                job.correction.tracking_map_version;
        message.target_tracking_map_version =
                job.correction.tracking_map_version + 1;
        message.pgo_graph_version = job.pgo_graph_version;
        message.anchor_candidate_id = job.active_frames.front().candidate_id;
        message.trigger_reason = job.trigger_reason;
        pcl::toROSMsg(*cloud, message.cloud);
        message.cloud.header = message.header;

        // 整理留在后端的优化位姿
        commit.source_correction_sequence = job.correction.sequence;
        commit.source_tracking_map_version =
                job.correction.tracking_map_version;
        commit.target_tracking_map_version =
                job.correction.tracking_map_version + 1;
        commit.pgo_graph_version = job.pgo_graph_version;
        for (size_t index = 0; index < job.active_frames.size(); ++ index) {

            commit.poses[job.active_frames[index].candidate_id] =
                active_optimized[index];
        }

        RCLCPP_INFO(
                get_logger(),
                "Built local tracking map %lu -> %lu (%s): active=%zu "
                "history=%zu points=%zu",
                static_cast<unsigned long>(
                        message.source_tracking_map_version),
                static_cast<unsigned long>(
                        message.target_tracking_map_version),
                job.trigger_reason.c_str(),
                job.active_frames.size(),
                job.frozen_history.size(),
                cloud->size());
        return true;
    }

    // 局部滑窗优化 keyframe
    bool LocalMapFeedbackNode::optimize_active_window(
            const BuildJob &job,
            std::vector<Eigen::Isometry3d> &optimized_poses
    ) const {

        if (job.active_frames.size() < 2)
            return false;

        // 大 PGO 且 active 全部已经被 PGO 覆盖时，
        // 直接使用 initial_pose，不再让终端 corrected 改写 PGO 结果。
        bool has_free_active_node = false;
        for (const auto &frame : job.active_frames) {

            if (!job.pgo_reanchor || !frame.pgo_covered) {

                has_free_active_node = true;
                break;
            }
        }
        if (job.pgo_reanchor && !has_free_active_node) {

            optimized_poses.clear();
            optimized_poses.reserve(job.active_frames.size());
            for (const auto &frame : job.active_frames)
                optimized_poses.push_back(frame.initial_pose);
            return true;
        }

        PoseGraph graph;
        PoseGraphOptions options;
        options.max_iterations = local_graph_max_iterations_;
        options.fix_first_node = true;
        graph.configure(options);

        for (size_t index = 0; index < job.active_frames.size(); ++ index) {

            PoseGraphNode node;
            node.id = static_cast<uint32_t>(index);
            node.stamp = job.active_frames[index].stamp;
            node.pose = job.active_frames[index].initial_pose;
            node.fixed =
                    job.pgo_reanchor &&
                    job.active_frames[index].pgo_covered;
            if (!graph.addNode(node))
                return false;
        }
        PoseGraphNode current;
        current.id = static_cast<uint32_t>(job.active_frames.size());
        current.stamp = job.correction.stamp;
        current.pose = job.correction.corrected;
        current.fixed = true;
        if (!graph.addNode(current))
            return false;

        const Eigen::Matrix<double, 6, 6> information =
                informationFromDiagonal(local_odom_info_diag_);
        for (size_t index = 1; index < job.active_frames.size(); ++ index) {

            const auto &previous = job.active_frames[index - 1];
            const auto &current_frame = job.active_frames[index];
            const bool preserve_pgo_edge =
                    job.pgo_reanchor &&
                    previous.pgo_covered &&
                    current_frame.pgo_covered;
            const Eigen::Isometry3d relative =
                    preserve_pgo_edge
                        ? previous.initial_pose.inverse() *
                                current_frame.initial_pose
                        : previous.raw_pose.inverse() *
                                current_frame.raw_pose;
            if (!graph.addOdomEdge(
                    static_cast<uint32_t>(index - 1),
                    static_cast<uint32_t>(index),
                    relative,
                    information))
                return false;
        }
        const Eigen::Isometry3d terminal_relative =
                job.active_frames.back().raw_pose.inverse() *
                job.correction.packet_predicted;
        if (!graph.addOdomEdge(
                    static_cast<uint32_t>(job.active_frames.size() - 1),
                    current.id,
                    terminal_relative,
                    information)) {

            return false;
        }

        // 构建好 PoseGraph 优化
        const PoseGraphOptimizationSummary summary = graph.optimize();
        if (!summary.success) return false;
        optimized_poses.resize(job.active_frames.size());
        for (size_t index = 0; index < job.active_frames.size(); ++ index) {

            if (!graph.getPose(
                        static_cast<uint32_t>(index),
                        optimized_poses[index]) ||
                !finite_pose(optimized_poses[index])) {

                return false;
            }
        }
        return true;
    }

    // 将body系点云通过优化好的位姿变到odom系重建局部跟踪iVox地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        LocalMapFeedbackNode::build_tracking_cloud(
            const BuildJob &job,
            const std::vector<Eigen::Isometry3d> &optimized_poses
    ) const {

        std::unordered_map<VoxelKey, pcl::PointXYZ, VoxelKeyHash> voxels;
        voxels.reserve(static_cast<size_t>(map_max_points_));
        const double inverse_leaf = 1.0 / map_voxel_leaf_size_m_;

        const auto insert_frame =
            [&](
                const FrameSnapshot &frame,
                const Eigen::Isometry3d &pose
            ) {
                if (!frame.cloud_body || !finite_pose(pose))
                    return;
                for (const auto &point : frame.cloud_body->points) {

                    const Eigen::Vector3d point_odom =
                            pose * Eigen::Vector3d(point.x, point.y, point.z);
                    if (!point_odom.allFinite())
                        continue;
                    const VoxelKey key{
                            static_cast<int64_t>(
                                    std::floor(point_odom.x() * inverse_leaf)),
                            static_cast<int64_t>(
                                    std::floor(point_odom.y() * inverse_leaf)),
                            static_cast<int64_t>(
                                    std::floor(point_odom.z() * inverse_leaf))};
                    if (voxels.find(key) != voxels.end())
                        continue;
                    if (voxels.size() >= static_cast<size_t>(map_max_points_))
                        return;
                    voxels.emplace(
                        key,
                        pcl::PointXYZ(
                            static_cast<float>(point_odom.x()),
                            static_cast<float>(point_odom.y()),
                            static_cast<float>(point_odom.z()))
                    );
                }
            };

        // Frozen spatial history is inserted first, so an overlapping active
        // voxel cannot erase the old surface that supplied relocalization.
        for (const auto &frame : job.frozen_history)
            insert_frame(frame, frame.initial_pose);
        for (size_t index = 0;
             index < job.active_frames.size() &&
             index < optimized_poses.size();
             ++ index) {

            insert_frame(job.active_frames[index], optimized_poses[index]);
        }

        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud->reserve(voxels.size());
        for (const auto &[key, point] : voxels) {

            static_cast<void>(key);
            cloud->push_back(point);
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = true;
        return cloud;
    }

    Eigen::Isometry3d LocalMapFeedbackNode::pose_to_isometry(
            const geometry_msgs::msg::Pose &pose
    ) {

        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        Eigen::Quaterniond quaternion(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z);
        if (quaternion.norm() > 1e-9 && quaternion.coeffs().allFinite()) {

            quaternion.normalize();
            result.linear() = quaternion.toRotationMatrix();
        } else {

            result.matrix().setConstant(
                    std::numeric_limits<double>::quiet_NaN());
            return result;
        }
        result.translation() = Eigen::Vector3d(
                pose.position.x, pose.position.y, pose.position.z);
        return result;
    }

    geometry_msgs::msg::Pose LocalMapFeedbackNode::isometry_to_pose(
            const Eigen::Isometry3d &pose
    ) {

        geometry_msgs::msg::Pose result;
        result.position.x = pose.translation().x();
        result.position.y = pose.translation().y();
        result.position.z = pose.translation().z();
        const Eigen::Quaterniond quaternion(pose.rotation());
        result.orientation.w = quaternion.w();
        result.orientation.x = quaternion.x();
        result.orientation.y = quaternion.y();
        result.orientation.z = quaternion.z();
        return result;
    }

    double LocalMapFeedbackNode::stamp_to_seconds(
            const builtin_interfaces::msg::Time &stamp
    ) {

        return static_cast<double>(stamp.sec) +
               static_cast<double>(stamp.nanosec) * 1e-9;
    }

    double LocalMapFeedbackNode::rotation_angle(
            const Eigen::Matrix3d &rotation
    ) {

        return std::acos(std::clamp(
                (rotation.trace() - 1.0) * 0.5, -1.0, 1.0));
    }

    bool LocalMapFeedbackNode::finite_pose(
            const Eigen::Isometry3d &pose
    ) {

        return pose.matrix().allFinite();
    }

    Eigen::Isometry3d LocalMapFeedbackNode::interpolate_correction(
            const Eigen::Isometry3d &from,
            const Eigen::Isometry3d &to,
            const double alpha
    ) {

        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.translation() =
                (1.0 - alpha) * from.translation() + alpha * to.translation();
        Eigen::Quaterniond from_rotation(from.rotation());
        Eigen::Quaterniond to_rotation(to.rotation());
        from_rotation.normalize();
        to_rotation.normalize();
        result.linear() = from_rotation.slerp(alpha, to_rotation)
                                  .normalized()
                                  .toRotationMatrix();
        return result;
    }

}  // namespace small_point_lio_pgo
