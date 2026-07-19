#include "small_point_lio_pgo/local_map_feedback_node.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace small_point_lio_pgo {

    namespace {

        Eigen::Isometry3d translation(const double x) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation().x() = x;
            return pose;
        }

    }  // namespace

    class LocalMapFeedbackTestPeer {
    public:
        /// Bundles the local-window optimizer status and output poses so tests
        /// can assert both convergence and correction distribution.
        struct WindowResult {
            bool success = false;///< Whether optimize_active_window succeeded.
            std::vector<Eigen::Isometry3d> poses;///< Optimized active-frame poses.
        };

        static void stop_worker(LocalMapFeedbackNode &node) {
            {
                std::lock_guard<std::mutex> lock(node.worker_mutex_);
                node.stop_worker_ = true;
                node.queued_job_.reset();
            }
            node.worker_cv_.notify_all();
            if (node.worker_thread_.joinable()) {
                node.worker_thread_.join();
            }
        }

        static void configure_quality_gate(
                LocalMapFeedbackNode &node,
                const int min_packets = 3) {
            node.evidence_window_packets_ = 30;
            node.evidence_window_duration_sec_ = 10.0;
            node.evidence_min_packets_ = min_packets;
            node.evidence_min_valid_updates_ = 100;
            node.evidence_min_duration_sec_ = 0.15;
            node.evidence_min_valid_ratio_ = 0.5;
            node.cumulative_trigger_translation_m_ = 0.15;
            node.cumulative_trigger_rotation_deg_ = 180.0;
            node.instant_trigger_translation_m_ = 10.0;
            node.instant_trigger_rotation_deg_ = 180.0;
            node.instant_trigger_min_valid_updates_ = 50;
            node.optimization_cooldown_sec_ = 0.0;
            node.min_active_keyframes_ = 3;
            node.active_window_keyframes_ = 20;
        }

        static void add_candidate(
                LocalMapFeedbackNode &node,
                const uint32_t id,
                const double stamp,
                const Eigen::Isometry3d &raw_pose,
                const Eigen::Isometry3d &epoch_prediction,
                const uint64_t map_version = 0) {
            LocalMapFeedbackNode::CandidateRecord candidate;
            candidate.id = id;
            candidate.stamp = stamp;
            candidate.raw_pose = raw_pose;
            candidate.cloud_body =
                    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            candidate.cloud_body->push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
            candidate.has_epoch_prediction = true;
            candidate.prediction_map_version = map_version;
            candidate.epoch_prediction = epoch_prediction;
            node.candidates_.push_back(std::move(candidate));
        }

        static void configure_instant_trigger(LocalMapFeedbackNode &node) {
            node.instant_trigger_translation_m_ = 0.25;
            node.instant_trigger_rotation_deg_ = 180.0;
            node.instant_trigger_min_valid_updates_ = 50;
        }

        static void configure_spatial_window(
                LocalMapFeedbackNode &node,
                const int active_keyframes,
                const double history_radius_m) {
            node.active_window_keyframes_ = active_keyframes;
            node.min_active_keyframes_ = active_keyframes;
            node.history_search_radius_m_ = history_radius_m;
            node.history_max_keyframes_ = 20;
        }

        static void add_correction(
                LocalMapFeedbackNode &node,
                const double stamp,
                const Eigen::Isometry3d &packet_prediction,
                const Eigen::Isometry3d &epoch_prediction,
                const Eigen::Isometry3d &corrected,
                const uint32_t attempted = 50,
                const uint32_t accepted = 50,
                const uint64_t map_version = 0) {
            LocalMapFeedbackNode::CorrectionRecord record;
            record.stamp = stamp;
            record.sequence = node.corrections_.size() + 1;
            record.tracking_map_version = map_version;
            record.packet_predicted = packet_prediction;
            record.epoch_predicted = epoch_prediction;
            record.corrected = corrected;
            record.attempted_updates = attempted;
            record.accepted_updates = accepted;
            node.corrections_.push_back(record);
            node.has_correction_ = true;
            node.last_correction_stamp_ = stamp;
            node.active_tracking_map_version_ = map_version;
        }

        static bool schedule_latest(LocalMapFeedbackNode &node) {
            std::lock_guard<std::mutex> lock(node.state_mutex_);
            return node.schedule_if_needed_locked(node.corrections_.back());
        }

        static std::string queued_reason(LocalMapFeedbackNode &node) {
            std::lock_guard<std::mutex> lock(node.worker_mutex_);
            return node.queued_job_ ? node.queued_job_->trigger_reason : "";
        }

        static bool pgo_deformation_significant(
                LocalMapFeedbackNode &node,
                const std::unordered_map<uint32_t, Eigen::Isometry3d> &previous,
                const std::unordered_map<uint32_t, Eigen::Isometry3d> &current) {
            std::lock_guard<std::mutex> lock(node.state_mutex_);
            return node.pgo_deformation_significant_locked(previous, current);
        }

        static std::unordered_map<uint32_t, Eigen::Isometry3d> seed_poses(
                LocalMapFeedbackNode &node,
                const std::unordered_map<uint32_t, Eigen::Isometry3d>
                        &optimized) {
            std::lock_guard<std::mutex> lock(node.state_mutex_);
            return node.seed_poses_locked(0, optimized, 1);
        }

        static WindowResult optimize_window(
                LocalMapFeedbackNode &node,
                const std::vector<Eigen::Isometry3d> &initial,
                const std::vector<Eigen::Isometry3d> &epoch_predictions,
                const Eigen::Isometry3d &current_epoch_prediction,
                const Eigen::Isometry3d &current_corrected) {
            LocalMapFeedbackNode::BuildJob job;
            for (size_t index = 0; index < initial.size(); ++index) {
                LocalMapFeedbackNode::FrameSnapshot frame;
                frame.candidate_id = static_cast<uint32_t>(index);
                frame.stamp = static_cast<double>(index);
                frame.initial_pose = initial[index];
                frame.epoch_prediction = epoch_predictions[index];
                frame.cloud_body =
                        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                job.active_frames.push_back(std::move(frame));
            }
            job.correction.stamp = static_cast<double>(initial.size());
            job.correction.epoch_predicted = current_epoch_prediction;
            job.correction.corrected = current_corrected;

            WindowResult result;
            result.success = node.optimize_active_window(job, result.poses);
            return result;
        }

        static std::vector<uint32_t> frozen_history_ids(
                LocalMapFeedbackNode &node) {
            std::lock_guard<std::mutex> lock(node.state_mutex_);
            const auto job = node.make_build_job_locked(
                    node.corrections_.back(), "test");
            std::vector<uint32_t> ids;
            if (!job) {
                return ids;
            }
            ids.reserve(job->frozen_history.size());
            for (const auto &frame: job->frozen_history) {
                ids.push_back(frame.candidate_id);
            }
            return ids;
        }

        static void prepare_pending_commit(
                LocalMapFeedbackNode &node,
                const uint64_t source_correction_sequence) {
            LocalMapFeedbackNode::PendingPoseCommit commit;
            commit.source_correction_sequence = source_correction_sequence;
            commit.source_tracking_map_version = 0;
            commit.target_tracking_map_version = 1;
            commit.pgo_graph_version = 3;
            commit.poses.emplace(42U, translation(4.2));
            node.active_tracking_map_version_ = 0;
            node.awaiting_map_apply_ = true;
            node.awaiting_target_version_ = 1;
            node.pending_pose_commit_ = std::move(commit);
            node.corrections_.push_back(
                    LocalMapFeedbackNode::CorrectionRecord{});
        }

        static void acknowledge(
                LocalMapFeedbackNode &node,
                const uint64_t active_version,
                const uint64_t source_correction_sequence) {
            std::lock_guard<std::mutex> lock(node.state_mutex_);
            node.acknowledge_map_version_locked(
                    active_version, source_correction_sequence);
        }

        static size_t override_count(const LocalMapFeedbackNode &node) {
            return node.local_pose_overrides_.size();
        }

        static uint64_t applied_pgo_version(const LocalMapFeedbackNode &node) {
            return node.applied_pgo_graph_version_;
        }

        static void expire_pending_pgo_commit(LocalMapFeedbackNode &node) {
            LocalMapFeedbackNode::PendingPoseCommit commit;
            commit.source_correction_sequence = 1;
            commit.source_tracking_map_version = 0;
            commit.target_tracking_map_version = 1;
            commit.pgo_graph_version = 3;
            node.pgo_graph_version_ = 3;
            node.applied_pgo_graph_version_ = 2;
            node.pgo_rebuild_pending_ = false;
            node.awaiting_map_apply_ = true;
            node.awaiting_target_version_ = 1;
            node.awaiting_since_ = LocalMapFeedbackNode::SteadyClock::now() -
                                   std::chrono::seconds(10);
            node.map_apply_timeout_sec_ = 0.01;
            node.pending_pose_commit_ = std::move(commit);
        }
    };

    class LocalMapFeedbackNodeTest : public ::testing::Test {
    protected:
        static void SetUpTestSuite() {
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }
        }

        static void TearDownTestSuite() {
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            }
        }

        std::shared_ptr<LocalMapFeedbackNode> make_node() {
            auto node = std::make_shared<LocalMapFeedbackNode>();
            LocalMapFeedbackTestPeer::stop_worker(*node);
            return node;
        }

        static void add_three_candidates(LocalMapFeedbackNode &node) {
            LocalMapFeedbackTestPeer::add_candidate(
                    node, 0, 0.00, translation(0.0), translation(0.0));
            LocalMapFeedbackTestPeer::add_candidate(
                    node, 1, 0.10, translation(0.1), translation(0.1));
            LocalMapFeedbackTestPeer::add_candidate(
                    node, 2, 0.15, translation(0.15), translation(0.15));
        }
    };

    TEST_F(LocalMapFeedbackNodeTest, PacketCountAloneDoesNotTrigger) {
        auto node = make_node();
        LocalMapFeedbackTestPeer::configure_quality_gate(*node);
        add_three_candidates(*node);
        LocalMapFeedbackTestPeer::add_correction(
                *node, 0.00, translation(0.0), translation(0.0), translation(0.0));
        LocalMapFeedbackTestPeer::add_correction(
                *node, 0.10, translation(0.1), translation(0.1), translation(0.1));
        LocalMapFeedbackTestPeer::add_correction(
                *node, 0.20, translation(0.2), translation(0.2), translation(0.2));

        EXPECT_FALSE(LocalMapFeedbackTestPeer::schedule_latest(*node));
        EXPECT_TRUE(LocalMapFeedbackTestPeer::queued_reason(*node).empty());
    }

    TEST_F(LocalMapFeedbackNodeTest, CumulativeCorrectionTriggersAfterQualityGate) {
        auto node = make_node();
        LocalMapFeedbackTestPeer::configure_quality_gate(*node);
        add_three_candidates(*node);
        LocalMapFeedbackTestPeer::add_correction(
                *node, 0.00, translation(0.0), translation(0.0), translation(0.0));
        LocalMapFeedbackTestPeer::add_correction(
                *node, 0.10, translation(0.1), translation(0.1), translation(0.1));
        LocalMapFeedbackTestPeer::add_correction(
                *node, 0.20, translation(0.2), translation(0.2), translation(0.4));

        EXPECT_TRUE(LocalMapFeedbackTestPeer::schedule_latest(*node));
        EXPECT_EQ(
                LocalMapFeedbackTestPeer::queued_reason(*node),
                "cumulative_scan_correction");
    }

    TEST_F(LocalMapFeedbackNodeTest, StrongSinglePacketCorrectionTriggersImmediately) {
        auto node = make_node();
        LocalMapFeedbackTestPeer::configure_quality_gate(*node, 10);
        add_three_candidates(*node);
        LocalMapFeedbackTestPeer::add_correction(
                *node, 0.20, translation(0.0), translation(0.0), translation(0.30));
        LocalMapFeedbackTestPeer::configure_instant_trigger(*node);

        // Directly tune the production policy through the test peer's quality
        // setup: this packet clears the default instant threshold but cannot
        // clear the ten-packet normal evidence gate.
        EXPECT_TRUE(LocalMapFeedbackTestPeer::schedule_latest(*node));
        EXPECT_EQ(
                LocalMapFeedbackTestPeer::queued_reason(*node),
                "instant_scan_correction");
    }

    TEST_F(LocalMapFeedbackNodeTest, RigidPgoGaugeShiftIsNotLocalDeformation) {
        auto node = make_node();
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 0, 0.0, translation(0.0), translation(0.0));
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 1, 1.0, translation(1.0), translation(1.0));
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 2, 2.0, translation(2.0), translation(2.0));

        const std::unordered_map<uint32_t, Eigen::Isometry3d> previous{
                {0, translation(0.0)},
                {1, translation(1.0)},
                {2, translation(2.0)}};
        const std::unordered_map<uint32_t, Eigen::Isometry3d> rigid_shifted{
                {0, translation(10.0)},
                {1, translation(11.0)},
                {2, translation(12.0)}};

        EXPECT_FALSE(LocalMapFeedbackTestPeer::pgo_deformation_significant(
                *node, previous, rigid_shifted));

        auto deformed = rigid_shifted;
        deformed[1] = translation(11.20);
        EXPECT_TRUE(LocalMapFeedbackTestPeer::pgo_deformation_significant(
                *node, previous, deformed));
    }

    TEST_F(LocalMapFeedbackNodeTest, PgoSeedsRemoveGaugeButKeepLocalDeformation) {
        auto node = make_node();
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 0, 0.0, translation(0.0), translation(0.0));
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 1, 1.0, translation(1.0), translation(1.0));
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 2, 2.0, translation(2.0), translation(2.0));

        const std::unordered_map<uint32_t, Eigen::Isometry3d> optimized{
                {0, translation(10.0)},
                {1, translation(11.2)},
                {2, translation(12.0)}};
        const auto seeds =
                LocalMapFeedbackTestPeer::seed_poses(*node, optimized);

        ASSERT_EQ(seeds.size(), 3U);
        EXPECT_NEAR(seeds.at(0).translation().x(), 0.0, 1e-9);
        EXPECT_NEAR(seeds.at(1).translation().x(), 1.2, 1e-9);
        EXPECT_NEAR(seeds.at(2).translation().x(), 2.0, 1e-9);
    }

    TEST_F(LocalMapFeedbackNodeTest, FixedLagGraphDistributesTerminalCorrection) {
        auto node = make_node();
        const auto result = LocalMapFeedbackTestPeer::optimize_window(
                *node,
                {translation(0.0), translation(1.0), translation(2.0)},
                {translation(0.0), translation(1.0), translation(2.0)},
                translation(3.0),
                translation(3.6));

        ASSERT_TRUE(result.success);
        ASSERT_EQ(result.poses.size(), 3U);
        EXPECT_NEAR(result.poses.front().translation().x(), 0.0, 1e-6);
        EXPECT_GT(result.poses[1].translation().x(), 1.05);
        EXPECT_LT(result.poses[1].translation().x(), 1.35);
        EXPECT_GT(result.poses[2].translation().x(), 2.25);
        EXPECT_LT(result.poses[2].translation().x(), 2.55);
    }

    TEST_F(LocalMapFeedbackNodeTest, FrozenHistorySelectionIsSpatialNotOnlyTemporal) {
        auto node = make_node();
        LocalMapFeedbackTestPeer::configure_quality_gate(*node);
        LocalMapFeedbackTestPeer::configure_spatial_window(*node, 2, 2.0);
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 0, 0.0, translation(0.0), translation(0.0));
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 1, 0.1, translation(50.0), translation(50.0));
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 2, 1.0, translation(0.1), translation(0.1));
        LocalMapFeedbackTestPeer::add_candidate(
                *node, 3, 1.1, translation(0.2), translation(0.2));
        LocalMapFeedbackTestPeer::add_correction(
                *node, 1.2, translation(0.3), translation(0.3), translation(0.3));

        // Restrict the active temporal tail to IDs 2 and 3. The much older ID
        // 0 must still be selected because it is spatially nearby; ID 1 is not.
        const auto ids = LocalMapFeedbackTestPeer::frozen_history_ids(*node);
        EXPECT_NE(std::find(ids.begin(), ids.end(), 0U), ids.end());
        EXPECT_EQ(std::find(ids.begin(), ids.end(), 1U), ids.end());
    }

    TEST_F(LocalMapFeedbackNodeTest, MapAcknowledgementMatchesBuildSequence) {
        auto node = make_node();
        LocalMapFeedbackTestPeer::prepare_pending_commit(*node, 10);

        // An older timed-out build can have the same target version. It must
        // not commit the poses belonging to the newer pending build.
        LocalMapFeedbackTestPeer::acknowledge(*node, 1, 9);
        EXPECT_EQ(LocalMapFeedbackTestPeer::override_count(*node), 0U);
        EXPECT_EQ(LocalMapFeedbackTestPeer::applied_pgo_version(*node), 0U);

        LocalMapFeedbackTestPeer::prepare_pending_commit(*node, 10);
        LocalMapFeedbackTestPeer::acknowledge(*node, 1, 10);
        EXPECT_EQ(LocalMapFeedbackTestPeer::override_count(*node), 1U);
        EXPECT_EQ(LocalMapFeedbackTestPeer::applied_pgo_version(*node), 3U);
    }

    TEST_F(LocalMapFeedbackNodeTest, TimedOutPgoMapIsRearmed) {
        auto node = make_node();
        LocalMapFeedbackTestPeer::configure_quality_gate(*node);
        add_three_candidates(*node);
        LocalMapFeedbackTestPeer::add_correction(
                *node,
                0.20,
                translation(0.2),
                translation(0.2),
                translation(0.2));
        LocalMapFeedbackTestPeer::expire_pending_pgo_commit(*node);

        EXPECT_TRUE(LocalMapFeedbackTestPeer::schedule_latest(*node));
        EXPECT_EQ(LocalMapFeedbackTestPeer::queued_reason(*node), "pgo");
    }

}  // namespace small_point_lio_pgo
