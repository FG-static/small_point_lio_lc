#include "small_point_lio/small_point_lio.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace small_point_lio {

    class SmallPointLioTestPeer {
    public:
        static void enable_feedback_without_worker(SmallPointLio &lio) {
            lio.parameters.local_map_feedback_enable = true;
            lio.parameters.local_map_max_apply_lag_translation_m = 0.10;
            lio.parameters.local_map_max_apply_lag_rotation_deg = 1.0;
        }

        static void set_estimator_state(
                SmallPointLio &lio,
                const Eigen::Vector3d &position,
                const Eigen::Vector3d &velocity) {
            lio.estimator.kf.x.position = position;
            lio.estimator.kf.x.velocity = velocity;
            lio.estimator.kf.x.rotation = Eigen::Matrix3d::Identity();
        }

        static state estimator_state(const SmallPointLio &lio) {
            return lio.estimator.kf.x;
        }

        static Eigen::Matrix<double, state::DIM, state::DIM> covariance(
                const SmallPointLio &lio) {
            return lio.estimator.kf.P;
        }

        static void add_correction_snapshot(
                SmallPointLio &lio,
                const uint64_t sequence,
                const Eigen::Isometry3d &correction) {
            lio.correction_history.push_back(
                    SmallPointLio::CorrectionSnapshot{sequence, correction});
        }

        static void add_tail_point(
                SmallPointLio &lio,
                const double stamp,
                const Eigen::Vector3f &point) {
            lio.tail_point_journal.push_back(
                    SmallPointLio::JournalPoint{
                            ++lio.point_insertion_sequence, stamp, point});
        }

        static void stage_built_map(
                SmallPointLio &lio,
                const uint64_t source_version,
                const uint64_t target_version,
                const uint64_t correction_sequence,
                const double cutoff_stamp,
                const std::vector<Eigen::Vector3f> &base_points) {
            SmallPointLio::BuiltMapUpdate built;
            built.metadata.cutoff_timestamp = cutoff_stamp;
            built.metadata.source_correction_sequence = correction_sequence;
            built.metadata.source_tracking_map_version = source_version;
            built.metadata.target_tracking_map_version = target_version;
            built.ivox = std::make_shared<SmallIVox>(0.5F, 1000);
            built.ivox->add_points(base_points);
            built.reset_generation =
                    lio.reset_generation.load(std::memory_order_acquire);
            std::lock_guard<std::mutex> lock(lio.map_builder_mutex);
            lio.built_map_update = std::move(built);
        }

        static void apply_built_map(SmallPointLio &lio) {
            lio.try_apply_built_map();
        }

        static size_t active_voxels(const SmallPointLio &lio) {
            return lio.estimator.ivox->size();
        }

        static bool has_pending_built_map(const SmallPointLio &lio) {
            std::lock_guard<std::mutex> lock(
                    const_cast<SmallPointLio &>(lio).map_builder_mutex);
            return lio.built_map_update.has_value();
        }

        static common::ScanToMapCorrection finalize_packet_with_predictions(
                SmallPointLio &lio,
                const state &epoch_prediction,
                const state &packet_prediction,
                const double stamp) {
            lio.prediction_epoch_valid = true;
            lio.prediction_only_kf.x = epoch_prediction;
            lio.packet_prediction_kf.x = packet_prediction;
            common::ScanToMapCorrection emitted;
            lio.set_scan_to_map_correction_callback(
                    [&emitted](const common::ScanToMapCorrection &correction) {
                        emitted = correction;
                    });
            lio.finalize_packet(stamp);
            lio.set_scan_to_map_correction_callback({});
            return emitted;
        }

        static state packet_prediction_state(const SmallPointLio &lio) {
            return lio.packet_prediction_kf.x;
        }

        static Eigen::Matrix<double, state::DIM, state::DIM>
        packet_prediction_covariance(const SmallPointLio &lio) {
            return lio.packet_prediction_kf.P;
        }
    };

    class LocalTrackingMapTest : public ::testing::Test {
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

        static rclcpp::NodeOptions node_options() {
            rclcpp::NodeOptions options;
            options.parameter_overrides({
                    rclcpp::Parameter("point_filter_num", int64_t{1}),
                    rclcpp::Parameter("min_distance", 0.1),
                    rclcpp::Parameter("max_distance", 100.0),
                    rclcpp::Parameter("space_downsample", false),
                    rclcpp::Parameter("space_downsample_leaf_size", 0.5),
                    rclcpp::Parameter(
                            "gravity", std::vector<double>{0.0, 0.0, -9.81}),
                    rclcpp::Parameter("check_satu", false),
                    rclcpp::Parameter("fix_gravity_direction", false),
                    rclcpp::Parameter("satu_acc", 3.0),
                    rclcpp::Parameter("satu_gyro", 35.0),
                    rclcpp::Parameter("acc_norm", 1.0),
                    rclcpp::Parameter("map_resolution", 0.5),
                    rclcpp::Parameter("init_map_size", int64_t{1}),
                    rclcpp::Parameter("extrinsic_est_en", false),
                    rclcpp::Parameter(
                            "extrinsic_T", std::vector<double>{0.0, 0.0, 0.0}),
                    rclcpp::Parameter(
                            "extrinsic_R",
                            std::vector<double>{
                                    1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 1.0}),
                    rclcpp::Parameter("laser_point_cov", 0.01),
                    rclcpp::Parameter("imu_meas_acc_cov", 0.01),
                    rclcpp::Parameter("imu_meas_omg_cov", 0.01),
                    rclcpp::Parameter("velocity_cov", 20.0),
                    rclcpp::Parameter("acceleration_cov", 500.0),
                    rclcpp::Parameter("omg_cov", 1000.0),
                    rclcpp::Parameter("ba_cov", 0.0001),
                    rclcpp::Parameter("bg_cov", 0.0001),
                    rclcpp::Parameter("plane_threshold", 0.1),
                    rclcpp::Parameter("match_sqaured", 81.0),
                    rclcpp::Parameter(
                            "publish_odometry_without_downsample", false),
                    // Keep the production builder thread off; the test peer
                    // stages its completed result deterministically.
                    rclcpp::Parameter("local_map_feedback_enable", false)});
            return options;
        }
    };

    TEST_F(LocalTrackingMapTest, VersionedSwapReplaysTailWithoutChangingEskf) {
        auto node = std::make_shared<rclcpp::Node>(
                "local_tracking_map_test", node_options());
        SmallPointLio lio(*node);
        SmallPointLioTestPeer::enable_feedback_without_worker(lio);
        SmallPointLioTestPeer::set_estimator_state(
                lio, Eigen::Vector3d(1.0, 2.0, 3.0),
                Eigen::Vector3d(0.4, 0.5, 0.6));
        const state before_state = SmallPointLioTestPeer::estimator_state(lio);
        const auto before_covariance = SmallPointLioTestPeer::covariance(lio);

        SmallPointLioTestPeer::add_correction_snapshot(
                lio, 7, Eigen::Isometry3d::Identity());
        SmallPointLioTestPeer::add_tail_point(
                lio, 1.1, Eigen::Vector3f(5.0F, 0.0F, 0.0F));
        SmallPointLioTestPeer::stage_built_map(
                lio,
                0,
                1,
                7,
                1.0,
                {Eigen::Vector3f(0.0F, 0.0F, 0.0F)});

        SmallPointLioTestPeer::apply_built_map(lio);

        EXPECT_EQ(lio.get_tracking_map_version(), 1U);
        EXPECT_EQ(SmallPointLioTestPeer::active_voxels(lio), 2U);
        const state after_state = SmallPointLioTestPeer::estimator_state(lio);
        EXPECT_TRUE(after_state.position.isApprox(before_state.position));
        EXPECT_TRUE(after_state.rotation.isApprox(before_state.rotation));
        EXPECT_TRUE(after_state.velocity.isApprox(before_state.velocity));
        EXPECT_TRUE(after_state.bg.isApprox(before_state.bg));
        EXPECT_TRUE(after_state.ba.isApprox(before_state.ba));
        EXPECT_TRUE(SmallPointLioTestPeer::covariance(lio).isApprox(
                before_covariance));
    }

    TEST_F(LocalTrackingMapTest, StaleSourceVersionCannotReplaceActiveMap) {
        auto node = std::make_shared<rclcpp::Node>(
                "stale_local_tracking_map_test", node_options());
        SmallPointLio lio(*node);
        SmallPointLioTestPeer::enable_feedback_without_worker(lio);
        SmallPointLioTestPeer::add_correction_snapshot(
                lio, 1, Eigen::Isometry3d::Identity());
        SmallPointLioTestPeer::stage_built_map(
                lio,
                0,
                1,
                1,
                0.0,
                {Eigen::Vector3f(0.0F, 0.0F, 0.0F)});
        SmallPointLioTestPeer::apply_built_map(lio);
        ASSERT_EQ(lio.get_tracking_map_version(), 1U);
        const size_t active_voxels =
                SmallPointLioTestPeer::active_voxels(lio);

        SmallPointLioTestPeer::stage_built_map(
                lio,
                0,
                1,
                1,
                0.0,
                {Eigen::Vector3f(10.0F, 0.0F, 0.0F),
                 Eigen::Vector3f(20.0F, 0.0F, 0.0F)});
        SmallPointLioTestPeer::apply_built_map(lio);

        EXPECT_EQ(lio.get_tracking_map_version(), 1U);
        EXPECT_EQ(SmallPointLioTestPeer::active_voxels(lio), active_voxels);
        EXPECT_FALSE(SmallPointLioTestPeer::has_pending_built_map(lio));
    }

    TEST_F(LocalTrackingMapTest, PacketPredictionForkRetainsFullPriorState) {
        auto node = std::make_shared<rclcpp::Node>(
                "packet_prediction_fork_test", node_options());
        SmallPointLio lio(*node);

        SmallPointLioTestPeer::set_estimator_state(
                lio,
                Eigen::Vector3d(3.0, 0.0, 0.0),
                Eigen::Vector3d(4.0, 0.0, 0.0));
        lio.Q.setIdentity();
        state epoch_prediction = SmallPointLioTestPeer::estimator_state(lio);
        epoch_prediction.position = Eigen::Vector3d(0.0, 0.0, 0.0);
        epoch_prediction.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
        state packet_prediction = epoch_prediction;
        packet_prediction.position = Eigen::Vector3d(2.5, 0.0, 0.0);
        packet_prediction.velocity = Eigen::Vector3d(3.5, 0.0, 0.0);

        const auto before_state = SmallPointLioTestPeer::estimator_state(lio);
        const auto before_covariance = SmallPointLioTestPeer::covariance(lio);
        const common::ScanToMapCorrection correction =
                SmallPointLioTestPeer::finalize_packet_with_predictions(
                        lio, epoch_prediction, packet_prediction, 1.0);

        EXPECT_EQ(correction.sequence, 1U);
        EXPECT_NEAR(correction.packet_predicted_pose.position.x(), 2.5, 1e-9);
        EXPECT_NEAR(correction.epoch_predicted_pose.position.x(), 0.0, 1e-9);
        EXPECT_NEAR(correction.corrected_pose.position.x(), 3.0, 1e-9);

        // The next packet shadow must start from the whole corrected filter,
        // not just from a pose-only SE(3) correction.
        const state refreshed =
                SmallPointLioTestPeer::packet_prediction_state(lio);
        EXPECT_TRUE(refreshed.position.isApprox(before_state.position));
        EXPECT_TRUE(refreshed.velocity.isApprox(before_state.velocity));
        EXPECT_TRUE(refreshed.bg.isApprox(before_state.bg));
        EXPECT_TRUE(refreshed.ba.isApprox(before_state.ba));
        EXPECT_TRUE(
                SmallPointLioTestPeer::packet_prediction_covariance(lio)
                        .isApprox(before_covariance));
    }

}  // namespace small_point_lio
