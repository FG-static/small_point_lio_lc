#include "small_point_lio_pgo/transform_utils.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <vector>

namespace small_point_lio_pgo {
namespace {

TEST(TransformUtils, BodyLidarRpyMatchesExampleCalibration) {
    const BodyLidarTransformResult result = makeBodyLidarTransform(
            {-0.011, -0.02329, 0.04412},
            {-0.261, -0.113, 0.0},
            {1.0, 0.0, 0.0, 0.0},
            {});

    EXPECT_EQ(result.rotation_source, RotationParameterSource::Rpy);
    EXPECT_TRUE(result.translation_valid);
    EXPECT_TRUE(result.rpy_valid);

    const Eigen::Matrix3d &rotation = result.transform.rotation();
    EXPECT_NEAR(rotation(0, 0), 0.9936, 1e-3);
    EXPECT_NEAR(rotation(0, 1), 0.0291, 1e-3);
    EXPECT_NEAR(rotation(0, 2), -0.1092, 1e-3);
    EXPECT_NEAR(rotation(1, 1), 0.9662, 1e-3);
    EXPECT_NEAR(rotation(1, 2), 0.2578, 1e-3);
    EXPECT_NEAR(rotation(2, 0), 0.1130, 1e-3);
    EXPECT_NEAR(rotation(2, 1), -0.2561, 1e-3);
    EXPECT_NEAR(rotation(2, 2), 0.9600, 1e-3);

    const Eigen::Vector3d lidar_point(1.2, -0.4, 0.8);
    const Eigen::Vector3d body_point = result.transform * lidar_point;
    EXPECT_TRUE(
            (result.transform.inverse() * body_point)
                    .isApprox(lidar_point, 1e-12));
}

TEST(TransformUtils, MatrixKeepsCompatibilityPrecedence) {
    const std::vector<double> matrix = {
        1.0, 0.0, 0.0, 4.0,
        0.0, 1.0, 0.0, 5.0,
        0.0, 0.0, 1.0, 6.0,
        0.0, 0.0, 0.0, 1.0,
    };
    const BodyLidarTransformResult result = makeBodyLidarTransform(
            {1.0, 2.0, 3.0},
            {0.1, 0.2, 0.3},
            {1.0, 0.0, 0.0, 0.0},
            matrix);

    EXPECT_EQ(result.rotation_source, RotationParameterSource::Matrix);
    EXPECT_TRUE(result.matrix_valid);
    EXPECT_TRUE(result.transform.translation().isApprox(
            Eigen::Vector3d(4.0, 5.0, 6.0), 1e-12));
}

TEST(TransformUtils, CommonBasisPoseAndPointRoundTrip) {
    Eigen::Isometry3d odom_from_body = Eigen::Isometry3d::Identity();
    odom_from_body.linear() = Eigen::AngleAxisd(
            0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    odom_from_body.translation() = Eigen::Vector3d(2.0, -1.0, 0.3);

    const Eigen::Vector3d point_body(0.4, -0.2, 1.1);
    const Eigen::Vector3d point_odom = odom_from_body * point_body;
    EXPECT_TRUE((odom_from_body.inverse() * point_odom)
                        .isApprox(point_body, 1e-12));
}

TEST(TransformUtils, IdentityBackendAlignmentDoesNotRotateAgain) {
    Eigen::Isometry3d raw_pose = Eigen::Isometry3d::Identity();
    raw_pose.linear() = (
            Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitY()))
                                .toRotationMatrix();
    raw_pose.translation() = Eigen::Vector3d(3.0, 2.0, 0.5);

    const Eigen::Isometry3d aligned = applyWorldAlignment(
            Eigen::Isometry3d::Identity(), raw_pose);
    EXPECT_TRUE(aligned.matrix().isApprox(raw_pose.matrix(), 1e-12));
}

}  // namespace
}  // namespace small_point_lio_pgo
