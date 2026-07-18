#include "small_point_lio_pgo/gicp.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

namespace small_point_lio_pgo {

    GicpMatcher::GicpMatcher() {

        configure(GicpParams{});
    }

    void GicpMatcher::configure(const GicpParams &params) {

        const int max_iterations = std::max(1, params.max_iterations);
        const double rotation_epsilon =
            std::isfinite(params.rotation_epsilon) &&
            params.rotation_epsilon > 0.0
            ? params.rotation_epsilon
            : 2e-3;
        const double transformation_epsilon =
            std::isfinite(params.transformation_epsilon) &&
            params.transformation_epsilon > 0.0
            ? params.transformation_epsilon
            : 5e-4;
        max_correspondence_distance_ =
            std::isfinite(params.max_correspondence_distance) &&
            params.max_correspondence_distance > 0.0
            ? params.max_correspondence_distance
            : 1.0;

        reg_.setRegistrationType("GICP");
        reg_.setNumThreads(params.num_threads);
        reg_.setCorrespondenceRandomness(params.correspondence_randomness);
        reg_.setMaxCorrespondenceDistance(max_correspondence_distance_);
        reg_.setMaximumIterations(max_iterations);
        reg_.setRotationEpsilon(rotation_epsilon);
        reg_.setTransformationEpsilon(transformation_epsilon);
    }

    GicpResult GicpMatcher::align(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
        const Eigen::Matrix4d &init_guess
    ) {

        GicpResult result;

        if (!source_cloud || source_cloud->empty() ||
            !target_cloud || target_cloud->empty()) {

            result.error_message = "empty source or target cloud";
            return result;
        }

        reg_.setInputSource(source_cloud);
        reg_.setInputTarget(target_cloud);

        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        try {

            reg_.align(*aligned, init_guess.cast<float>());
        } catch (const std::exception &e) {

            result.error_message = e.what();
            return result;
        }

        result.raw_score = reg_.getFitnessScore();
        // PCL nearest-neighbor distances are squared, so max_range must also
        // be squared. This keeps non-overlapping submap points out of the gate.
        const double max_correspondence_distance_sq =
            max_correspondence_distance_ * max_correspondence_distance_;
        result.score = reg_.getFitnessScore(max_correspondence_distance_sq);
        result.transform = reg_.getFinalTransformation().cast<double>();
        // 旧代码只读取 score 和 transform。small_gicp 本身还提供真实收敛状态、
        // 有效对应点和最终优化误差，这些才足以判断结果能否作为 loop edge。
        const auto &registration_result = reg_.getRegistrationResult();
        result.converged = reg_.hasConverged() && registration_result.converged;
        // small_gicp stores the zero-based final iteration index.
        result.iterations = registration_result.iterations + 1U;
        result.num_inliers = registration_result.num_inliers;
        result.inlier_ratio = static_cast<double>(result.num_inliers) /
            static_cast<double>(source_cloud->size());
        result.registration_error = registration_result.error;
        result.hessian = registration_result.H;
        if (!result.transform.allFinite()) {

            result.score = 1e9;
            result.raw_score = 1e9;
            result.transform = Eigen::Matrix4d::Identity();
            result.error_message = "non-finite transformation";
            return result;
        }
        if (!result.converged)
            result.error_message = "registration did not converge";

        // GicpMatcher 同时服务前端里程计，因此 success 保留原来的兼容语义；
        // 后端 loop verification 会单独强制 converged=true 和质量 gate。
        result.success = true;
        return result;
    }

} // namespace small_point_lio_pgo
