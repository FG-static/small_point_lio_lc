#ifndef SMALL_POINT_LIO_PGO__GICP_HPP
#define SMALL_POINT_LIO_PGO__GICP_HPP

#include "small_gicp_compat.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <limits>
#include <string>

namespace small_point_lio_pgo {

    struct GicpParams {

        int num_threads = 4;
        int correspondence_randomness = 20;
        double max_correspondence_distance = 1.0;
        int max_iterations = 10;
        double rotation_epsilon = 2e-3;
        double transformation_epsilon = 5e-4;
    };

    struct GicpResult {

        // success 只表示调用完成且拿到了有限变换，保留给前端的旧语义；
        // loop edge 还必须额外检查 converged 和下面的配准质量。
        bool success = false;
        bool converged = false;
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        // 保存最终 Hessian、inlier 和 error；当前检查其有效性并输出日志，
        // Hessian 条件数可留给后续更细的退化检测。
        Eigen::Matrix<double, 6, 6> hessian =
            Eigen::Matrix<double, 6, 6>::Zero();
        // score only includes nearest-neighbor squared distances inside the
        // configured correspondence radius; raw_score includes every source point.
        double score = 1e9;
        double raw_score = 1e9;
        double inlier_ratio = 0.0;
        double registration_error = std::numeric_limits<double>::infinity();
        size_t iterations = 0;
        size_t num_inliers = 0;
        std::string error_message;
    };

    class GicpMatcher {

    public:

        GicpMatcher();

        void configure(const GicpParams &params);

        GicpResult align(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
            const Eigen::Matrix4d &init_guess
        );

    private:

        small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg_;
        double max_correspondence_distance_ = 1.0;
    };

} // namespace small_point_lio_pgo

#endif  // SMALL_POINT_LIO_PGO__GICP_HPP
