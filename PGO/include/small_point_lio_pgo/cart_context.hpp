#pragma once

#include <Eigen/Dense>

#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace small_point_lio_pgo {

    class CartContext {

    public:

        struct Params {

            double x_unit_m = 5.0;
            double y_unit_m = 2.0;
            double x_max_m = 100.0;
            double y_max_m = 40.0;
            double voxel_leaf_m = 0.5;
            double height_offset_m = 0.0;
            bool use_align_key = false;
            double align_search_ratio = 1.0;
        };

        struct Descriptor {

            Eigen::MatrixXf image;
            Eigen::VectorXf retrieval_key; // x轴快速定位
            Eigen::VectorXf align_key; // y轴快速定位
        };

        struct MatchResult {

            bool valid = false;
            float distance = std::numeric_limits<float>::infinity();
            int shift_cols = 0;
            double lateral_offset_m = 0.0;
            int engaged_columns = 0;
        };

        CartContext();

        explicit CartContext(
            const Params &params
        );

        const Params &params() const;

        int numX() const;

        int numY() const;

        Descriptor makeDescriptor(
            const pcl::PointCloud<pcl::PointXYZ> &cloud
        ) const;

        Descriptor flipped180(
            const Descriptor &descriptor
        ) const;

        MatchResult compare(
            const Descriptor &current,
            const Descriptor &history
        ) const;

    private:

        Params params_;
        int num_x_ = 0;
        int num_y_ = 0;
    };

} // namespace small_point_lio_pgo
