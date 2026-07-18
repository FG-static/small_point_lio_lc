#include "small_point_lio_pgo/cart_context.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <pcl/filters/voxel_grid.h>

namespace small_point_lio_pgo {

    namespace {

        int computeBinCount(
            const double range_m,
            const double unit_m
        ) {

            if (!std::isfinite(range_m) || !std::isfinite(unit_m) ||
                range_m <= 0.0 || unit_m <= 0.0)
                return 0;

            return std::max(1, static_cast<int>(std::round(2.0 * range_m / unit_m)));
        }

        int sgn(
            const float value
        ) {

            if (value > 0.0F)
                return 1;
            if (value < 0.0F)
                return -1;
            return 0;
        }

        /**
         * @brief 将连续的坐标值映射到网格的离散 bin 索引，车体位置对应网格中心
         * @param value 连续坐标值
         * @param unit_m 单元网格长度
         * @param bin_count bin 数量
         * @return int 正值索引（bin_count / 2 表示中心）
         */
        int centeredBinIndex(
            const float value,
            const double unit_m,
            const int bin_count
        ) {

            const int offset =
                sgn(value) *
                static_cast<int>(std::floor(std::abs(value) / unit_m));
            return offset + bin_count / 2;
        }

        // 支持负数的 mod 运算
        int positiveModulo(
            const int value,
            const int modulo
        ) {

            if (modulo <= 0)
                return 0;

            const int remainder = value % modulo;
            return remainder < 0 ? remainder + modulo : remainder;
        }

        // 将 column shift 转为带符号的 shift
        // 例如 column = 40, shift = 39 -> shift = -1
        int signedCircularShift(
            const int shift,
            const int columns
        ) {

            if (columns <= 0)
                return 0;

            const int wrapped = positiveModulo(shift, columns);
            if (wrapped > columns / 2)
                return wrapped - columns;
            return wrapped;
        }

        // 用 align key 快速估计循环对齐所需偏移
        int estimateShiftFromAlignKey(
            const Eigen::VectorXf &current_key,
            const Eigen::VectorXf &history_key
        ) {

            if (current_key.size() == 0 ||
                current_key.size() != history_key.size())
                return 0;

            const int columns = static_cast<int>(current_key.size());
            float best_rmse = std::numeric_limits<float>::infinity();
            int best_shift = 0;

            for (int shift = 0; shift < columns; ++shift) {

                float squared_error = 0.0F;
                for (int col = 0; col < columns; ++col) {

                    const int current_col =
                        positiveModulo(col - shift, columns);
                    const float diff =
                        current_key(current_col) - history_key(col);
                    squared_error += diff * diff;
                }

                const float rmse =
                    std::sqrt(squared_error / static_cast<float>(columns));
                if (rmse < best_rmse) {

                    best_rmse = rmse;
                    best_shift = shift;
                }
            }

            return best_shift;
        }

        // 生成需要测试的列偏移集合
        std::vector<int> makeShiftSearchSpace(
            const int columns,
            const bool use_align_key,
            const double align_search_ratio,
            const int init_shift
        ) {

            std::vector<int> shifts;
            if (columns <= 0)
                return shifts;

            if (!use_align_key ||
                !std::isfinite(align_search_ratio) ||
                align_search_ratio >= 1.0) {

                shifts.reserve(static_cast<size_t>(columns));
                for (int shift = 0; shift < columns; ++shift)
                    shifts.push_back(shift);
                return shifts;
            }

            const double clamped_ratio =
                std::max(0.0, std::min(1.0, align_search_ratio));
            const int radius = static_cast<int>(
                std::ceil(0.5 * clamped_ratio * columns)
            );
            std::vector<bool> seen(static_cast<size_t>(columns), false);

            const auto append_shift = [&](const int raw_shift) {

                const int shift = positiveModulo(raw_shift, columns);
                if (seen[static_cast<size_t>(shift)])
                    return;
                seen[static_cast<size_t>(shift)] = true;
                shifts.push_back(shift);
            };

            append_shift(init_shift);
            for (int delta = 1; delta <= radius; ++delta) {

                append_shift(init_shift - delta);
                append_shift(init_shift + delta);
            }

            return shifts;
        }

    } // namespace

    CartContext::CartContext() : CartContext(Params{}) {
    }

    CartContext::CartContext(
        const Params &params
    ) : params_(params),
        num_x_(computeBinCount(params_.x_max_m, params_.x_unit_m)),
        num_y_(computeBinCount(params_.y_max_m, params_.y_unit_m)) {
    }

    const CartContext::Params &CartContext::params() const {

        return params_;
    }

    int CartContext::numX() const {

        return num_x_;
    }

    int CartContext::numY() const {

        return num_y_;
    }

    CartContext::Descriptor CartContext::makeDescriptor(
        const pcl::PointCloud<pcl::PointXYZ> &cloud
    ) const {

        Descriptor descriptor;
        descriptor.image = Eigen::MatrixXf::Zero(num_x_, num_y_);
        descriptor.retrieval_key = Eigen::VectorXf::Zero(num_x_);
        descriptor.align_key = Eigen::VectorXf::Zero(num_y_);

        if (num_x_ <= 0 || num_y_ <= 0 || cloud.empty())
            return descriptor;
        if (!std::isfinite(params_.x_unit_m) || params_.x_unit_m <= 0.0 ||
            !std::isfinite(params_.y_unit_m) || params_.y_unit_m <= 0.0 ||
            !std::isfinite(params_.x_max_m) || params_.x_max_m <= 0.0 ||
            !std::isfinite(params_.y_max_m) || params_.y_max_m <= 0.0)
            return descriptor;

        auto working_cloud =
            std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        if (std::isfinite(params_.voxel_leaf_m) &&
            params_.voxel_leaf_m > 0.0) {

            auto input_cloud =
                std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud);
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setLeafSize(
                static_cast<float>(params_.voxel_leaf_m),
                static_cast<float>(params_.voxel_leaf_m),
                static_cast<float>(params_.voxel_leaf_m)
            );
            voxel_filter.setInputCloud(input_cloud);
            voxel_filter.filter(*working_cloud);
        } else *working_cloud = cloud;

        for (const auto &point : working_cloud->points) {

            if (!std::isfinite(point.x) ||
                !std::isfinite(point.y) ||
                !std::isfinite(point.z))
                continue;

            if (point.x == 0.0F || point.y == 0.0F)
                continue;

            if (point.x <= -params_.x_max_m ||
                point.x >= params_.x_max_m ||
                point.y <= -params_.y_max_m ||
                point.y >= params_.y_max_m)
                continue;

            const int x_index =
                centeredBinIndex(point.x, params_.x_unit_m, num_x_);
            const int y_index =
                centeredBinIndex(point.y, params_.y_unit_m, num_y_);

            if (x_index < 0 || x_index >= num_x_ ||
                y_index < 0 || y_index >= num_y_)
                continue;

            const float z_value =
                point.z + static_cast<float>(params_.height_offset_m);
            descriptor.image(x_index, y_index) =
                std::max(descriptor.image(x_index, y_index), z_value);
        }

        for (int x = 0; x < num_x_; ++ x)
            descriptor.retrieval_key(x) = descriptor.image.row(x).mean();

        for (int y = 0; y < num_y_; ++ y)
            descriptor.align_key(y) = descriptor.image.col(y).mean();

        return descriptor;
    }

    CartContext::Descriptor CartContext::flipped180(
        const Descriptor &descriptor
    ) const {

        Descriptor flipped;
        if (num_x_ <= 0 || num_y_ <= 0)
            return flipped;
        if (descriptor.image.rows() != num_x_ ||
            descriptor.image.cols() != num_y_)
            return flipped;

        flipped.image = Eigen::MatrixXf::Zero(num_x_, num_y_);
        flipped.retrieval_key = Eigen::VectorXf::Zero(num_x_);
        flipped.align_key = Eigen::VectorXf::Zero(num_y_);

        for (int x = 0; x < num_x_; ++ x) {

            for (int y = 0; y < num_y_; ++ y) {

                flipped.image(x, y) =
                    descriptor.image(num_x_ - 1 - x, num_y_ - 1 - y);
            }
        }

        for (int x = 0; x < num_x_; ++ x)
            flipped.retrieval_key(x) = flipped.image.row(x).mean();

        for (int y = 0; y < num_y_; ++ y)
            flipped.align_key(y) = flipped.image.col(y).mean();

        return flipped;
    }

    CartContext::MatchResult CartContext::compare(
        const Descriptor &current,
        const Descriptor &history
    ) const {

        MatchResult result;
        if (num_x_ <= 0 || num_y_ <= 0)
            return result;

        if (current.image.rows() != num_x_ ||
            current.image.cols() != num_y_ ||
            history.image.rows() != num_x_ ||
            history.image.cols() != num_y_)
            return result;

        const int init_shift = params_.use_align_key
            ? estimateShiftFromAlignKey(current.align_key, history.align_key)
            : 0;
        const std::vector<int> shifts = makeShiftSearchSpace(
            num_y_,
            params_.use_align_key,
            params_.align_search_ratio,
            init_shift
        );

        double best_similarity = -std::numeric_limits<double>::infinity();
        int best_shift = 0;
        int best_engaged_columns = 0;
        constexpr float kMinColumnNorm = 1e-6F;

        for (const int shift : shifts) {

            double sum_cosine_similarity = 0.0;
            int engaged_columns = 0;

            for (int col = 0; col < num_y_; ++ col) {

                const int current_col =
                    positiveModulo(col - shift, num_y_);
                const auto current_column =
                    current.image.col(current_col);
                const auto history_column =
                    history.image.col(col);

                const float current_norm = current_column.norm();
                const float history_norm = history_column.norm();
                if (current_norm <= kMinColumnNorm ||
                    history_norm <= kMinColumnNorm)
                    continue;

                const float cosine_similarity =
                    current_column.dot(history_column) /
                    (current_norm * history_norm);
                if (!std::isfinite(cosine_similarity))
                    continue;

                sum_cosine_similarity += cosine_similarity;
                ++ engaged_columns;
            }

            if (engaged_columns <= 0)
                continue;

            const double mean_similarity =
                sum_cosine_similarity /
                static_cast<double>(engaged_columns);
            if (mean_similarity > best_similarity) {

                best_similarity = mean_similarity;
                best_shift = shift;
                best_engaged_columns = engaged_columns;
            }
        }

        if (!std::isfinite(best_similarity) || best_engaged_columns <= 0)
            return result;

        const int signed_shift = signedCircularShift(best_shift, num_y_);
        result.valid = true;
        result.distance = static_cast<float>(1.0 - best_similarity);
        result.shift_cols = signed_shift;
        result.lateral_offset_m =
            static_cast<double>(signed_shift) * params_.y_unit_m;
        result.engaged_columns = best_engaged_columns;
        return result;
    }

} // namespace small_point_lio_pgo
