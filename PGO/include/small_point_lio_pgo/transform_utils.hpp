#ifndef SMALL_POINT_LIO_PGO__TRANSFORM_UTILS_HPP
#define SMALL_POINT_LIO_PGO__TRANSFORM_UTILS_HPP

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <vector>

namespace small_point_lio_pgo {

    enum class RotationParameterSource {
        Identity,
        Quaternion,
        Rpy,
        Matrix,
    };

    struct BodyLidarTransformResult {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        RotationParameterSource rotation_source =
                RotationParameterSource::Identity;
        bool translation_valid = false;
        bool quaternion_valid = false;
        bool rpy_provided = false;
        bool rpy_valid = false;
        bool matrix_provided = false;
        bool matrix_valid = false;
    };

    inline bool finiteVector(
            const std::vector<double> &values,
            const std::size_t expected_size) {
        if (values.size() != expected_size)
            return false;
        for (const double value : values) {
            if (!std::isfinite(value))
                return false;
        }
        return true;
    }

    /// 从兼容参数构造 base/body <- LiDAR 刚体变换。
    /// 旋转优先级为完整矩阵、RPY（弧度）、四元数。
    inline BodyLidarTransformResult makeBodyLidarTransform(
            const std::vector<double> &translation,
            const std::vector<double> &rpy,
            const std::vector<double> &quaternion,
            const std::vector<double> &matrix) {
        BodyLidarTransformResult result;
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

        result.translation_valid = finiteVector(translation, 3);
        if (result.translation_valid)
            t = Eigen::Vector3d(
                    translation[0], translation[1], translation[2]);

        result.quaternion_valid = finiteVector(quaternion, 4);
        if (result.quaternion_valid) {
            q = Eigen::Quaterniond(
                    quaternion[0], quaternion[1],
                    quaternion[2], quaternion[3]);
            result.quaternion_valid =
                    q.coeffs().allFinite() && q.norm() > 1e-9;
            if (result.quaternion_valid) {
                q.normalize();
                result.rotation_source =
                        RotationParameterSource::Quaternion;
            } else {
                q.setIdentity();
            }
        }

        result.rpy_provided = !rpy.empty();
        result.rpy_valid = finiteVector(rpy, 3);
        if (result.rpy_valid) {
            q = Eigen::Quaterniond(
                    Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));
            q.normalize();
            result.rotation_source = RotationParameterSource::Rpy;
        }

        result.matrix_provided = !matrix.empty();
        if (finiteVector(matrix, 16)) {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            for (int row = 0; row < 4; ++row) {
                for (int column = 0; column < 4; ++column) {
                    T(row, column) = matrix[
                            static_cast<std::size_t>(row * 4 + column)];
                }
            }
            Eigen::Quaterniond matrix_rotation(T.block<3, 3>(0, 0));
            result.matrix_valid =
                    T.block<3, 1>(0, 3).allFinite() &&
                    matrix_rotation.coeffs().allFinite() &&
                    matrix_rotation.norm() > 1e-9;
            if (result.matrix_valid) {
                matrix_rotation.normalize();
                result.transform.linear() =
                        matrix_rotation.toRotationMatrix();
                result.transform.translation() = T.block<3, 1>(0, 3);
                result.rotation_source = RotationParameterSource::Matrix;
                return result;
            }
        }

        result.transform.linear() = q.toRotationMatrix();
        result.transform.translation() = t;
        return result;
    }

    inline const char *rotationParameterSourceName(
            const RotationParameterSource source) {
        switch (source) {
            case RotationParameterSource::Quaternion:
                return "quaternion";
            case RotationParameterSource::Rpy:
                return "rpy";
            case RotationParameterSource::Matrix:
                return "matrix";
            case RotationParameterSource::Identity:
            default:
                return "identity";
        }
    }

    inline Eigen::Isometry3d applyWorldAlignment(
            const Eigen::Isometry3d &alignment,
            const Eigen::Isometry3d &raw_pose) {
        return alignment * raw_pose;
    }

}  // namespace small_point_lio_pgo

#endif  // SMALL_POINT_LIO_PGO__TRANSFORM_UTILS_HPP
