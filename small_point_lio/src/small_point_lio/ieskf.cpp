/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "ieskf.h"

namespace small_point_lio {

    void ieskf::init(const measurement_model_imu &h_imu, const measurement_model_frame &h_frame) {
        this->h_imu = h_imu;
        this->h_frame = h_frame;
    }

    void ieskf::init_timestamp(double timestamp) {
        time_predict_state_last = timestamp;
        time_predict_cov_last = timestamp;
    }

    void ieskf::predict_state(double timestamp) {
        auto dt_state = static_cast<state::value_type>(timestamp - time_predict_state_last);
        if (dt_state > 0) [[likely]] {
            time_predict_state_last = timestamp;
            x.position += x.velocity * dt_state;
            x.rotation *= exp<state::value_type>(x.omg * dt_state);
            x.velocity += (x.rotation * x.acceleration + x.gravity) * dt_state;
        }
    }

    void ieskf::predict_cov(double timestamp, const Eigen::Matrix<state::value_type, state::DIM, state::DIM> &Q) {
        auto dt_cov = static_cast<state::value_type>(timestamp - time_predict_cov_last);
        if (dt_cov > 0) [[likely]] {
            time_predict_cov_last = timestamp;
            Eigen::Matrix<state::value_type, 3, 1> seg_SO3 = -x.omg * dt_cov;
            Eigen::Matrix<state::value_type, state::DIM, state::DIM> F = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity();
            F.block<3, 3>(state::position_index, state::velocity_index).diagonal().fill(dt_cov);
            F.block<3, 3>(state::rotation_index, state::rotation_index) = exp<state::value_type>(seg_SO3);
            F.block<3, 3>(state::rotation_index, state::omg_index) = A_matrix<state::value_type>(seg_SO3) * dt_cov;
            F.block<3, 3>(state::velocity_index, state::rotation_index) = -x.rotation * hat<state::value_type>(x.acceleration);
            F.block<3, 3>(state::velocity_index, state::acceleration_index) = x.rotation * dt_cov;
            F.block<3, 3>(state::velocity_index, state::gravity_index).diagonal().fill(dt_cov);
            P = F * P * F.transpose() + Q * (dt_cov * dt_cov);
        }
    }

    bool ieskf::update_imu() {
        if (!h_imu) {
            return false;
        }
        imu_measurement_result measurement_result;
        h_imu(x, measurement_result);
        Eigen::Matrix<state::value_type, 6, 1> z = measurement_result.z;
        Eigen::Matrix<state::value_type, state::DIM, 6> PHT = Eigen::Matrix<state::value_type, state::DIM, 6>::Zero();
        Eigen::Matrix<state::value_type, 6, state::DIM> HP = Eigen::Matrix<state::value_type, 6, state::DIM>::Zero();
        Eigen::Matrix<state::value_type, 6, 6> HPHT = Eigen::Matrix<state::value_type, 6, 6>::Zero();

        for (int i = 0; i < 3; i++) {
            if (!measurement_result.satu_check[i]) {
                PHT.col(i) = P.col(state::omg_index + i) + P.col(state::bg_index + i);
                HP.row(i) = P.row(state::omg_index + i) + P.row(state::bg_index + i);
            }
            if (!measurement_result.satu_check[i + 3]) {
                PHT.col(i + 3) = P.col(state::acceleration_index + i) + P.col(state::ba_index + i);
                HP.row(i + 3) = P.row(state::acceleration_index + i) + P.row(state::ba_index + i);
            }
        }
        for (int i = 0; i < 3; i++) {
            if (!measurement_result.satu_check[i]) {
                HPHT.col(i) = HP.col(state::omg_index + i) + HP.col(state::bg_index + i);
            }
            if (!measurement_result.satu_check[i + 3]) {
                HPHT.col(i + 3) = HP.col(state::acceleration_index + i) + HP.col(state::ba_index + i);
            }
            HPHT(i, i) += measurement_result.imu_meas_omg_cov;
            HPHT(i + 3, i + 3) += measurement_result.imu_meas_acc_cov;
        }

        Eigen::LDLT<Eigen::Matrix<state::value_type, 6, 6>> ldlt(HPHT);
        if (ldlt.info() != Eigen::Success) [[unlikely]] {
            return false;
        }
        Eigen::Matrix<state::value_type, state::DIM, 6> K = PHT * ldlt.solve(Eigen::Matrix<state::value_type, 6, 6>::Identity());
        x.plus(K * z);
        P -= K * HP;
        return true;
    }

    bool ieskf::update_frame_ieskf(int max_iteration, state::value_type convergence_threshold) {
        if (!h_frame || max_iteration <= 0) {
            return false;
        }

        bool updated = false;
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> prior_information = P.inverse();

        for (int iter = 0; iter < max_iteration; iter++) {
            frame_measurement_result measurement_result;
            h_frame(x, measurement_result);
            if (!measurement_result.valid || measurement_result.H.rows() == 0 || measurement_result.H.rows() != measurement_result.z.rows()) {
                return updated;
            }

            state::value_type cov = measurement_result.laser_point_cov;
            if (cov <= 0) {
                cov = 1e-6;
            }

            Eigen::Matrix<state::value_type, state::DIM, state::DIM> HTH = measurement_result.H.transpose() * measurement_result.H;
            Eigen::Matrix<state::value_type, state::DIM, 1> HTz = measurement_result.H.transpose() * measurement_result.z;
            Eigen::Matrix<state::value_type, state::DIM, state::DIM> A = prior_information + HTH / cov;
            Eigen::Matrix<state::value_type, state::DIM, 1> b = HTz / cov;

            Eigen::LDLT<Eigen::Matrix<state::value_type, state::DIM, state::DIM>> ldlt(A);
            if (ldlt.info() != Eigen::Success) [[unlikely]] {
                return updated;
            }

            Eigen::Matrix<state::value_type, state::DIM, 1> dx = ldlt.solve(b);
            x.plus(dx);
            updated = true;
            if (dx.norm() < convergence_threshold) {
                P = ldlt.solve(Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity());
                return true;
            }
        }

        frame_measurement_result measurement_result;
        h_frame(x, measurement_result);
        if (!measurement_result.valid || measurement_result.H.rows() == 0 || measurement_result.H.rows() != measurement_result.z.rows()) {
            return updated;
        }
        state::value_type cov = measurement_result.laser_point_cov;
        if (cov <= 0) {
            cov = 1e-6;
        }
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> A = prior_information + measurement_result.H.transpose() * measurement_result.H / cov;
        Eigen::LDLT<Eigen::Matrix<state::value_type, state::DIM, state::DIM>> ldlt(A);
        if (ldlt.info() == Eigen::Success) {
            P = ldlt.solve(Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity());
        }
        return updated;
    }

}// namespace small_point_lio
