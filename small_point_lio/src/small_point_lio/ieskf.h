/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "eskf.h"
#include <pch.h>

namespace small_point_lio {

    struct frame_measurement_result {// NOLINT(cppcoreguidelines-pro-type-member-init)
        bool valid = false;
        Eigen::Matrix<state::value_type, Eigen::Dynamic, state::DIM> H;
        Eigen::Matrix<state::value_type, Eigen::Dynamic, 1> z;
        state::value_type laser_point_cov = 0.0;
    };

    class ieskf {
    public:
        using measurement_model_imu = std::function<void(const state &, imu_measurement_result &)>;
        using measurement_model_frame = std::function<void(const state &, frame_measurement_result &)>;

        state x;
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> P = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity();

    private:
        double time_predict_state_last = 0.0;
        double time_predict_cov_last = 0.0;
        measurement_model_imu h_imu;
        measurement_model_frame h_frame;

    public:
        ieskf() = default;

        void init(const measurement_model_imu &h_imu, const measurement_model_frame &h_frame);

        void init_timestamp(double timestamp);

        void predict_state(double timestamp);

        void predict_cov(double timestamp, const Eigen::Matrix<state::value_type, state::DIM, state::DIM> &Q);

        bool update_imu();

        bool update_frame_ieskf(int max_iteration = 4, state::value_type convergence_threshold = 1e-4);
    };

}// namespace small_point_lio
