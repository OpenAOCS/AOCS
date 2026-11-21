#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include "common/randomMatrix.hpp"

struct GPS_SensorMsg {
    Eigen::Vector3d r_sc_ecef;
    Eigen::Vector3d v_sc_ecef;
    Eigen::Matrix<double, 6, 1> measurementCovarianceDiag_ric;
};

class GPS_Sensor {
public:
    GPS_Sensor(const Eigen::Array<double, 6, 1>& measurementCovarianceDiag_lvlh,
               bool isOmnidirectional,
               double GPS_antenna_half_angle,
               std::uint64_t rng_seed);
    ~GPS_Sensor() = default;

    GPS_SensorMsg& GetGPS_SensorMsg(double r_sc_ecef, double v_sc_ecef);

private:
    Eigen::Array<double, 6, 1> measurementCovarianceDiag_lvlh_;
    bool isOmnidirectional_;
    double GPS_antenna_half_angle_;
    Eigen::Matrix<double, 6, 1> noise_{};
    GPS_SensorMsg message_{};
    RandomMatrix<double, 6, 1> noise_matrix_;
};

inline GPS_Sensor::GPS_Sensor(const Eigen::Array<double, 6, 1>& measurementCovarianceDiag_lvlh,
                              bool isOmnidirectional,
                              double GPS_antenna_half_angle,
                              std::uint64_t rng_seed)
    : measurementCovarianceDiag_lvlh_(measurementCovarianceDiag_lvlh),
      isOmnidirectional_(isOmnidirectional),
      GPS_antenna_half_angle_(GPS_antenna_half_angle),
      noise_matrix_(noise_, rng_seed) {}

inline GPS_SensorMsg& GPS_Sensor::GetGPS_SensorMsg(double r_sc_ecef, double v_sc_ecef) {
    noise_matrix_.update();

    Eigen::Matrix<double, 6, 1> state;
    state << r_sc_ecef, r_sc_ecef, r_sc_ecef,
             v_sc_ecef, v_sc_ecef, v_sc_ecef;

    // Scale unit Gaussian noise by standard deviation along each RIC axis
    state.array() += measurementCovarianceDiag_lvlh_ * noise_.array();

    message_.r_sc_ecef = state.head<3>();
    message_.v_sc_ecef = state.tail<3>();
    message_.measurementCovarianceDiag_ric = measurementCovarianceDiag_lvlh_.matrix();

    return message_;
}