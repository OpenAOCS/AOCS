#pragma once

#include "common/random_matrix.hpp"
#include "sensors/sensor.hpp"

#include <Eigen/Dense>

#include <cstdint>
#include <vector>

struct GPS_SensorMsg {
    double t_J2000_s                                             = 0.0;
    Eigen::Vector3d r_sc_ecef_m                                  = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_sc_ecef_m_s                                = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> covariance_ecef = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero();
    bool isValid                                                 = false;
};

class GPS_Sensor : public Sensor {
  public:
    struct GPSConfig : Sensor::SensorConfig {
        double rate_Hz              = 0.0;
        double radialAccuracy_m     = 0.0;
        double inTrackAccuracy_m    = 0.0;
        double crossTrackAccuracy_m = 0.0;
        std::uint64_t randomSeed    = 1;

        GPSConfig() { sensor_ = SensorEventSensor::GPS; }
    };

    GPS_Sensor(const GPSConfig& config, const std::vector<SensorEvent>& events);

    double rate_Hz() const { return config_.rate_Hz; }

    GPS_SensorMsg measure(const Eigen::Vector3d& position_eci_m, const Eigen::Vector3d& velocity_eci_m_s, double t_J2000_s,
                          double simulationTime_s);

  private:
    GPSConfig config_;
    Eigen::Matrix<double, 6, 1> noise_{};
    RandomMatrix<double, 6, 1> noise_matrix_;
};
