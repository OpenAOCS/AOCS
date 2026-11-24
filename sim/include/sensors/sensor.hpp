#pragma once

#include "common/sensor_events.hpp"

#include <Eigen/Dense>

#include <vector>


class Sensor {
  public:
    struct SensorConfig {
      Eigen::Vector3d position = Eigen::Vector3d::Zero();
      Eigen::Matrix3d dcm_sensor_sf_wrt_bf_bf = Eigen::Matrix3d::Identity(); // DCM from sensor frame to body frame
      SensorEventSensor sensor_ = SensorEventSensor::GPS;
    };

    Sensor(const Sensor::SensorConfig& config, const std::vector<SensorEvent>& events);
    virtual ~Sensor() = default;

    bool isValidAt(double simulationTime_s) const;

    Eigen::Vector3d getPosition() const { return config_.position; }
    Eigen::Matrix3d getDCMSensorSFwrtBFbf() const { return config_.dcm_sensor_sf_wrt_bf_bf; }

  private:
    const Sensor::SensorConfig& config_;
    std::vector<SensorEvent> events_;
};
