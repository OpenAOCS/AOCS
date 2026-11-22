#include "sensors/gps.hpp"

#include "environment/earth_rotation/iau2000b.hpp"
#include "sim_constants.hpp"

#include <cmath>
#include <stdexcept>

GPS_Sensor::GPS_Sensor(
    const Config& config,
    const std::vector<SensorEvent>& events)
    : Sensor(SensorEventSensor::GPS, events),
      config_(config),
      noise_matrix_(noise_, config.randomSeed) {
    const Eigen::Vector3d accuracy(
        config_.radialAccuracy_m,
        config_.inTrackAccuracy_m,
        config_.crossTrackAccuracy_m);
    if (!std::isfinite(config_.rate_Hz) || config_.rate_Hz <= 0.0 ||
        !accuracy.allFinite() || (accuracy.array() <= 0.0).any() ||
        !(accuracy.array().square().isFinite()).all()) {
        throw std::invalid_argument(
            "GPS rate and position accuracies must be finite and positive");
    }
}

GPS_SensorMsg GPS_Sensor::measure(
    const Eigen::Vector3d& position_eci_m,
    const Eigen::Vector3d& velocity_eci_m_s,
    double t_J2000_s,
    double simulationTime_s) {
    const Eigen::Vector3d angularMomentum =
        position_eci_m.cross(velocity_eci_m_s);
    const double positionSquared = position_eci_m.squaredNorm();
    const double angularMomentumSquared = angularMomentum.squaredNorm();
    if (!position_eci_m.allFinite() ||
        !velocity_eci_m_s.allFinite() ||
        !std::isfinite(t_J2000_s) ||
        !std::isfinite(positionSquared) || positionSquared == 0.0 ||
        !std::isfinite(angularMomentumSquared) ||
        angularMomentumSquared == 0.0) {
        throw std::invalid_argument(
            "GPS measurement requires a finite non-degenerate spacecraft state");
    }
    GPS_SensorMsg message;
    message.isValid = isValidAt(simulationTime_s);
    if (!message.isValid) {
        return message;
    }

    const aocs::environment::JulianDate date =
        aocs::environment::JulianDate::fromJ2000Seconds(t_J2000_s);
    const Eigen::Matrix3d eciToEcef =
        aocs::environment::EarthRotation(date, date).reduce().gcrsToItrs;
    const Eigen::Vector3d earthAngularVelocity_ecef_rad_s(
        0.0, 0.0, SimConstants::kEarthAngularRate);
    const Eigen::Vector3d radialDirection_eci = position_eci_m.normalized();
    const Eigen::Vector3d orbitNormal_eci = angularMomentum.normalized();
    const Eigen::Vector3d inTrackDirection_eci =
        orbitNormal_eci.cross(radialDirection_eci).normalized();

    Eigen::Matrix3d rtnToEci;
    rtnToEci.col(0) = radialDirection_eci;
    rtnToEci.col(1) = inTrackDirection_eci;
    rtnToEci.col(2) = orbitNormal_eci;

    Eigen::Matrix<double, 6, 1> accuracy;
    accuracy << config_.radialAccuracy_m,
        config_.inTrackAccuracy_m,
        config_.crossTrackAccuracy_m,
        1.0, 1.0, 1.0;
    Eigen::Matrix<double, 6, 6> rtnToEcefState =
        Eigen::Matrix<double, 6, 6>::Zero();
    const Eigen::Matrix3d rtnToEcef = eciToEcef * rtnToEci;
    Eigen::Matrix3d omegaCross;
    omegaCross << 0.0, -earthAngularVelocity_ecef_rad_s.z(),
        earthAngularVelocity_ecef_rad_s.y(),
        earthAngularVelocity_ecef_rad_s.z(), 0.0,
        -earthAngularVelocity_ecef_rad_s.x(),
        -earthAngularVelocity_ecef_rad_s.y(),
        earthAngularVelocity_ecef_rad_s.x(), 0.0;
    rtnToEcefState.block<3, 3>(0, 0) = rtnToEcef;
    rtnToEcefState.block<3, 3>(3, 0) = -omegaCross * rtnToEcef;
    rtnToEcefState.block<3, 3>(3, 3) = rtnToEcef;

    const Eigen::Matrix<double, 6, 6> rtnCovariance =
        accuracy.array().square().matrix().asDiagonal();
    message.covariance_ecef =
        rtnToEcefState * rtnCovariance * rtnToEcefState.transpose();

    noise_matrix_.update();
    const Eigen::Vector3d noisePositionRtn_m =
        accuracy.head<3>().cwiseProduct(noise_.head<3>());
    const Eigen::Vector3d noiseVelocityRtn_m_s =
        accuracy.tail<3>().cwiseProduct(noise_.tail<3>());
    const Eigen::Vector3d position_ecef_m =
        eciToEcef *
        (position_eci_m + rtnToEci * noisePositionRtn_m);
    const Eigen::Vector3d velocity_ecef_m_s =
        eciToEcef *
            (velocity_eci_m_s + rtnToEci * noiseVelocityRtn_m_s) -
        earthAngularVelocity_ecef_rad_s.cross(position_ecef_m);
    message.r_sc_ecef_m = position_ecef_m;
    message.v_sc_ecef_m_s = velocity_ecef_m_s;
    return message;
}