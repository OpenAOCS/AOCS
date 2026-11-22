#include "simulation/simulation.hpp"

#include "environment/earth_rotation/iau2000b.hpp"
#include "sim_constants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aocs::simulation {

Simulation::Sensors::GPS::GPS(const Config& config)
    : config_(config) {
    if (!std::isfinite(config_.rate_Hz) || config_.rate_Hz <= 0.0 ||
        !std::isfinite(config_.radialAccuracy_m) ||
        config_.radialAccuracy_m <= 0.0 ||
        !std::isfinite(config_.inTrackAccuracy_m) ||
        config_.inTrackAccuracy_m <= 0.0 ||
        !std::isfinite(config_.crossTrackAccuracy_m) ||
        config_.crossTrackAccuracy_m <= 0.0 ||
        !std::isfinite(config_.radialAccuracy_m * config_.radialAccuracy_m) ||
        !std::isfinite(
            config_.inTrackAccuracy_m * config_.inTrackAccuracy_m) ||
        !std::isfinite(
            config_.crossTrackAccuracy_m * config_.crossTrackAccuracy_m)) {
        throw std::invalid_argument(
            "GPS rate and position accuracies must be finite and positive");
    }
}

sensorMessage Simulation::Sensors::GPS::measure(
    const SpacecraftState& state,
    double t_J2000_s) const {
    const Eigen::Vector3d angularMomentum =
        state.position_eci_m.cross(state.velocity_eci_m_s);
    const double positionSquared = state.position_eci_m.squaredNorm();
    const double angularMomentumSquared = angularMomentum.squaredNorm();
    if (!state.position_eci_m.allFinite() ||
        !state.velocity_eci_m_s.allFinite() ||
        !std::isfinite(t_J2000_s) ||
        !std::isfinite(positionSquared) || positionSquared == 0.0 ||
        !std::isfinite(angularMomentumSquared) ||
        angularMomentumSquared == 0.0) {
        throw std::invalid_argument(
            "GPS measurement requires a finite non-degenerate spacecraft state");
    }

    const aocs::environment::JulianDate date =
        aocs::environment::JulianDate::fromJ2000Seconds(t_J2000_s);
    const Eigen::Matrix3d eciToEcef =
        aocs::environment::EarthRotation(date, date).reduce().gcrsToItrs;
    const Eigen::Vector3d position_ecef_m =
        eciToEcef * state.position_eci_m;
    const Eigen::Vector3d earthAngularVelocity_ecef_rad_s(
        0.0, 0.0, SimConstants::kEarthAngularRate);
    const Eigen::Vector3d velocity_ecef_m_s =
        eciToEcef * state.velocity_eci_m_s -
        earthAngularVelocity_ecef_rad_s.cross(position_ecef_m);

    const Eigen::Vector3d radialDirection_eci =
        state.position_eci_m.normalized();
    const Eigen::Vector3d orbitNormal_eci = angularMomentum.normalized();
    const Eigen::Vector3d inTrackDirection_eci =
        orbitNormal_eci.cross(radialDirection_eci).normalized();
    Eigen::Matrix3d rtnToEci;
    rtnToEci.col(0) = radialDirection_eci;
    rtnToEci.col(1) = inTrackDirection_eci;
    rtnToEci.col(2) = orbitNormal_eci;

    const Eigen::Vector3d accuracy_m(
        config_.radialAccuracy_m,
        config_.inTrackAccuracy_m,
        config_.crossTrackAccuracy_m);
    const Eigen::Matrix3d covarianceRtn_m2 =
        accuracy_m.array().square().matrix().asDiagonal();
    const Eigen::Matrix3d covarianceEci_m2 =
        rtnToEci * covarianceRtn_m2 * rtnToEci.transpose();
    const Eigen::Matrix3d covarianceEcef_m2 =
        eciToEcef * covarianceEci_m2 * eciToEcef.transpose();

    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> covariance =
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Identity();
    covariance.block<3, 3>(0, 0) = covarianceEcef_m2;

    sensorMessage message;
    message.GPS_position_ECEF_m = {
        position_ecef_m.x(), position_ecef_m.y(), position_ecef_m.z()};
    message.GPS_velocity_ECEF_m_s = {
        velocity_ecef_m_s.x(), velocity_ecef_m_s.y(), velocity_ecef_m_s.z()};
    message.GPS_time_s = t_J2000_s + SimConstants::kGpsSecondsAtJ2000;
    message.aocs_time_J2000_s = t_J2000_s;
    message.has_new_GPS_measurement = 1;
    std::copy(
        covariance.data(),
        covariance.data() + covariance.size(),
        message.GPS_covariance_row_major.begin());
    return message;
}

} // namespace aocs::simulation
