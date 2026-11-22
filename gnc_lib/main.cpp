#include "conversions/frames/FK5.hpp"
#include "flight_constants.hpp"
#include "gnc_lib.hpp"
#include "navigation/navigation.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace aocs {
namespace {

navigation::NavigationInput mapNavigationInput(const sensorMessage& sensors) {
    navigation::NavigationInput input;
    input.time_J2000_s = sensors.aocs_time_J2000_s;
    if (sensors.has_new_GPS_measurement == 0) { return input; }

    const double gpsTime_J2000_s            = sensors.GPS_time_J2000_s;

    const Eigen::Quaterniond earthRotation  = getQuaternionECEFtoECI(gpsTime_J2000_s);
    const Eigen::Matrix3d ecefToEci         = earthRotation.toRotationMatrix();
    const Eigen::Vector3d position_ecef_m   = Eigen::Map<const Eigen::Vector3d>(sensors.GPS_position_ECEF_m.data());
    const Eigen::Vector3d velocity_ecef_m_s = Eigen::Map<const Eigen::Vector3d>(sensors.GPS_velocity_ECEF_m_s.data());
    const Eigen::Vector3d earthAngularVelocity_ecef_rad_s(0.0, 0.0, flight_constants::kEarthAngularRate);

    navigation::InertialStateEstimate measurement;
    measurement.t_estimate_J2000_s = gpsTime_J2000_s;
    measurement.vec3_r_sc_eci      = ecefToEci * position_ecef_m;
    measurement.vec3_v_sc_eci      = ecefToEci * (velocity_ecef_m_s + earthAngularVelocity_ecef_rad_s.cross(position_ecef_m));

    const navigation::StateCovariance covarianceEcef =
        Eigen::Map<const navigation::StateCovariance>(sensors.GPS_covariance_row_major.data());
    Eigen::Matrix<double, 6, 6> ecefStateToEci = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix3d angularRateCross;
    angularRateCross << 0.0, -earthAngularVelocity_ecef_rad_s.z(), earthAngularVelocity_ecef_rad_s.y(),
        earthAngularVelocity_ecef_rad_s.z(), 0.0, -earthAngularVelocity_ecef_rad_s.x(), -earthAngularVelocity_ecef_rad_s.y(),
        earthAngularVelocity_ecef_rad_s.x(), 0.0;
    ecefStateToEci.block<3, 3>(0, 0) = ecefToEci;
    ecefStateToEci.block<3, 3>(3, 0) = ecefToEci * angularRateCross;
    ecefStateToEci.block<3, 3>(3, 3) = ecefToEci;
    measurement.covariance           = ecefStateToEci * covarianceEcef * ecefStateToEci.transpose();
    input.gpsMeasurement             = std::move(measurement);
    return input;
}

void mapNavigationOutputToIpc(const navigation::InertialStateEstimate& estimate, navigationMessage& message) {
    message.valid = 0;
    if (!std::isfinite(estimate.t_estimate_J2000_s) || !estimate.vec3_r_sc_eci.allFinite() ||
        !estimate.vec3_v_sc_eci.allFinite() || !estimate.covariance.allFinite()) {
        return;
    }

    message.time_J2000_s = estimate.t_estimate_J2000_s;
    for (Eigen::Index axis = 0; axis < static_cast<Eigen::Index>(3); ++axis) {
        const std::size_t index         = static_cast<std::size_t>(axis);
        message.position_ECI_m[index]   = estimate.vec3_r_sc_eci(axis);
        message.velocity_ECI_m_s[index] = estimate.vec3_v_sc_eci(axis);
    }

    const navigation::StateCovariance covariance = estimate.covariance;
    std::copy(covariance.data(), covariance.data() + covariance.size(), message.covariance_row_major.begin());
    message.valid = 1;
}

} // namespace

GNCExecutive initializeGnc() { return GNCExecutive(); }

int runGncStep(GNCExecutive& executive, sharedMemoryMap& sharedMemory) {
    navigationMessage& navigationOutput      = sharedMemory.aocsMemoryRegion.navigationEstimate;
    navigationOutput.valid                   = 0;

    const navigation::NavigationInput inputs = mapNavigationInput(sharedMemory.simMemoryRegion.sensorData);
    executive.stateMachine.evaluate(inputs.gpsMeasurement.has_value());
    if (!executive.stateMachine.navigationReady()) { return 1; }

    const std::optional<navigation::InertialStateEstimate> estimate = executive.navigation.runStep(inputs);
    if (!estimate) { return 1; }

    mapNavigationOutputToIpc(*estimate, navigationOutput);
    return navigationOutput.valid == 1 ? 0 : 1;
}

} // namespace aocs
