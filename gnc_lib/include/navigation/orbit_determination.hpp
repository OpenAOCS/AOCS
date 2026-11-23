#pragma once

#include <cstdint>

#include <Eigen/Dense>

#include <math/RK4.hpp>

namespace aocs::navigation {

using StateCovariance = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

enum class OrbitDeterminationMethod : std::uint8_t { ExtendedKalmanFilter, UnscentedKalmanFilter };

struct InertialStateEstimate {
    double t_estimate_J2000_s                 = 0.0;
    Eigen::Matrix<double, 3, 1> vec3_r_sc_eci = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 3, 1> vec3_v_sc_eci = Eigen::Matrix<double, 3, 1>::Zero();
    StateCovariance covariance                = StateCovariance::Zero();
};

class OrbitDetermination {
  public:
    explicit OrbitDetermination(OrbitDeterminationMethod method,
                                const Eigen::Vector3d& accelerationProcessNoiseSpectralDensityRTN_m2_s3);

    void InitializeInertialState(const InertialStateEstimate& initialEstimate);
    void PropagateTo(double t_J2000_s);
    void UpdateInertialState(const InertialStateEstimate& measurement, double t_J2000_s);
    const InertialStateEstimate& GetInertialStateEstimate() const;

  private:
    void PropagateInertialState(InertialStateEstimate& state, double t_start_J2000_s, double t_end_J2000_s);
    void MeasurementUpdate(InertialStateEstimate& estimate, const InertialStateEstimate& measurement);

    OrbitDeterminationMethod method;
    Eigen::Vector3d accelerationProcessNoiseSpectralDensityRTN_m2_s3;
    InertialStateEstimate inertialState;
    Rk4Workspace<Eigen::Matrix<double, 6, 1>> rk4Workspace{6};
};

} // namespace aocs::navigation
