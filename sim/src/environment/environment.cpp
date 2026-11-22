#include "simulation/simulation.hpp"

#include "environment/earth_rotation/iau2000b.hpp"

#include <stdexcept>

namespace aocs::simulation {

Simulation::Environment::Environment(const Config& config) : config_(config) {
    if (config_.gravityModel == GravityModel::E_EGM2008) {
        if (config_.gravityOrder > config_.gravityDegree) {
            throw std::invalid_argument("EGM2008 gravity order must not exceed degree");
        }
        coefficients_ = std::make_unique<EGM2008Coefficients>(config_.coefficientsPath);
        if (config_.gravityDegree > coefficients_->maxDegree) {
            throw std::invalid_argument("EGM2008 gravity degree exceeds the coefficient dataset maximum");
        }
    } else if (config_.gravityModel != GravityModel::E_PointMass) {
        throw std::invalid_argument("Unsupported gravity model");
    }

    gravity_ = std::make_unique<Gravity>(coefficients_.get());
}

Eigen::Vector3d Simulation::Environment::computeForceEci(const SpacecraftState& state, double epoch_J2000_s,
                                                         double elapsedTime_s, double mass_kg) const {
    const aocs::environment::JulianDate date = aocs::environment::JulianDate::fromJ2000Seconds(epoch_J2000_s + elapsedTime_s);
    const Eigen::Matrix3d eciToEcef          = aocs::environment::EarthRotation(date, date).reduce().gcrsToItrs;
    const Eigen::Vector3d position_ecef_m    = eciToEcef * state.position_eci_m;
    const Eigen::Vector3d force_ecef_N =
        gravity_->computeGravity(position_ecef_m, mass_kg, config_.gravityModel, config_.gravityDegree, config_.gravityOrder);
    return eciToEcef.transpose() * force_ecef_N;
}

} // namespace aocs::simulation
