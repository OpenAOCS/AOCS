#include "environment/environment.hpp"

#include "environment/earth_rotation/iau2000b.hpp"

namespace aocs::simulation {

Eigen::Vector3d computeEnvironmentalForceEci(
    const Simulation& simulation,
    const SpacecraftState& state,
    double elapsedTime_s) {
    const double t_J2000_s = simulation.epoch_J2000_s + elapsedTime_s;
    const aocs::environment::JulianDate date =
        aocs::environment::JulianDate::fromJ2000Seconds(t_J2000_s);
    const auto earthRotation = aocs::environment::EarthRotation(date, date).reduce();
    const Eigen::Vector3d position_ecef =
        earthRotation.gcrsToItrs * state.position_eci_m;
    const Eigen::Vector3d gravityForce_ecef = simulation.gravity->computeGravity(
        position_ecef,
        simulation.mass_kg,
        simulation.gravityModel,
        simulation.gravityDegree,
        simulation.gravityOrder);

    return earthRotation.gcrsToItrs.transpose() * gravityForce_ecef;
}

} // namespace aocs::simulation
