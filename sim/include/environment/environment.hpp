#pragma once

#include "simulation/initialization.hpp"

namespace aocs::simulation {

Eigen::Vector3d computeEnvironmentalForceEci(
    const Simulation& simulation,
    const SpacecraftState& state,
    double elapsedTime_s);

} // namespace aocs::simulation