#pragma once

#include "simulation/simulation.hpp"

#include <memory>

namespace aocs::simulation {

std::unique_ptr<Simulation> initializeSimulation();

} // namespace aocs::simulation
