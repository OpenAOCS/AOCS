#pragma once

#include "ipc.hpp"

namespace aocs::simulation {

struct Simulation;

namespace data {

void writeState(
    Simulation& simulation,
    const sensorMessage& sensors,
    const navigationEstimateMessage& estimate);

} // namespace data
} // namespace aocs::simulation
