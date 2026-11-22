#pragma once

namespace aocs::simulation {

struct Simulation;

namespace data {

void publishTelemetry(
    const Simulation& simulation,
    bool interactive,
    bool& hasDrawnTelemetry);

} // namespace data
} // namespace aocs::simulation
