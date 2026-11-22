#include "data/file_writer.hpp"
#include "data/telemetry_writer.hpp"
#include "gnc_lib.hpp"
#include "ipc.hpp"
#include "simulation/initialization.hpp"
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <unistd.h>

namespace {

int runAOCSLoop(GncTransport& transport, const sensorMessage& sensors) {
    return transport.exchange(sensors);
}

} // namespace

int main() {
    std::unique_ptr<aocs::simulation::Simulation> simulation =
        aocs::simulation::initializeSimulation();
    if (!simulation) {
        return 1;
    }

    try {
        SharedMemoryRegion sharedMemory;
        aocs::GNCExecutive gncExecutive = aocs::initializeGnc();
        SharedMemoryGncTransport transport(
            sharedMemory.data(),
            [&gncExecutive](sharedMemoryMap& memory) {
                return aocs::runGncStep(gncExecutive, memory);
            });

        const bool interactive = isatty(STDOUT_FILENO) != 0;
        bool hasDrawnTelemetry = false;
        simulation->sim_start = std::chrono::steady_clock::now();
        auto lastTelemetryUpdate = simulation->sim_start;
        aocs::simulation::data::publishTelemetry(
            *simulation, interactive, hasDrawnTelemetry);
        sensorMessage latestGpsMeasurement;
        navigationMessage latestNavigationEstimate{};
        std::uint64_t trueStateStep = 0;
        std::uint64_t aocsStep = 0;
        while (simulation->running) {
            simulation->Step();
            ++trueStateStep;
            if (trueStateStep % simulation->aocsIntervalSteps == 0) {
                if (aocsStep % simulation->gpsSampleIntervalSteps == 0) {
                    latestGpsMeasurement = simulation->sensors.gps.measure(
                        simulation->state,
                        simulation->epoch_J2000_s +
                            simulation->elapsedTime_s);
                } else {
                    latestGpsMeasurement.has_new_GPS_measurement = 0;
                    latestGpsMeasurement.aocs_time_J2000_s =
                        simulation->epoch_J2000_s + simulation->elapsedTime_s;
                }
                if (runAOCSLoop(transport, latestGpsMeasurement) != 0) {
                    std::cerr << "GNC loop reported a failure\n";
                    return 1;
                }
                latestNavigationEstimate =
                    sharedMemory.data().aocsMemoryRegion.navigationEstimate;
                if (latestNavigationEstimate.valid == 0) {
                    std::cerr << "GNC did not publish a valid navigation estimate\n";
                    return 1;
                }
                ++aocsStep;
            }

            if (latestNavigationEstimate.valid != 0 &&
                trueStateStep % simulation->logIntervalSteps == 0) {
                simulation->outputFile->writeState(
                    simulation->epoch_J2000_s,
                    simulation->elapsedTime_s,
                    simulation->state.position_eci_m,
                    simulation->state.velocity_eci_m_s,
                    latestGpsMeasurement,
                    latestNavigationEstimate);
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastTelemetryUpdate >= std::chrono::milliseconds(100)) {
                aocs::simulation::data::publishTelemetry(
                    *simulation, interactive, hasDrawnTelemetry);
                lastTelemetryUpdate = now;
            }
            if (simulation->elapsedTime_s >= simulation->simulationLength_s) {
                simulation->running = false;
            }
        }
        aocs::simulation::data::publishTelemetry(
            *simulation, interactive, hasDrawnTelemetry);
        simulation->outputFile->close();
    } catch (const std::exception& error) {
        std::cerr << "Simulation runtime failure: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
