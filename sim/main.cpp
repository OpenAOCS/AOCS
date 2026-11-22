#include "data/file_writer.hpp"
#include "data/telemetry_writer.hpp"
#include "environment/environment.hpp"
#include "environment/earth_rotation/iau2000b.hpp"
#include "gnc_lib.hpp"
#include "ipc.hpp"
#include "sim_constants.hpp"
#include "simulation/initialization.hpp"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <unistd.h>

namespace {

constexpr double kPlaceholderGpsMeasurementVariance = 1.0;

struct ActuatorState {
    Eigen::Vector3d force_eci_N = Eigen::Vector3d::Zero();
};

ActuatorState updateActuatorDynamics(
    const aocs::simulation::Simulation&) {
    return {};
}

sensorMessage updateSensorModels(
    const aocs::simulation::Simulation& simulation) {
    const double t_J2000_s =
        simulation.epoch_J2000_s + simulation.elapsedTime_s;
    const aocs::environment::JulianDate date =
        aocs::environment::JulianDate::fromJ2000Seconds(t_J2000_s);
    const auto earthRotation =
        aocs::environment::EarthRotation(date, date).reduce();
    const Eigen::Vector3d position_ecef_m =
        earthRotation.gcrsToItrs * simulation.state.position_eci_m;
    const Eigen::Vector3d earthAngularVelocity_ecef_rad_s(
        0.0, 0.0, SimConstants::kEarthAngularRate);

    sensorMessage sensors;
    sensors.GPS_position_ECEF_m = {
        position_ecef_m.x(), position_ecef_m.y(), position_ecef_m.z()};
    const Eigen::Vector3d velocity_ecef_m_s =
        earthRotation.gcrsToItrs * simulation.state.velocity_eci_m_s -
        earthAngularVelocity_ecef_rad_s.cross(position_ecef_m);
    sensors.GPS_velocity_ECEF_m_s = {
        velocity_ecef_m_s.x(), velocity_ecef_m_s.y(), velocity_ecef_m_s.z()};
    sensors.GPS_time_s = t_J2000_s + aocs::ipc::kGpsSecondsAtJ2000;
    sensors.aocs_time_J2000_s = t_J2000_s;
    sensors.has_new_GPS_measurement = 1;
    for (std::size_t axis = 0; axis < aocs::ipc::kStateDimension; ++axis) {
        sensors.GPS_covariance_row_major[
            axis * aocs::ipc::kStateDimension + axis] =
            kPlaceholderGpsMeasurementVariance;
    }
    return sensors;
}

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
        navigationEstimateMessage latestNavigationEstimate{};
        std::uint64_t trueStateStep = 0;
        std::uint64_t aocsStep = 0;
        while (simulation->running) {
            const ActuatorState actuators =
                updateActuatorDynamics(*simulation);
            simulation->propagation.propagate(
                simulation->state,
                simulation->elapsedTime_s,
                simulation->dt_sec,
                [&](const aocs::simulation::SpacecraftState& state,
                    double elapsedTime_s) -> Eigen::Vector3d {
                    return (aocs::simulation::computeEnvironmentalForceEci(
                                *simulation, state, elapsedTime_s) +
                            actuators.force_eci_N) /
                           simulation->mass_kg;
                });
            ++trueStateStep;
            if (trueStateStep % simulation->aocsIntervalSteps == 0) {
                if (aocsStep % simulation->gpsSampleIntervalSteps == 0) {
                    latestGpsMeasurement = updateSensorModels(*simulation);
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
                    sharedMemory.data().navigationEstimate;
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
