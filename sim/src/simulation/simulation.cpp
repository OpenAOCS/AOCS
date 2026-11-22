#include "simulation/simulation.hpp"

#include "telemetry/file_writer.hpp"
#include "environment/earth_rotation/iau2000b.hpp"
#include "sim_constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aocs::simulation {
namespace {

std::uint64_t intervalSteps(
    double fasterRate_Hz,
    double slowerRate_Hz,
    const char* errorMessage) {
    if (!std::isfinite(fasterRate_Hz) || !std::isfinite(slowerRate_Hz) ||
        fasterRate_Hz <= 0.0 || slowerRate_Hz <= 0.0) {
        throw std::invalid_argument("Simulation rates must be finite and positive");
    }

    const double ratio = fasterRate_Hz / slowerRate_Hz;
    const double roundedRatio = std::round(ratio);
    if (!std::isfinite(ratio) || roundedRatio < 1.0 ||
        static_cast<long double>(roundedRatio) >
            static_cast<long double>(
                std::numeric_limits<std::uint64_t>::max()) ||
        std::abs(ratio - roundedRatio) > 1e-9) {
        throw std::invalid_argument(errorMessage);
    }
    return static_cast<std::uint64_t>(roundedRatio);
}

} // namespace

Simulation::Simulation(const Config& config)
    : state{
          config.initialConditions.position_eci_m,
          config.initialConditions.velocity_eci_m_s},
      epoch_J2000_s(config.initialConditions.epoch_J2000_s),
      mass_kg(config.spacecraft.mass_kg),
      environment(config.environment),
      sensors(config.sensors),
      actuators(config.actuators) {
    if (!state.position_eci_m.allFinite() ||
        !state.velocity_eci_m_s.allFinite() ||
        !std::isfinite(epoch_J2000_s)) {
        throw std::invalid_argument(
            "Initial state and epoch must contain only finite values");
    }
    if (!std::isfinite(mass_kg) || mass_kg <= 0.0) {
        throw std::invalid_argument("Spacecraft mass must be finite and positive");
    }
    if (config.timing.orbitCount == 0) {
        throw std::invalid_argument("Simulation orbit count must be positive");
    }

    dt_sec = 1.0 / config.timing.trueStateRate_Hz;
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
        throw std::invalid_argument(
            "True-state rate must produce a positive finite timestep");
    }
    aocsIntervalSteps = intervalSteps(
        config.timing.trueStateRate_Hz,
        config.timing.aocsRate_Hz,
        "True-state rate must be an integer multiple of the AOCS rate");
    logIntervalSteps = intervalSteps(
        config.timing.trueStateRate_Hz,
        config.timing.logRate_Hz,
        "True-state rate must be an integer multiple of the log rate");
    gpsSampleIntervalSteps = intervalSteps(
        config.timing.aocsRate_Hz,
        sensors.gps.rate_Hz(),
        "AOCS rate must be an integer multiple of the GPS rate");

    const double initialRadius_m = state.position_eci_m.norm();
    const double angularMomentumSquared =
        state.position_eci_m.cross(state.velocity_eci_m_s).squaredNorm();
    if (!std::isfinite(initialRadius_m) || initialRadius_m <= 0.0 ||
        !std::isfinite(angularMomentumSquared) ||
        angularMomentumSquared <= 0.0) {
        throw std::invalid_argument(
            "Initial conditions must define a finite, non-degenerate orbit");
    }
    const double specificEnergy_m2_s2 =
        0.5 * state.velocity_eci_m_s.squaredNorm() -
        SimConstants::kEarthMu / initialRadius_m;
    if (!std::isfinite(specificEnergy_m2_s2) ||
        specificEnergy_m2_s2 >= 0.0) {
        throw std::invalid_argument(
            "Initial conditions must define a finite bound Earth orbit");
    }

    const double semiMajorAxis_m =
        -SimConstants::kEarthMu / (2.0 * specificEnergy_m2_s2);
    const double orbitalPeriod_s =
        2.0 * SimConstants::kPi *
        std::sqrt(semiMajorAxis_m * semiMajorAxis_m * semiMajorAxis_m /
                  SimConstants::kEarthMu);
    const double requestedDuration_s =
        static_cast<double>(config.timing.orbitCount) * orbitalPeriod_s;
    simulationLength_s =
        std::round(requestedDuration_s / dt_sec) * dt_sec;
    if (!std::isfinite(simulationLength_s) || simulationLength_s <= 0.0) {
        throw std::invalid_argument(
            "Simulation duration must be finite and positive");
    }
}

Simulation::~Simulation() = default;

void Simulation::Step() {
    propagation_.propagate(
        state,
        elapsedTime_s,
        dt_sec,
        [this](const SpacecraftState& intermediate,
               double elapsedTime_s) -> Eigen::Vector3d {
            // Attitude is not modeled yet, so actuator body axes are ECI-aligned.
            const Eigen::Vector3d netForceEci_N =
                environment.computeForceEci(
                    intermediate,
                    epoch_J2000_s,
                    elapsedTime_s,
                    mass_kg) +
                actuators.netForceBody_N;
            const Eigen::Vector3d momentumRateEci_N = netForceEci_N;
            return momentumRateEci_N / mass_kg;
        });
}

} // namespace aocs::simulation
