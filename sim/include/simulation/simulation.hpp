#pragma once

#include "environment/earth_gravity/gravity.hpp"
#include "sensors/gps.hpp"
#include "sim_to_aocs.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace aocs::simulation {

namespace data {
class FileWriter;
}

struct SpacecraftState {
    Eigen::Vector3d position_eci_m;
    Eigen::Vector3d velocity_eci_m_s;
};

class Propagation {
  public:
    using AccelerationFunction = std::function<Eigen::Vector3d(const SpacecraftState&, double)>;

    class RK4 {
      public:
        static void integrate(SpacecraftState& state, double time_s, double dt_sec, const AccelerationFunction& acceleration) {
            if (!std::isfinite(time_s) || !std::isfinite(dt_sec) || dt_sec <= 0.0 || !acceleration) {
                throw std::invalid_argument("RK4 requires finite time, a positive finite timestep, "
                                            "and an acceleration function");
            }

            const StateDerivative k1 = evaluate(state, time_s, acceleration);
            const StateDerivative k2 = evaluate(offsetState(state, k1, 0.5 * dt_sec), time_s + 0.5 * dt_sec, acceleration);
            const StateDerivative k3 = evaluate(offsetState(state, k2, 0.5 * dt_sec), time_s + 0.5 * dt_sec, acceleration);
            const StateDerivative k4 = evaluate(offsetState(state, k3, dt_sec), time_s + dt_sec, acceleration);

            const double scale       = dt_sec / 6.0;
            SpacecraftState nextState{state.position_eci_m + scale * (k1.positionRate + 2.0 * k2.positionRate +
                                                                      2.0 * k3.positionRate + k4.positionRate),
                                      state.velocity_eci_m_s + scale * (k1.velocityRate + 2.0 * k2.velocityRate +
                                                                        2.0 * k3.velocityRate + k4.velocityRate)};
            if (!nextState.position_eci_m.allFinite() || !nextState.velocity_eci_m_s.allFinite()) {
                throw std::runtime_error("RK4 produced a non-finite spacecraft state");
            }
            state = nextState;
        }

      private:
        struct StateDerivative {
            Eigen::Vector3d positionRate;
            Eigen::Vector3d velocityRate;
        };

        static SpacecraftState offsetState(const SpacecraftState& state, const StateDerivative& derivative, double scale) {
            return {state.position_eci_m + scale * derivative.positionRate,
                    state.velocity_eci_m_s + scale * derivative.velocityRate};
        }

        static StateDerivative evaluate(const SpacecraftState& state, double time_s, const AccelerationFunction& acceleration) {
            const Eigen::Vector3d velocityRate = acceleration(state, time_s);
            if (!velocityRate.allFinite()) { throw std::runtime_error("RK4 acceleration must be finite"); }
            return {state.velocity_eci_m_s, velocityRate};
        }
    };

    void propagate(SpacecraftState& state, double& elapsedTime_s, double dt_sec,
                   const AccelerationFunction& acceleration) const {
        RK4::integrate(state, elapsedTime_s, dt_sec, acceleration);
        elapsedTime_s += dt_sec;
    }
};

class Simulation {
  public:
    class Environment {
      public:
        struct Config {
            GravityModel gravityModel    = GravityModel::E_PointMass;
            std::uint32_t gravityDegree  = 0;
            std::uint32_t gravityOrder   = 0;
            std::string coefficientsPath = "data/earth_gravity/emg2008_coefficients.bin";
        };

        explicit Environment(const Config& config);

        Eigen::Vector3d computeForceEci(const SpacecraftState& state, double epoch_J2000_s, double elapsedTime_s,
                                        double mass_kg) const;

      private:
        Config config_;
        std::unique_ptr<EGM2008Coefficients> coefficients_;
        std::unique_ptr<Gravity> gravity_;
    };

    class Sensors {
      public:
        class GPS {
          public:
            using Config = GPS_Sensor::Config;

            GPS(const Config& config, const std::vector<SensorEvent>& events = {});

            sensorMessage measure(const SpacecraftState& state, double epoch_J2000_s, double simulationTime_s);

            double rate_Hz() const { return model_.rate_Hz(); }

          private:
            GPS_Sensor model_;
        };

        struct Config {
            GPS::Config gps;
            std::vector<SensorEvent> events;
        };

        explicit Sensors(const Config& config) : gps(config.gps, config.events) {}

        GPS gps;
    };

    class Actuators {
      public:
        struct Config {};

        explicit Actuators(const Config&) {}

        Eigen::Vector3d netForceBody_N   = Eigen::Vector3d::Zero();
        Eigen::Vector3d netTorqueBody_Nm = Eigen::Vector3d::Zero();
    };

    struct Config {
        struct InitialConditions {
            Eigen::Vector3d position_eci_m   = Eigen::Vector3d::Zero();
            Eigen::Vector3d velocity_eci_m_s = Eigen::Vector3d::Zero();
            double epoch_J2000_s             = 0.0;
        } initialConditions;

        struct Timing {
            std::uint32_t orbitCount = 0;
            double trueStateRate_Hz  = 0.0;
            double aocsRate_Hz       = 0.0;
            double logRate_Hz        = 0.0;
        } timing;

        struct Spacecraft {
            double mass_kg = 0.0;
        } spacecraft;

        Environment::Config environment;
        Sensors::Config sensors;
        Actuators::Config actuators;
    };

    explicit Simulation(const Config& config);
    ~Simulation();

    void Step();

    SpacecraftState state;
    double epoch_J2000_s                 = 0.0;
    double elapsedTime_s                 = 0.0;
    double simulationLength_s            = 0.0;
    double dt_sec                        = 0.0;
    std::uint64_t aocsIntervalSteps      = 0;
    std::uint64_t logIntervalSteps       = 0;
    std::uint64_t gpsSampleIntervalSteps = 0;
    double mass_kg                       = 0.0;
    std::chrono::steady_clock::time_point sim_start;
    Environment environment;
    Sensors sensors;
    Actuators actuators;
    std::unique_ptr<data::FileWriter> outputFile;
    bool running = true;

  private:
    Propagation propagation_;
};

} // namespace aocs::simulation
