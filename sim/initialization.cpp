#include "simulation/initialization.hpp"

#include "data/file_writer.hpp"

#include <json/reader.h>
#include <json/value.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace aocs::simulation {
namespace {

Json::Value requireMember(
    const Json::Value& object,
    const char* name,
    const std::string& context) {
    if (!object.isObject() || !object.isMember(name)) {
        throw std::invalid_argument(context + "." + name + " is required");
    }
    return object[name];
}

double readFiniteNumber(
    const Json::Value& object,
    const char* name,
    const std::string& context) {
    const Json::Value value = requireMember(object, name, context);
    if (!value.isNumeric() || !std::isfinite(value.asDouble())) {
        throw std::invalid_argument(
            context + "." + name + " must be a finite number");
    }
    return value.asDouble();
}

std::uint32_t readUnsigned(
    const Json::Value& object,
    const char* name,
    const std::string& context) {
    const Json::Value value = requireMember(object, name, context);
    if (!value.isUInt()) {
        throw std::invalid_argument(
            context + "." + name + " must be a non-negative integer");
    }
    return value.asUInt();
}

Eigen::Vector3d readVector3(
    const Json::Value& object,
    const char* name,
    const std::string& context) {
    const Json::Value value = requireMember(object, name, context);
    if (!value.isArray() || value.size() != 3 ||
        !value[0].isNumeric() || !value[1].isNumeric() ||
        !value[2].isNumeric()) {
        throw std::invalid_argument(
            context + "." + name + " must be an array of three numbers");
    }

    const Eigen::Vector3d result(
        value[0].asDouble(), value[1].asDouble(), value[2].asDouble());
    if (!result.allFinite()) {
        throw std::invalid_argument(
            context + "." + name + " must contain finite values");
    }
    return result;
}

Simulation::Config readConfiguration(const Json::Value& document) {
    Simulation::Config config;

    const Json::Value simulation = requireMember(
        document, "simulation", "config");
    const Json::Value initialConditions = requireMember(
        simulation, "initial_conditions", "simulation");
    config.initialConditions.position_eci_m = readVector3(
        initialConditions, "r0_sc_eci_m", "simulation.initial_conditions");
    config.initialConditions.velocity_eci_m_s = readVector3(
        initialConditions, "v0_sc_eci_m", "simulation.initial_conditions");
    config.initialConditions.epoch_J2000_s = readFiniteNumber(
        initialConditions, "t_J2000_s", "simulation.initial_conditions");

    config.timing.orbitCount =
        readUnsigned(simulation, "orbit_count", "simulation");
    config.timing.trueStateRate_Hz =
        readFiniteNumber(simulation, "true_state_rate_hz", "simulation");
    config.timing.aocsRate_Hz =
        readFiniteNumber(simulation, "aocs_rate_hz", "simulation");
    config.timing.logRate_Hz =
        readFiniteNumber(simulation, "log_rate_hz", "simulation");

    const Json::Value spacecraft = requireMember(
        document, "spacecraft", "config");
    config.spacecraft.mass_kg =
        readFiniteNumber(spacecraft, "mass_kg", "spacecraft");
    const Json::Value sensors = requireMember(
        spacecraft, "sensors", "spacecraft");
    const Json::Value gps =
        requireMember(sensors, "gps", "spacecraft.sensors");
    config.sensors.gps.rate_Hz =
        readFiniteNumber(gps, "rate_hz", "spacecraft.sensors.gps");
    config.sensors.gps.radialAccuracy_m =
        readFiniteNumber(gps, "radial_accuracy_m", "spacecraft.sensors.gps");
    config.sensors.gps.inTrackAccuracy_m =
        readFiniteNumber(gps, "intrack_accuracy_m", "spacecraft.sensors.gps");
    config.sensors.gps.crossTrackAccuracy_m =
        readFiniteNumber(gps, "cross_track_accuracy_m", "spacecraft.sensors.gps");

    const Json::Value environment =
        requireMember(document, "environment", "config");
    const Json::Value gravity =
        requireMember(environment, "gravity", "environment");
    const Json::Value model =
        requireMember(gravity, "model", "environment.gravity");
    if (!model.isString()) {
        throw std::invalid_argument(
            "environment.gravity.model must be a string");
    }
    if (model.asString() == "point_mass") {
        config.environment.gravityModel = GravityModel::E_PointMass;
    } else if (model.asString() == "egm2008") {
        config.environment.gravityModel = GravityModel::E_EGM2008;
        config.environment.gravityDegree =
            readUnsigned(gravity, "degree", "environment.gravity");
        config.environment.gravityOrder =
            readUnsigned(gravity, "order", "environment.gravity");
    } else {
        throw std::invalid_argument(
            "environment.gravity.model must be 'point_mass' or 'egm2008'");
    }
    return config;
}

Json::Value loadConfiguration() {
    std::ifstream configFile("config.json");
    if (!configFile) {
        configFile.clear();
        configFile.open("sim/config.json");
    }
    if (!configFile) {
        throw std::runtime_error(
            "Unable to open config.json or sim/config.json");
    }

    Json::Value document;
    Json::Reader reader;
    if (!reader.parse(configFile, document)) {
        throw std::invalid_argument(reader.getFormattedErrorMessages());
    }
    return document;
}

} // namespace

std::unique_ptr<Simulation> initializeSimulation() {
    try {
        auto simulation = std::make_unique<Simulation>(
            readConfiguration(loadConfiguration()));
        simulation->outputFile =
            std::make_unique<data::FileWriter>("output.tsfile");
        return simulation;
    } catch (const std::exception& error) {
        std::cerr << "Unable to initialize simulation: "
                  << error.what() << '\n';
        return nullptr;
    }
}

} // namespace aocs::simulation
