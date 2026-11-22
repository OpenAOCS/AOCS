#include "simulation/initialization.hpp"

#include "telemetry/file_writer.hpp"

#include <json/reader.h>
#include <json/value.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace aocs::simulation {
namespace {

std::string trim(std::string value) {
    const auto first =
        std::find_if_not(value.begin(), value.end(), [](unsigned char character) { return std::isspace(character); });
    const auto last =
        std::find_if_not(value.rbegin(), value.rend(), [](unsigned char character) { return std::isspace(character); }).base();
    return first < last ? std::string(first, last) : std::string();
}

double parseScenarioNumber(const std::map<std::string, std::string>& fields, const char* name) {
    const auto field = fields.find(name);
    if (field == fields.end()) { throw std::invalid_argument(std::string("Scenario event is missing ") + name); }

    std::size_t parsedLength = 0;
    const double value       = std::stod(field->second, &parsedLength);
    if (parsedLength != field->second.size() || !std::isfinite(value)) {
        throw std::invalid_argument(std::string("Scenario event ") + name + " must be a finite number");
    }
    return value;
}

SensorEvent parseScenarioEvent(const std::map<std::string, std::string>& fields) {
    const auto requiredField = [&fields](const char* name) -> const std::string& {
        const auto field = fields.find(name);
        if (field == fields.end()) { throw std::invalid_argument(std::string("Scenario event is missing ") + name); }
        return field->second;
    };

    if (requiredField("type") != "sensor_fault") { throw std::invalid_argument("Unsupported scenario event type"); }

    SensorEventSensor sensor;
    const std::string& sensorName = requiredField("sensor");
    if (sensorName == "gps") {
        sensor = SensorEventSensor::GPS;
    } else if (sensorName == "imu") {
        sensor = SensorEventSensor::IMU;
    } else if (sensorName == "camera") {
        sensor = SensorEventSensor::Camera;
    } else {
        throw std::invalid_argument("Scenario sensor must be 'gps', 'imu', or 'camera'");
    }

    SensorEventValid validity;
    const std::string& faultType = requiredField("fault_type");
    if (faultType == "invalid") {
        validity = SensorEventValid::Invalid;
    } else if (faultType == "valid") {
        validity = SensorEventValid::Valid;
    } else {
        throw std::invalid_argument("Scenario fault_type must be 'invalid' or 'valid'");
    }

    const double startTime_s = parseScenarioNumber(fields, "simulation_time");
    const double duration_s  = parseScenarioNumber(fields, "duration");
    if (startTime_s < 0.0 || duration_s <= 0.0 || !std::isfinite(startTime_s + duration_s)) {
        throw std::invalid_argument("Scenario event time must be non-negative with positive duration");
    }
    return {sensor, SensorEventType::fault, validity, startTime_s, startTime_s + duration_s};
}

std::vector<SensorEvent> loadScenarioEvents(const std::string& scenarioName) {
    if (scenarioName.empty() || !std::all_of(scenarioName.begin(), scenarioName.end(), [](unsigned char character) {
            return std::isalnum(character) || character == '_' || character == '-';
        })) {
        throw std::invalid_argument("Scenario name contains invalid characters");
    }

    const std::vector<std::string> paths{"sim/scenarios/" + scenarioName + ".yaml", "scenarios/" + scenarioName + ".yaml",
                                         "../../sim/scenarios/" + scenarioName + ".yaml"};
    std::ifstream scenarioFile;
    for (const std::string& path : paths) {
        scenarioFile.open(path);
        if (scenarioFile) { break; }
        scenarioFile.clear();
    }
    if (!scenarioFile) { throw std::runtime_error("Unable to open scenario " + scenarioName + ".yaml"); }

    bool inEvents            = false;
    std::size_t eventsIndent = 0;
    bool foundEvents         = false;
    std::map<std::string, std::string> fields;
    std::vector<SensorEvent> events;
    const auto finishEvent = [&] {
        if (!fields.empty()) {
            events.push_back(parseScenarioEvent(fields));
            fields.clear();
        }
    };

    std::string line;
    while (std::getline(scenarioFile, line)) {
        const std::size_t indent = line.find_first_not_of(" \t");
        std::string content      = trim(line);
        if (content.empty() || content.front() == '#') { continue; }

        if (!inEvents) {
            if (content == "events:") {
                inEvents     = true;
                foundEvents  = true;
                eventsIndent = indent;
            }
            continue;
        }
        if (indent <= eventsIndent) {
            finishEvent();
            inEvents = false;
            continue;
        }

        if (content.rfind("- ", 0) == 0) {
            finishEvent();
            content = trim(content.substr(2));
        }

        const std::size_t separator = content.find(':');
        if (separator == std::string::npos) { throw std::invalid_argument("Malformed scenario event entry: " + content); }
        const std::string key     = trim(content.substr(0, separator));
        std::string value         = trim(content.substr(separator + 1));
        const std::size_t comment = value.find(" #");
        if (comment != std::string::npos) { value = trim(value.substr(0, comment)); }
        if (value.size() >= 2 &&
            ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
            value = value.substr(1, value.size() - 2);
        }
        if (key == "simulation_time" && fields.count(key) != 0) { finishEvent(); }
        fields[key] = value;
    }
    finishEvent();

    if (!foundEvents) { throw std::invalid_argument("Scenario file is missing events"); }
    return events;
}

Simulation::Config readConfiguration(const Json::Value& document) {
    Simulation::Config config;

    const Json::Value simulation = requireMember(document, "simulation", "config");
    if (simulation.isMember("scenario_config")) {
        const Json::Value scenarioName = simulation["scenario_config"];
        if (!scenarioName.isString()) { throw std::invalid_argument("simulation.scenario_config must be a string"); }
        config.sensors.events = loadScenarioEvents(scenarioName.asString());
    }
    const Json::Value initialConditions       = requireMember(simulation, "initial_conditions", "simulation");
    config.initialConditions.position_eci_m   = readVector3(initialConditions, "r0_sc_eci_m", "simulation.initial_conditions");
    config.initialConditions.velocity_eci_m_s = readVector3(initialConditions, "v0_sc_eci_m", "simulation.initial_conditions");
    config.initialConditions.epoch_J2000_s  = readFiniteNumber(initialConditions, "t_J2000_s", "simulation.initial_conditions");

    config.timing.orbitCount                = readUnsigned(simulation, "orbit_count", "simulation");
    config.timing.trueStateRate_Hz          = readFiniteNumber(simulation, "true_state_rate_hz", "simulation");
    config.timing.aocsRate_Hz               = readFiniteNumber(simulation, "aocs_rate_hz", "simulation");
    config.timing.logRate_Hz                = readFiniteNumber(simulation, "log_rate_hz", "simulation");

    const Json::Value spacecraft            = requireMember(document, "spacecraft", "config");
    config.spacecraft.mass_kg               = readFiniteNumber(spacecraft, "mass_kg", "spacecraft");
    const Json::Value sensors               = requireMember(spacecraft, "sensors", "spacecraft");
    const Json::Value gps                   = requireMember(sensors, "gps", "spacecraft.sensors");
    config.sensors.gps.rate_Hz              = readFiniteNumber(gps, "rate_hz", "spacecraft.sensors.gps");
    config.sensors.gps.radialAccuracy_m     = readFiniteNumber(gps, "radial_accuracy_m", "spacecraft.sensors.gps");
    config.sensors.gps.inTrackAccuracy_m    = readFiniteNumber(gps, "intrack_accuracy_m", "spacecraft.sensors.gps");
    config.sensors.gps.crossTrackAccuracy_m = readFiniteNumber(gps, "cross_track_accuracy_m", "spacecraft.sensors.gps");

    const Json::Value environment           = requireMember(document, "environment", "config");
    const Json::Value gravity               = requireMember(environment, "gravity", "environment");
    const Json::Value model                 = requireMember(gravity, "model", "environment.gravity");
    if (!model.isString()) { throw std::invalid_argument("environment.gravity.model must be a string"); }
    if (model.asString() == "point_mass") {
        config.environment.gravityModel = GravityModel::E_PointMass;
    } else if (model.asString() == "egm2008") {
        config.environment.gravityModel  = GravityModel::E_EGM2008;
        config.environment.gravityDegree = readUnsigned(gravity, "degree", "environment.gravity");
        config.environment.gravityOrder  = readUnsigned(gravity, "order", "environment.gravity");
    } else {
        throw std::invalid_argument("environment.gravity.model must be 'point_mass' or 'egm2008'");
    }
    return config;
}

Json::Value loadConfiguration() {
    std::ifstream configFile("config.json");
    if (!configFile) {
        configFile.clear();
        configFile.open("sim/config.json");
    }
    if (!configFile) { throw std::runtime_error("Unable to open config.json or sim/config.json"); }

    Json::Value document;
    Json::Reader reader;
    if (!reader.parse(configFile, document)) { throw std::invalid_argument(reader.getFormattedErrorMessages()); }
    return document;
}

} // namespace

std::unique_ptr<Simulation> initializeSimulation() {
    try {
        auto simulation        = std::make_unique<Simulation>(readConfiguration(loadConfiguration()));
        simulation->outputFile = std::make_unique<data::FileWriter>("output.tsfile");
        return simulation;
    } catch (const std::exception& error) {
        std::cerr << "Unable to initialize simulation: " << error.what() << '\n';
        return nullptr;
    }
}

} // namespace aocs::simulation
