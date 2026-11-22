#pragma once

#include "simulation/simulation.hpp"

#include <json/value.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace aocs::simulation {

inline Json::Value requireMember(const Json::Value& object, const char* name, const std::string& context) {
    if (!object.isObject() || !object.isMember(name)) { throw std::invalid_argument(context + "." + name + " is required"); }
    return object[name];
}

inline double readFiniteNumber(const Json::Value& object, const char* name, const std::string& context) {
    const Json::Value value = requireMember(object, name, context);
    if (!value.isNumeric() || !std::isfinite(value.asDouble())) {
        throw std::invalid_argument(context + "." + name + " must be a finite number");
    }
    return value.asDouble();
}

inline std::uint32_t readUnsigned(const Json::Value& object, const char* name, const std::string& context) {
    const Json::Value value = requireMember(object, name, context);
    if (!value.isUInt()) { throw std::invalid_argument(context + "." + name + " must be a non-negative integer"); }
    return value.asUInt();
}

inline Eigen::Vector3d readVector3(const Json::Value& object, const char* name, const std::string& context) {
    const Json::Value value = requireMember(object, name, context);
    if (!value.isArray() || value.size() != 3 || !value[0].isNumeric() || !value[1].isNumeric() || !value[2].isNumeric()) {
        throw std::invalid_argument(context + "." + name + " must be an array of three numbers");
    }

    const Eigen::Vector3d result(value[0].asDouble(), value[1].asDouble(), value[2].asDouble());
    if (!result.allFinite()) { throw std::invalid_argument(context + "." + name + " must contain finite values"); }
    return result;
}

std::unique_ptr<Simulation> initializeSimulation();

} // namespace aocs::simulation
