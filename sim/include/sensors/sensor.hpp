#pragma once

#include "common/sensor_events.hpp"

#include <vector>

class Sensor {
public:
    Sensor(
        SensorEventSensor sensor,
        const std::vector<SensorEvent>& events);
    virtual ~Sensor() = default;

    bool isValidAt(double simulationTime_s) const;

private:
    SensorEventSensor sensor_;
    std::vector<SensorEvent> events_;
};
