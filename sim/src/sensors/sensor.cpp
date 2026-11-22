#include "sensors/sensor.hpp"

#include <cmath>
#include <stdexcept>

Sensor::Sensor(
    SensorEventSensor sensor,
    const std::vector<SensorEvent>& events)
    : sensor_(sensor), events_(events) {
    for (const SensorEvent& event : events_) {
        if (!std::isfinite(event.t_eventStart_sim_s) ||
            !std::isfinite(event.t_eventEnd_sim_s) ||
            event.t_eventStart_sim_s < 0.0 ||
            event.t_eventEnd_sim_s <= event.t_eventStart_sim_s) {
            throw std::invalid_argument(
                "Sensor events must have finite, increasing simulation times");
        }
        if (event.type != SensorEventType::fault) {
            throw std::invalid_argument("Unsupported sensor event type");
        }
        if (event.valid != SensorEventValid::Valid &&
            event.valid != SensorEventValid::Invalid) {
            throw std::invalid_argument("Unsupported sensor event validity");
        }
    }
}

bool Sensor::isValidAt(double simulationTime_s) const {
    if (!std::isfinite(simulationTime_s) || simulationTime_s < 0.0) {
        throw std::invalid_argument(
            "Sensor simulation time must be finite and non-negative");
    }

    const SensorEvent* activeEvent = nullptr;
    for (const SensorEvent& event : events_) {
        if (event.sensor == sensor_ &&
            simulationTime_s >= event.t_eventStart_sim_s &&
            simulationTime_s < event.t_eventEnd_sim_s &&
            (!activeEvent ||
             event.t_eventStart_sim_s >= activeEvent->t_eventStart_sim_s)) {
            activeEvent = &event;
        }
    }
    return !activeEvent || activeEvent->valid == SensorEventValid::Valid;
}
