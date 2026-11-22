#pragma once

#include <cstdint>

enum class SensorEventSensor { GPS, IMU, Camera };

enum class SensorEventValid { Valid, Invalid };

enum class SensorEventType { fault };

struct SensorEvent {
    SensorEventSensor sensor;
    SensorEventType type;
    SensorEventValid valid;
    double t_eventStart_sim_s;
    double t_eventEnd_sim_s;
};