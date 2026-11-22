#pragma once

#include <array>
#include <cstdint>

struct sensorMessage {
    std::array<double, 3> GPS_position_ECEF_m{};
    std::array<double, 3> GPS_velocity_ECEF_m_s{};
    double GPS_time_J2000_s               = 0.0;
    double aocs_time_J2000_s              = 0.0;
    std::uint32_t has_new_GPS_measurement = 0;
    std::array<double, 36> GPS_covariance_row_major{};
};

struct SimMemoryRegion {
    sensorMessage sensorData;
};