#pragma once

#include <array>
#include <cstdint>

struct navigationMessage {
    std::uint32_t valid = 0;
    double time_J2000_s = 0.0;
    std::array<double, 3> position_ECI_m{};
    std::array<double, 3> velocity_ECI_m_s{};
    std::array<double, 36> covariance_row_major{}; // 6 x 6 covariance matrix
};

struct AOCSMemoryRegion {
    navigationMessage navigationEstimate;
};