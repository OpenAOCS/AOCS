#include "data/telemetry_writer.hpp"

#include "astro/ijk_keplerian.hpp"
#include "sim_constants.hpp"
#include "simulation/initialization.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace aocs::simulation::data {
namespace {

constexpr std::size_t kTelemetryLineCount = 7;

std::string renderTelemetry(
    const Simulation& simulation,
    bool useColor) {
    const orbitalElements elements = IJKKeplerian::convert_ijk_to_keplerian(
        simulation.state.position_eci_m,
        simulation.state.velocity_eci_m_s);
    const double progress = std::clamp(
        simulation.elapsedTime_s / simulation.simulationLength_s,
        0.0,
        1.0);
    constexpr int progressBarWidth = 30;
    const int progressBarFill =
        static_cast<int>(std::round(progress * progressBarWidth));
    const char* blue = useColor ? "\033[38;2;74;117;158m" : "";
    const char* orange = useColor ? "\033[38;2;204;132;82m" : "";
    const char* reset = useColor ? "\033[0m" : "";

    std::ostringstream display;
    display << std::fixed << std::setprecision(3)
            << blue << "SIMULATION TELEMETRY" << reset << '\n'
            << blue << "Time: " << reset << orange << simulation.elapsedTime_s
            << reset << blue << " / " << reset << orange
            << simulation.simulationLength_s << reset << blue
            << " s | J2000: " << reset << orange
            << simulation.epoch_J2000_s + simulation.elapsedTime_s << reset
            << blue << " s" << reset << '\n'
            << std::setprecision(2)
            << blue << "Orbit: a = " << reset << orange << elements.sma << reset
            << blue << " m  e = " << reset << orange << elements.e << reset
            << blue << "  i = " << reset << orange
            << elements.i * SimConstants::kRadtoDeg << reset << blue << " deg"
            << reset << '\n'
            << blue << "       RAAN = " << reset << orange
            << elements.omega * SimConstants::kRadtoDeg << reset << blue
            << " deg  argp = " << reset << orange
            << elements.argp * SimConstants::kRadtoDeg << reset << blue
            << " deg  nu = " << reset << orange
            << elements.nu * SimConstants::kRadtoDeg << reset << blue << " deg"
            << reset << '\n'
            << std::setprecision(1)
            << blue << std::left << std::setw(20) << "ECI position [m]:"
            << reset << orange << std::right
            << std::setw(14) << simulation.state.position_eci_m.x()
            << std::setw(14) << simulation.state.position_eci_m.y()
            << std::setw(14) << simulation.state.position_eci_m.z()
            << reset << '\n'
            << blue << std::left << std::setw(20) << "ECI velocity [m/s]:"
            << reset << orange << std::right
            << std::setw(14) << simulation.state.velocity_eci_m_s.x()
            << std::setw(14) << simulation.state.velocity_eci_m_s.y()
            << std::setw(14) << simulation.state.velocity_eci_m_s.z()
            << reset << '\n'
            << blue << '[' << reset << orange
            << std::string(progressBarFill, '=') << reset << blue
            << std::string(progressBarWidth - progressBarFill, ' ') << ']'
            << reset << ' ' << orange << std::setprecision(1) << progress * 100.0
            << reset << blue << "%  Simulation Rate: " << reset << orange
            << simulation.elapsedTime_s /
                   std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - simulation.sim_start)
                       .count()
            << reset << blue << "x Real-Time" << reset;
    return display.str();
}

} // namespace

void publishTelemetry(
    const Simulation& simulation,
    bool interactive,
    bool& hasDrawnTelemetry) {
    if (!interactive && simulation.running) {
        return;
    }

    if (interactive && hasDrawnTelemetry) {
        std::cout << "\033[" << kTelemetryLineCount << 'A';
    }
    std::istringstream lines(renderTelemetry(simulation, interactive));
    std::string line;
    while (std::getline(lines, line)) {
        if (interactive) {
            std::cout << "\r\033[2K";
        }
        std::cout << line << '\n';
    }
    std::cout.flush();
    hasDrawnTelemetry = true;
}

} // namespace aocs::simulation::data
