#include "simulation/simulation.hpp"

#include <algorithm>

namespace aocs::simulation {

Simulation::Sensors::GPS::GPS(const Config& config, const std::vector<SensorEvent>& events) : model_(config, events) {}

sensorMessage Simulation::Sensors::GPS::measure(const SpacecraftState& state, double epoch_J2000_s, double simulationTime_s) {
    const GPS_SensorMsg measurement =
        model_.measure(state.position_eci_m, state.velocity_eci_m_s, epoch_J2000_s, simulationTime_s);

    sensorMessage message;
    message.GPS_time_J2000_s  = epoch_J2000_s;
    message.aocs_time_J2000_s = epoch_J2000_s;
    if (!measurement.isValid) { return message; }

    message.GPS_position_ECEF_m   = {measurement.r_sc_ecef_m.x(), measurement.r_sc_ecef_m.y(), measurement.r_sc_ecef_m.z()};
    message.GPS_velocity_ECEF_m_s = {measurement.v_sc_ecef_m_s.x(), measurement.v_sc_ecef_m_s.y(),
                                     measurement.v_sc_ecef_m_s.z()};
    std::copy(measurement.covariance_ecef.data(), measurement.covariance_ecef.data() + measurement.covariance_ecef.size(),
              message.GPS_covariance_row_major.begin());
    message.has_new_GPS_measurement = 1;
    return message;
}

} // namespace aocs::simulation
