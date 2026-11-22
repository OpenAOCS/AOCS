#include "data/file_writer.hpp"
#include "simulation/initialization.hpp"

namespace aocs::simulation::data {

void writeState(
    Simulation& simulation,
    const sensorMessage& sensors,
    const navigationEstimateMessage& estimate) {
    const Eigen::Vector3d& position = simulation.state.position_eci_m;
    const Eigen::Vector3d& velocity = simulation.state.velocity_eci_m_s;
    simulation.outputFile << simulation.elapsedTime_s << ','
                          << position.x() << ',' << position.y() << ',' << position.z()
                          << ',' << velocity.x() << ',' << velocity.y() << ','
                          << velocity.z() << ','
                          << sensors.GPS_time_s - aocs::ipc::kGpsSecondsAtJ2000 -
                                 simulation.epoch_J2000_s
                          << ',' << sensors.has_new_GPS_measurement
                          << ',' << estimate.time_J2000_s -
                                 simulation.epoch_J2000_s;
    for (double value : estimate.position_ECI_m) {
        simulation.outputFile << ',' << value;
    }
    for (double value : estimate.velocity_ECI_m_s) {
        simulation.outputFile << ',' << value;
    }
    for (double value : estimate.covariance_row_major) {
        simulation.outputFile << ',' << value;
    }
    simulation.outputFile << '\n';
}

} // namespace aocs::simulation::data
