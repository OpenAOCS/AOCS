#pragma once

#include "aocs_to_sim.hpp"
#include "sim_to_aocs.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string>

namespace aocs::simulation {

namespace data {

class FileWriter {
  public:
    explicit FileWriter(const std::string& path);
    ~FileWriter();

    FileWriter(const FileWriter&)            = delete;
    FileWriter& operator=(const FileWriter&) = delete;

    void writeState(double epoch_J2000_s, double elapsedTime_s, const Eigen::Vector3d& position_eci_m,
                    const Eigen::Vector3d& velocity_eci_m_s, const sensorMessage& sensors, const navigationMessage& estimate);
    void close();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace data
} // namespace aocs::simulation
