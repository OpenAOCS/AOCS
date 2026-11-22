#include "telemetry/file_writer.hpp"

#include "utils/db_utils.h"
#include "file/write_file.h"
#include "writer/tsfile_table_writer.h"
#include "sim_constants.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace aocs::simulation::data {
namespace {

constexpr char kTableName[] = "simulation";
constexpr char kSimulationId[] = "aocs";
constexpr std::uint32_t kBatchRowCount = 4196;

const std::vector<common::ColumnSchema>& columns() {
    static const std::vector<common::ColumnSchema> tableColumns = [] {
        std::vector<common::ColumnSchema> result;
        result.emplace_back(
            "simulation_id", common::STRING, common::UNCOMPRESSED,
            common::PLAIN, common::ColumnCategory::TAG);

        const auto addDoubleField = [&result](const std::string& name) {
            result.emplace_back(
                name, common::DOUBLE, common::UNCOMPRESSED, common::PLAIN,
                common::ColumnCategory::FIELD);
        };
        addDoubleField("t_sec");
        addDoubleField("r_x_m");
        addDoubleField("r_y_m");
        addDoubleField("r_z_m");
        addDoubleField("v_x_m_s");
        addDoubleField("v_y_m_s");
        addDoubleField("v_z_m_s");
        addDoubleField("gps_time_s");
        result.emplace_back(
            "gps_measurement_updated", common::INT32, common::UNCOMPRESSED,
            common::PLAIN, common::ColumnCategory::FIELD);
        addDoubleField("estimate_time_s");
        addDoubleField("estimate_r_x_m");
        addDoubleField("estimate_r_y_m");
        addDoubleField("estimate_r_z_m");
        addDoubleField("estimate_v_x_m_s");
        addDoubleField("estimate_v_y_m_s");
        addDoubleField("estimate_v_z_m_s");
        for (std::size_t row = 0; row < 6; ++row) {
            for (std::size_t col = 0; col < 6; ++col) {
                addDoubleField(
                    "cov_" + std::to_string(row) + "_" +
                    std::to_string(col));
            }
        }
        return result;
    }();
    return tableColumns;
}

std::vector<std::string> columnNames() {
    std::vector<std::string> result;
    result.reserve(columns().size());
    for (const common::ColumnSchema& column : columns()) {
        result.push_back(column.get_column_name());
    }
    return result;
}

std::vector<common::TSDataType> columnTypes() {
    std::vector<common::TSDataType> result;
    result.reserve(columns().size());
    for (const common::ColumnSchema& column : columns()) {
        result.push_back(column.get_data_type());
    }
    return result;
}

std::vector<common::ColumnCategory> columnCategories() {
    std::vector<common::ColumnCategory> result;
    result.reserve(columns().size());
    for (const common::ColumnSchema& column : columns()) {
        result.push_back(column.get_column_category());
    }
    return result;
}

void requireSuccess(int code, const std::string& action) {
    if (code != 0) {
        throw std::runtime_error(
            action + " failed with TsFile error code " + std::to_string(code));
    }
}

std::int64_t toUnixTimestampNanoseconds(
    double epoch_J2000_s,
    double elapsedTime_s) {
    const long double timestamp_ns =
        (SimConstants::kUnixSecondsAtJ2000 +
         static_cast<long double>(epoch_J2000_s) +
         static_cast<long double>(elapsedTime_s)) *
        SimConstants::kNanosecondsPerSecond;
    const long double roundedTimestamp_ns = std::round(timestamp_ns);
    if (!std::isfinite(timestamp_ns) ||
        roundedTimestamp_ns <
            static_cast<long double>(
                std::numeric_limits<std::int64_t>::min()) ||
        roundedTimestamp_ns >=
            -static_cast<long double>(
                std::numeric_limits<std::int64_t>::min())) {
        throw std::overflow_error(
            "Simulation time cannot be represented as an int64 nanosecond timestamp");
    }
    return static_cast<std::int64_t>(roundedTimestamp_ns);
}

} // namespace

struct FileWriter::Impl {
    explicit Impl(const std::string& path)
        : tableSchema(kTableName, columns()),
          tablet(
              kTableName,
              columnNames(),
              columnTypes(),
              columnCategories(),
              kBatchRowCount) {
        int flags = O_WRONLY | O_CREAT | O_TRUNC;
#ifdef _WIN32
        flags |= O_BINARY;
#endif
        requireSuccess(file.create(path, flags, 0666), "Create TsFile");
        writer =
            std::make_unique<storage::TsFileTableWriter>(&file, &tableSchema);
        requireSuccess(writer->get_error(), "Initialize TsFile writer");
    }

    void writeState(
        double epoch_J2000_s,
        double elapsedTime_s,
        const Eigen::Vector3d& position_eci_m,
        const Eigen::Vector3d& velocity_eci_m_s,
        const sensorMessage& sensors,
        const navigationMessage& estimate) {
        if (closed) {
            throw std::logic_error("Cannot write to a closed TsFile");
        }

        const std::uint32_t row = rowCount;
        requireSuccess(
            tablet.add_timestamp(
                row, toUnixTimestampNanoseconds(epoch_J2000_s, elapsedTime_s)),
            "Write TsFile timestamp");

        const auto addValue = [this, row](
                                  const std::string& column,
                                  auto value) {
            requireSuccess(
                tablet.add_value(row, column, value),
                "Write TsFile column " + column);
        };
        addValue("simulation_id", kSimulationId);
        addValue("t_sec", elapsedTime_s);
        addValue("r_x_m", position_eci_m.x());
        addValue("r_y_m", position_eci_m.y());
        addValue("r_z_m", position_eci_m.z());
        addValue("v_x_m_s", velocity_eci_m_s.x());
        addValue("v_y_m_s", velocity_eci_m_s.y());
        addValue("v_z_m_s", velocity_eci_m_s.z());
        addValue(
            "gps_time_s",
            sensors.GPS_time_s - SimConstants::kGpsSecondsAtJ2000 -
                epoch_J2000_s);
        addValue(
            "gps_measurement_updated",
            static_cast<std::int32_t>(sensors.has_new_GPS_measurement));
        addValue(
            "estimate_time_s", estimate.time_J2000_s - epoch_J2000_s);

        const char* const estimatePositionColumns[] = {
            "estimate_r_x_m", "estimate_r_y_m", "estimate_r_z_m"};
        const char* const estimateVelocityColumns[] = {
            "estimate_v_x_m_s", "estimate_v_y_m_s", "estimate_v_z_m_s"};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            addValue(estimatePositionColumns[axis], estimate.position_ECI_m[axis]);
            addValue(estimateVelocityColumns[axis], estimate.velocity_ECI_m_s[axis]);
        }
        for (std::size_t index = 0;
             index < estimate.covariance_row_major.size();
             ++index) {
            addValue(
                "cov_" + std::to_string(
                             index / 6) +
                    "_" + std::to_string(
                              index % 6),
                estimate.covariance_row_major[index]);
        }

        ++rowCount;
        if (rowCount == kBatchRowCount) {
            flushBatch();
        }
    }

    int close() {
        if (closed) {
            return 0;
        }

        int result = 0;
        if (rowCount != 0) {
            result = writer->write_table(tablet);
            if (result == 0) {
                rowCount = 0;
            }
        }
        const int closeResult = writer->close();
        if (result == 0) {
            result = closeResult;
        }
        if (result == 0) {
            closed = true;
        }
        return result;
    }

private:
    void flushBatch() {
        requireSuccess(writer->write_table(tablet), "Write TsFile batch");
        tablet.reset();
        rowCount = 0;
    }

    storage::WriteFile file;
    storage::TableSchema tableSchema;
    storage::Tablet tablet;
    std::unique_ptr<storage::TsFileTableWriter> writer;
    std::uint32_t rowCount = 0;
    bool closed = false;
};

FileWriter::FileWriter(const std::string& path) {
    requireSuccess(storage::libtsfile_init(), "Initialize TsFile library");
    try {
        impl_ = std::make_unique<Impl>(path);
    } catch (const std::exception&) {
        storage::libtsfile_destroy();
        throw;
    }
}

FileWriter::~FileWriter() {
    if (!impl_) {
        return;
    }
    const int result = impl_->close();
    if (result != 0) {
        std::cerr << "Unable to finalize TsFile (error code " << result << ")\n";
    }
    impl_.reset();
    storage::libtsfile_destroy();
}

void FileWriter::writeState(
    double epoch_J2000_s,
    double elapsedTime_s,
    const Eigen::Vector3d& position_eci_m,
    const Eigen::Vector3d& velocity_eci_m_s,
    const sensorMessage& sensors,
    const navigationMessage& estimate) {
    impl_->writeState(
        epoch_J2000_s,
        elapsedTime_s,
        position_eci_m,
        velocity_eci_m_s,
        sensors,
        estimate);
}

void FileWriter::close() {
    requireSuccess(impl_->close(), "Finalize TsFile");
}

} // namespace aocs::simulation::data
