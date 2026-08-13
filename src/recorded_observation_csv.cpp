#include "recorded_observation_csv.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace {

constexpr std::string_view kHeader =
    "modality,sensor_id,timestamp_ns,tx_m,ty_m,tz_m,qw,qx,qy,qz,observation_confidence,"
    "sample_x_m,sample_y_m,sample_z_m,color_r,color_g,color_b,intensity,temperature_k,"
    "velocity_x_mps,velocity_y_mps,velocity_z_mps,sample_confidence,semantic_label,"
    "accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_radps,gyro_y_radps,gyro_z_radps";
constexpr std::size_t kColumnCount = 30U;

std::vector<std::string> split_csv_row(const std::string& row) {
    std::vector<std::string> values;
    std::stringstream stream(row);
    std::string value;
    while (std::getline(stream, value, ',')) {
        values.push_back(value);
    }
    if (!row.empty() && row.back() == ',') {
        values.emplace_back();
    }
    return values;
}

float parse_float(const std::string& value, const std::size_t line_number) {
    std::size_t consumed = 0U;
    try {
        const float parsed = std::stof(value, &consumed);
        if (consumed != value.size() || !std::isfinite(parsed)) {
            throw std::runtime_error("invalid float");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error("invalid finite float at CSV line " + std::to_string(line_number));
    }
}

int parse_int(const std::string& value, const std::size_t line_number) {
    std::size_t consumed = 0U;
    try {
        const long long parsed = std::stoll(value, &consumed);
        if (consumed != value.size() || parsed < std::numeric_limits<int>::min() ||
            parsed > std::numeric_limits<int>::max()) {
            throw std::runtime_error("invalid integer");
        }
        return static_cast<int>(parsed);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid integer at CSV line " + std::to_string(line_number));
    }
}

voxel4d::TimestampNanoseconds parse_timestamp(const std::string& value,
                                              const std::size_t line_number) {
    std::size_t consumed = 0U;
    try {
        const long long parsed = std::stoll(value, &consumed);
        if (consumed != value.size()) {
            throw std::runtime_error("invalid timestamp");
        }
        return static_cast<voxel4d::TimestampNanoseconds>(parsed);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid timestamp at CSV line " + std::to_string(line_number));
    }
}

voxel4d::SensorModality parse_modality(const std::string& value, const std::size_t line_number) {
    const int encoded = parse_int(value, line_number);
    if (encoded < static_cast<int>(voxel4d::SensorModality::kRgbd) ||
        encoded > static_cast<int>(voxel4d::SensorModality::kImu)) {
        throw std::runtime_error("unsupported modality at CSV line " + std::to_string(line_number));
    }
    return static_cast<voxel4d::SensorModality>(encoded);
}

bool same_pose(const voxel4d::SensorPose& left, const voxel4d::SensorPose& right) {
    return left.translation_meters == right.translation_meters &&
           left.world_from_sensor == right.world_from_sensor;
}

bool same_metadata(const voxel4d::SensorObservation& observation,
                   const voxel4d::SensorModality modality, const std::string& sensor_id,
                   const voxel4d::TimestampNanoseconds timestamp_nanoseconds,
                   const voxel4d::SensorPose& pose, const float confidence) {
    return observation.modality == modality && observation.sensor_id == sensor_id &&
           observation.timestamp_nanoseconds == timestamp_nanoseconds &&
           same_pose(observation.world_from_sensor, pose) &&
           observation.observation_confidence == confidence;
}

void write_common(std::ostream& output, const voxel4d::SensorObservation& observation) {
    output << static_cast<int>(observation.modality) << ',' << observation.sensor_id << ','
           << observation.timestamp_nanoseconds << ','
           << observation.world_from_sensor.translation_meters.x << ','
           << observation.world_from_sensor.translation_meters.y << ','
           << observation.world_from_sensor.translation_meters.z << ','
           << observation.world_from_sensor.world_from_sensor.w << ','
           << observation.world_from_sensor.world_from_sensor.x << ','
           << observation.world_from_sensor.world_from_sensor.y << ','
           << observation.world_from_sensor.world_from_sensor.z << ','
           << observation.observation_confidence;
}

void write_spatial_row(std::ostream& output, const voxel4d::SensorObservation& observation,
                       const voxel4d::SpatialSample& sample) {
    write_common(output, observation);
    output << ',' << sample.position_sensor_meters.x << ',' << sample.position_sensor_meters.y
           << ',' << sample.position_sensor_meters.z << ',' << sample.color_linear.r << ','
           << sample.color_linear.g << ',' << sample.color_linear.b << ',' << sample.intensity
           << ',' << sample.temperature_kelvin << ',' << sample.velocity_sensor_meters_per_second.x
           << ',' << sample.velocity_sensor_meters_per_second.y << ','
           << sample.velocity_sensor_meters_per_second.z << ',' << sample.confidence << ','
           << sample.semantic_label << ",0,0,0,0,0,0\n";
}

void write_imu_row(std::ostream& output, const voxel4d::SensorObservation& observation,
                   const voxel4d::ImuSample& sample) {
    write_common(output, observation);
    output << ",0,0,0,0,0,0,0,0,0,0,0,0,0,"
           << sample.linear_acceleration_meters_per_second_squared.x << ','
           << sample.linear_acceleration_meters_per_second_squared.y << ','
           << sample.linear_acceleration_meters_per_second_squared.z << ','
           << sample.angular_velocity_radians_per_second.x << ','
           << sample.angular_velocity_radians_per_second.y << ','
           << sample.angular_velocity_radians_per_second.z << '\n';
}

void append_if_valid(std::vector<voxel4d::SensorObservation>& observations,
                     std::optional<voxel4d::SensorObservation>& current,
                     const std::size_t line_number) {
    if (!current) {
        return;
    }
    if (!current->is_valid()) {
        throw std::runtime_error("invalid reconstructed observation at CSV line " +
                                 std::to_string(line_number));
    }
    observations.push_back(std::move(*current));
    current.reset();
}

}  // namespace

namespace voxel4d {

bool RecordedObservationCsv::write(const std::string& output_path,
                                   const std::vector<SensorObservation>& observations) {
    if (output_path.empty()) {
        return false;
    }

    std::ofstream output(output_path);
    if (!output) {
        return false;
    }
    output << kHeader << '\n';
    output << std::setprecision(9);
    for (const SensorObservation& observation : observations) {
        if (!observation.is_valid() || observation.sensor_id.find(',') != std::string::npos ||
            observation.sensor_id.find('\n') != std::string::npos) {
            return false;
        }
        if (observation.modality == SensorModality::kImu) {
            write_imu_row(output, observation, *observation.imu_sample);
        } else {
            for (const SpatialSample& sample : observation.spatial_samples) {
                write_spatial_row(output, observation, sample);
            }
        }
    }
    return static_cast<bool>(output);
}

std::vector<SensorObservation> RecordedObservationCsv::read(const std::string& input_path) {
    std::ifstream input(input_path);
    if (!input) {
        throw std::runtime_error("unable to open recorded observation CSV");
    }

    std::string row;
    if (!std::getline(input, row) || row != kHeader) {
        throw std::runtime_error("recorded observation CSV has an unexpected header");
    }

    std::vector<SensorObservation> observations;
    std::optional<SensorObservation> current_spatial_observation;
    std::size_t line_number = 1U;
    while (std::getline(input, row)) {
        ++line_number;
        if (row.empty()) {
            throw std::runtime_error("empty row at CSV line " + std::to_string(line_number));
        }
        const std::vector<std::string> columns = split_csv_row(row);
        if (columns.size() != kColumnCount) {
            throw std::runtime_error("unexpected column count at CSV line " +
                                     std::to_string(line_number));
        }

        const SensorModality modality = parse_modality(columns.at(0), line_number);
        const std::string& sensor_id = columns.at(1);
        const TimestampNanoseconds timestamp_nanoseconds =
            parse_timestamp(columns.at(2), line_number);
        SensorPose pose{};
        pose.translation_meters = glm::vec3(parse_float(columns.at(3), line_number),
                                            parse_float(columns.at(4), line_number),
                                            parse_float(columns.at(5), line_number));
        pose.world_from_sensor = glm::quat(
            parse_float(columns.at(6), line_number), parse_float(columns.at(7), line_number),
            parse_float(columns.at(8), line_number), parse_float(columns.at(9), line_number));
        const float observation_confidence = parse_float(columns.at(10), line_number);

        if (modality == SensorModality::kImu) {
            append_if_valid(observations, current_spatial_observation, line_number);
            SensorObservation observation{};
            observation.modality = modality;
            observation.sensor_id = sensor_id;
            observation.timestamp_nanoseconds = timestamp_nanoseconds;
            observation.world_from_sensor = pose;
            observation.observation_confidence = observation_confidence;
            observation.imu_sample = ImuSample{glm::vec3(parse_float(columns.at(24), line_number),
                                                         parse_float(columns.at(25), line_number),
                                                         parse_float(columns.at(26), line_number)),
                                               glm::vec3(parse_float(columns.at(27), line_number),
                                                         parse_float(columns.at(28), line_number),
                                                         parse_float(columns.at(29), line_number))};
            if (!observation.is_valid()) {
                throw std::runtime_error("invalid IMU observation at CSV line " +
                                         std::to_string(line_number));
            }
            observations.push_back(std::move(observation));
            continue;
        }

        if (!current_spatial_observation ||
            !same_metadata(*current_spatial_observation, modality, sensor_id, timestamp_nanoseconds,
                           pose, observation_confidence)) {
            append_if_valid(observations, current_spatial_observation, line_number);
            current_spatial_observation = SensorObservation{};
            current_spatial_observation->modality = modality;
            current_spatial_observation->sensor_id = sensor_id;
            current_spatial_observation->timestamp_nanoseconds = timestamp_nanoseconds;
            current_spatial_observation->world_from_sensor = pose;
            current_spatial_observation->observation_confidence = observation_confidence;
        }

        SpatialSample sample{};
        sample.position_sensor_meters = glm::vec3(parse_float(columns.at(11), line_number),
                                                  parse_float(columns.at(12), line_number),
                                                  parse_float(columns.at(13), line_number));
        sample.color_linear = glm::vec3(parse_float(columns.at(14), line_number),
                                        parse_float(columns.at(15), line_number),
                                        parse_float(columns.at(16), line_number));
        sample.intensity = parse_float(columns.at(17), line_number);
        sample.temperature_kelvin = parse_float(columns.at(18), line_number);
        sample.velocity_sensor_meters_per_second = glm::vec3(
            parse_float(columns.at(19), line_number), parse_float(columns.at(20), line_number),
            parse_float(columns.at(21), line_number));
        sample.confidence = parse_float(columns.at(22), line_number);
        sample.semantic_label = parse_int(columns.at(23), line_number);
        current_spatial_observation->spatial_samples.push_back(sample);
    }
    append_if_valid(observations, current_spatial_observation, line_number + 1U);
    return observations;
}

}  // namespace voxel4d
