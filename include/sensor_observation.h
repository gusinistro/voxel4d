#pragma once

#include <glm/glm.hpp>
#include <optional>
#include <string>
#include <vector>

#include "sensor_pose.h"
#include "time_types.h"

namespace voxel4d {

/** @brief Supported observation modalities for the deterministic PoC adapters. */
enum class SensorModality {
    kRgbd,
    kLidar,
    kRadar,
    kThermal,
    kImu,
};

/** @brief One sensor-local spatial measurement with modality-agnostic attributes. */
struct SpatialSample {
    glm::vec3 position_sensor_meters{0.0F};
    glm::vec3 color_linear{0.0F};
    float intensity{0.0F};
    float temperature_kelvin{0.0F};
    glm::vec3 velocity_sensor_meters_per_second{0.0F};
    float confidence{0.0F};
    int semantic_label{0};

    [[nodiscard]] bool is_valid() const;
};

/** @brief One inertial sample in the coordinate system of its reporting sensor. */
struct ImuSample {
    glm::vec3 linear_acceleration_meters_per_second_squared{0.0F};
    glm::vec3 angular_velocity_radians_per_second{0.0F};

    [[nodiscard]] bool is_valid() const;
};

/**
 * @brief Time-stamped observation envelope produced by one sensor adapter.
 *
 * Spatial modalities provide one or more `SpatialSample` values. An IMU
 * observation carries exactly one `ImuSample` and no spatial samples. The
 * envelope contains measurement provenance and a world-from-sensor pose but
 * does not imply clock synchronization or cross-sensor calibration estimation.
 */
struct SensorObservation {
    SensorModality modality{SensorModality::kRgbd};
    std::string sensor_id;
    TimestampNanoseconds timestamp_nanoseconds{0};
    SensorPose world_from_sensor{};
    float observation_confidence{0.0F};
    std::vector<SpatialSample> spatial_samples;
    std::optional<ImuSample> imu_sample;

    [[nodiscard]] bool is_valid() const;
};

[[nodiscard]] const char* sensor_modality_name(SensorModality modality);

}  // namespace voxel4d
