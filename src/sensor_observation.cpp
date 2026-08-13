#include "sensor_observation.h"

#include <cmath>

namespace {

bool is_finite(const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool is_unit_interval(const float value) {
    return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
}

}  // namespace

namespace voxel4d {

bool SpatialSample::is_valid() const {
    return is_finite(position_sensor_meters) && is_finite(color_linear) &&
           std::isfinite(intensity) && std::isfinite(temperature_kelvin) &&
           is_finite(velocity_sensor_meters_per_second) && is_unit_interval(confidence);
}

bool ImuSample::is_valid() const {
    return is_finite(linear_acceleration_meters_per_second_squared) &&
           is_finite(angular_velocity_radians_per_second);
}

bool SensorObservation::is_valid() const {
    if (sensor_id.empty() || !world_from_sensor.is_valid() ||
        !is_unit_interval(observation_confidence)) {
        return false;
    }

    for (const SpatialSample& sample : spatial_samples) {
        if (!sample.is_valid()) {
            return false;
        }
    }

    switch (modality) {
        case SensorModality::kRgbd:
        case SensorModality::kLidar:
        case SensorModality::kRadar:
        case SensorModality::kThermal:
            return !spatial_samples.empty() && !imu_sample.has_value();
        case SensorModality::kImu:
            return spatial_samples.empty() && imu_sample.has_value() && imu_sample->is_valid();
    }
    return false;
}

const char* sensor_modality_name(const SensorModality modality) {
    switch (modality) {
        case SensorModality::kRgbd:
            return "RGB-D";
        case SensorModality::kLidar:
            return "LiDAR";
        case SensorModality::kRadar:
            return "radar";
        case SensorModality::kThermal:
            return "thermal";
        case SensorModality::kImu:
            return "IMU";
    }
    return "unknown";
}

}  // namespace voxel4d
