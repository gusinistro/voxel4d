#include "synthetic_sensor_adapter.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace {

bool is_finite(const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

voxel4d::SensorPose sensor_pose_from_rig(const voxel4d::SensorPose& world_from_rig,
                                         const glm::vec3& rig_to_sensor_meters) {
    voxel4d::SensorPose rig_from_sensor{};
    rig_from_sensor.translation_meters = rig_to_sensor_meters;
    return world_from_rig.compose(rig_from_sensor);
}

voxel4d::SpatialSample make_spatial_sample(const voxel4d::SensorPose& world_from_sensor,
                                           const glm::vec3& object_center_world_meters,
                                           const glm::vec3& object_velocity_world_meters_per_second,
                                           const glm::vec3& color_linear, const float intensity,
                                           const float temperature_kelvin, const float confidence) {
    voxel4d::SpatialSample sample{};
    sample.position_sensor_meters =
        world_from_sensor.transform_point_to_sensor(object_center_world_meters);
    sample.color_linear = color_linear;
    sample.intensity = intensity;
    sample.temperature_kelvin = temperature_kelvin;
    sample.velocity_sensor_meters_per_second =
        world_from_sensor.transform_vector_to_sensor(object_velocity_world_meters_per_second);
    sample.confidence = confidence;
    sample.semantic_label = 1;
    return sample;
}

voxel4d::SensorObservation make_spatial_observation(
    const voxel4d::SensorModality modality, std::string sensor_id,
    const voxel4d::TimestampNanoseconds timestamp_nanoseconds,
    const voxel4d::SensorPose& world_from_sensor, const glm::vec3& object_center_world_meters,
    const glm::vec3& object_velocity_world_meters_per_second, const glm::vec3& color_linear,
    const float intensity, const float temperature_kelvin, const float confidence) {
    voxel4d::SensorObservation observation{};
    observation.modality = modality;
    observation.sensor_id = std::move(sensor_id);
    observation.timestamp_nanoseconds = timestamp_nanoseconds;
    observation.world_from_sensor = world_from_sensor;
    observation.observation_confidence = confidence;
    observation.spatial_samples.push_back(make_spatial_sample(
        world_from_sensor, object_center_world_meters, object_velocity_world_meters_per_second,
        color_linear, intensity, temperature_kelvin, confidence));
    return observation;
}

}  // namespace

namespace voxel4d {

std::vector<SensorObservation> SyntheticSensorAdapter::generate_frame(
    const TimestampNanoseconds timestamp_nanoseconds, const SensorPose& world_from_rig,
    const glm::vec3& object_center_world_meters,
    const glm::vec3& object_velocity_world_meters_per_second) const {
    if (!world_from_rig.is_valid() || !is_finite(object_center_world_meters) ||
        !is_finite(object_velocity_world_meters_per_second)) {
        throw std::invalid_argument("synthetic sensor frame inputs must be finite and pose-valid");
    }

    const SensorPose rgbd_pose = sensor_pose_from_rig(world_from_rig, glm::vec3(0.0F));
    const SensorPose lidar_pose =
        sensor_pose_from_rig(world_from_rig, glm::vec3(0.20F, 0.0F, 0.0F));
    const SensorPose radar_pose =
        sensor_pose_from_rig(world_from_rig, glm::vec3(-0.20F, 0.0F, 0.0F));
    const SensorPose thermal_pose =
        sensor_pose_from_rig(world_from_rig, glm::vec3(0.0F, 0.15F, 0.0F));
    const SensorPose imu_pose = sensor_pose_from_rig(world_from_rig, glm::vec3(0.0F, 0.0F, 0.10F));

    std::vector<SensorObservation> observations;
    observations.reserve(5U);
    observations.push_back(make_spatial_observation(
        SensorModality::kRgbd, "synthetic-rgbd", timestamp_nanoseconds, rgbd_pose,
        object_center_world_meters, object_velocity_world_meters_per_second,
        glm::vec3(0.8F, 0.3F, 0.2F), 0.8F, 296.15F, 0.95F));
    observations.push_back(make_spatial_observation(
        SensorModality::kLidar, "synthetic-lidar", timestamp_nanoseconds, lidar_pose,
        object_center_world_meters, object_velocity_world_meters_per_second, glm::vec3(0.0F), 0.9F,
        0.0F, 0.98F));
    observations.push_back(make_spatial_observation(
        SensorModality::kRadar, "synthetic-radar", timestamp_nanoseconds, radar_pose,
        object_center_world_meters, object_velocity_world_meters_per_second, glm::vec3(0.0F), 0.7F,
        0.0F, 0.80F));
    observations.push_back(make_spatial_observation(
        SensorModality::kThermal, "synthetic-thermal", timestamp_nanoseconds, thermal_pose,
        object_center_world_meters, object_velocity_world_meters_per_second, glm::vec3(0.0F), 0.0F,
        305.15F, 0.90F));

    SensorObservation imu_observation{};
    imu_observation.modality = SensorModality::kImu;
    imu_observation.sensor_id = "synthetic-imu";
    imu_observation.timestamp_nanoseconds = timestamp_nanoseconds;
    imu_observation.world_from_sensor = imu_pose;
    imu_observation.observation_confidence = 0.99F;
    imu_observation.imu_sample = ImuSample{glm::vec3(0.0F), glm::vec3(0.0F, 0.0F, 0.01F)};
    observations.push_back(std::move(imu_observation));

    return observations;
}

}  // namespace voxel4d
