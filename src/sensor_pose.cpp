#include "sensor_pose.h"

#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include <stdexcept>

namespace {

constexpr float kMinimumQuaternionLengthSquared = 1.0e-12F;

bool is_finite(const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool is_finite(const glm::quat& value) {
    return std::isfinite(value.w) && std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

}  // namespace

namespace voxel4d {

bool SensorPose::is_valid() const {
    return is_finite(translation_meters) && is_finite(world_from_sensor) &&
           glm::dot(world_from_sensor, world_from_sensor) > kMinimumQuaternionLengthSquared;
}

SensorPose SensorPose::normalized() const {
    if (!is_valid()) {
        throw std::invalid_argument("sensor pose must have finite values and a non-zero rotation");
    }

    SensorPose result = *this;
    result.world_from_sensor = glm::normalize(world_from_sensor);
    return result;
}

glm::vec3 SensorPose::transform_point_to_world(const glm::vec3& point_sensor_meters) const {
    const SensorPose unit_pose = normalized();
    return unit_pose.translation_meters + unit_pose.world_from_sensor * point_sensor_meters;
}

glm::vec3 SensorPose::transform_vector_to_world(const glm::vec3& vector_sensor) const {
    return normalized().world_from_sensor * vector_sensor;
}

glm::vec3 SensorPose::transform_point_to_sensor(const glm::vec3& point_world_meters) const {
    return inverse().transform_point_to_world(point_world_meters);
}

glm::vec3 SensorPose::transform_vector_to_sensor(const glm::vec3& vector_world) const {
    return inverse().transform_vector_to_world(vector_world);
}

SensorPose SensorPose::inverse() const {
    const SensorPose unit_pose = normalized();
    SensorPose result{};
    result.world_from_sensor = glm::conjugate(unit_pose.world_from_sensor);
    result.translation_meters = result.world_from_sensor * -unit_pose.translation_meters;
    return result;
}

SensorPose SensorPose::compose(const SensorPose& sensor_from_child) const {
    const SensorPose parent = normalized();
    const SensorPose child = sensor_from_child.normalized();

    SensorPose result{};
    result.world_from_sensor = parent.world_from_sensor * child.world_from_sensor;
    result.translation_meters =
        parent.translation_meters + parent.world_from_sensor * child.translation_meters;
    return result.normalized();
}

}  // namespace voxel4d
