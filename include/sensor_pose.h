#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace voxel4d {

/**
 * @brief Rigid transform from a sensor coordinate system into world coordinates.
 *
 * Translation is expressed in meters. Rotation is a normalized world-from-sensor
 * quaternion. The type is intentionally independent of a particular sensor or
 * estimator so it can be used by RGB-D, LiDAR, radar, thermal, and IMU adapters.
 */
struct SensorPose {
    glm::vec3 translation_meters{0.0F};
    glm::quat world_from_sensor{1.0F, 0.0F, 0.0F, 0.0F};

    /** @return true when all fields are finite and the quaternion has usable length. */
    [[nodiscard]] bool is_valid() const;

    /** @return A copy with a unit quaternion. Throws when the pose is invalid. */
    [[nodiscard]] SensorPose normalized() const;

    /** @brief Converts a sensor-space point to world coordinates. */
    [[nodiscard]] glm::vec3 transform_point_to_world(const glm::vec3& point_sensor_meters) const;

    /** @brief Converts a sensor-space direction or velocity to world coordinates. */
    [[nodiscard]] glm::vec3 transform_vector_to_world(const glm::vec3& vector_sensor) const;

    /** @brief Converts a world-space point to sensor coordinates. */
    [[nodiscard]] glm::vec3 transform_point_to_sensor(const glm::vec3& point_world_meters) const;

    /** @brief Converts a world-space direction or velocity to sensor coordinates. */
    [[nodiscard]] glm::vec3 transform_vector_to_sensor(const glm::vec3& vector_world) const;

    /** @return The transform that maps world coordinates back into sensor coordinates. */
    [[nodiscard]] SensorPose inverse() const;

    /** @brief Composes this world-from-sensor pose with sensor-from-child. */
    [[nodiscard]] SensorPose compose(const SensorPose& sensor_from_child) const;
};

}  // namespace voxel4d
