#pragma once

#include <glm/glm.hpp>
#include <vector>

#include "sensor_observation.h"

namespace voxel4d {

/**
 * @brief Produces deterministic RGB-D, LiDAR, radar, thermal, and IMU envelopes.
 *
 * This adapter is a test and demonstration source only. It does not communicate
 * with hardware, estimate calibration, or synchronize external clocks.
 */
class SyntheticSensorAdapter {
   public:
    /**
     * @brief Generates one observation envelope for each supported modality.
     * @throws std::invalid_argument when world_from_rig is invalid.
     */
    [[nodiscard]] std::vector<SensorObservation> generate_frame(
        TimestampNanoseconds timestamp_nanoseconds, const SensorPose& world_from_rig,
        const glm::vec3& object_center_world_meters,
        const glm::vec3& object_velocity_world_meters_per_second) const;
};

}  // namespace voxel4d
