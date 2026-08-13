#pragma once

#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

#include "sensor_pose.h"

namespace voxel4d {

/** @brief One known correspondence between consecutive sensor-space 3D points. */
struct PointCorrespondence3D {
    glm::vec3 previous_point_meters{0.0F};
    glm::vec3 current_point_meters{0.0F};
};

/** @brief Deterministic rigid-motion estimate between two point sets. */
struct VisualOdometryResult {
    bool success{false};
    SensorPose current_from_previous{};
    float root_mean_square_error_meters{0.0F};
    std::size_t correspondence_count{0};
};

/**
 * @brief Estimates rigid 3D motion from synthetic or externally matched point pairs.
 *
 * The estimator uses a least-squares rigid alignment over already established
 * correspondences. Feature extraction, descriptor matching, RANSAC outlier
 * rejection, calibration, and real camera capture are intentionally outside the
 * scope of this CPU baseline.
 */
class VisualOdometryEstimator {
   public:
    /**
     * @return A successful rigid transform from previous to current coordinates,
     *         or an unsuccessful result for insufficient or degenerate input.
     * @throws std::invalid_argument when any correspondence contains non-finite data.
     */
    [[nodiscard]] VisualOdometryResult estimate_rigid_motion(
        const std::vector<PointCorrespondence3D>& correspondences) const;
};

}  // namespace voxel4d
