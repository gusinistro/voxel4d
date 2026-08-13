#pragma once

#include <glm/glm.hpp>

#include "multiview_calibration.h"

namespace voxel4d {

/** @brief A two-view point estimate with geometric consistency diagnostics. */
struct TriangulatedPoint {
    glm::vec3 position_world_meters{0.0F};
    float ray_separation_meters{0.0F};
    float mean_reprojection_error_pixels{0.0F};
};

/**
 * @brief Deterministic two-view pinhole geometry for calibrated observations.
 *
 * The implementation finds the closest points on the two camera rays and uses
 * their midpoint. It rejects parallel rays and intersections behind either
 * camera. It is a minimal geometry primitive, not bundle adjustment or a
 * robust multi-view reconstruction pipeline.
 */
class MultiviewGeometry {
   public:
    /** @throws std::invalid_argument when calibration, observations, or geometry is invalid. */
    [[nodiscard]] static TriangulatedPoint triangulate_two_view(
        const CalibratedCamera& first_camera, const TimedPixelObservation& first_observation,
        const CalibratedCamera& second_camera, const TimedPixelObservation& second_observation);

    /** @throws std::invalid_argument when point is non-finite or behind the camera. */
    [[nodiscard]] static glm::vec2 project_world_to_pixel(const CalibratedCamera& camera,
                                                          const glm::vec3& point_world_meters);

   private:
    [[nodiscard]] static float reprojection_error_pixels(const CalibratedCamera& camera,
                                                         const TimedPixelObservation& observation,
                                                         const glm::vec3& point_world_meters);
};

}  // namespace voxel4d
