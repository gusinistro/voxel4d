#pragma once

#include <cstddef>

#include "pnm_image.h"
#include "temporal_voxel_map.h"

namespace voxel4d {

/** @brief Counts produced geometry while integrating one decoded RGB-D frame. */
struct RgbdIntegrationResult {
    std::size_t valid_depth_samples{0U};
    std::size_t inserted_samples{0U};
};

/**
 * @brief CPU reference integration of decoded RGB-D frames into temporal SVO snapshots.
 *
 * Each input frame creates an independent bounded SVO snapshot. TUM-style optical-axis depth
 * is lifted with calibrated pinhole intrinsics before the supplied world-from-camera pose is
 * applied. Invalid or zero depth samples are ignored deterministically.
 */
class RgbdTemporalIntegrator {
   public:
    /** @throws std::invalid_argument for invalid SVO depth or root size. */
    RgbdTemporalIntegrator(std::size_t octree_depth, float root_size_meters);

    /** @throws std::invalid_argument when the frame, pose, map, or timestamp is invalid. */
    [[nodiscard]] RgbdIntegrationResult integrate(const DecodedRgbdFrame& frame,
                                                  const SensorPose& world_from_camera,
                                                  TimestampNanoseconds timestamp_nanoseconds,
                                                  TemporalVoxelMap& temporal_map) const;

   private:
    std::size_t octree_depth_{0U};
    float root_size_meters_{0.0F};
};

}  // namespace voxel4d
