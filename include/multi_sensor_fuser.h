#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "octree.h"
#include "sensor_observation.h"

namespace voxel4d {

/**
 * @brief Deterministic CPU fusion of one validated sensor observation into an SVO.
 *
 * The fuser transforms sensor-local samples to world space, preserves modality
 * provenance, combines confidence monotonically, and updates only attributes
 * supplied by the observation modality. It is not a probabilistic SLAM, TSDF,
 * calibration, clock-synchronization, or learned fusion implementation.
 */
class MultiSensorFuser {
   public:
    /** @throws std::invalid_argument when octree is null. */
    explicit MultiSensorFuser(std::shared_ptr<SparseVoxelOctree> octree);

    /**
     * @return Number of in-bounds spatial samples fused. IMU-only observations return zero.
     * @throws std::invalid_argument when observation violates its public contract.
     */
    [[nodiscard]] std::size_t fuse(const SensorObservation& observation) const;

    /** @return Stable modality bit used in VoxelAttribute::source_modality_mask. */
    [[nodiscard]] static std::uint32_t modality_mask(SensorModality modality);

   private:
    std::shared_ptr<SparseVoxelOctree> octree_;
};

}  // namespace voxel4d
