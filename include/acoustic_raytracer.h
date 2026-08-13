#pragma once

#include <glm/glm.hpp>
#include <memory>

#include "octree.h"

namespace voxel4d {

/** @brief Result of a direct-path acoustic query through occupied voxels. */
struct AcousticTraceResult {
    bool blocked{false};
    float path_length_meters{0.0F};
    float travel_time_seconds{0.0F};
    float transmission_gain{1.0F};
    std::shared_ptr<OctreeNode> first_blocking_voxel{};
};

/**
 * @brief CPU direct-path sound tracer using the existing leaf-grid voxel DDA.
 *
 * The tracer reports whether the direct segment intersects an occupied voxel.
 * It intentionally excludes reflections, diffraction, reverberation, frequency
 * absorption, and wave-equation simulation.
 */
class AcousticRaytracer {
   public:
    /** @throws std::invalid_argument when octree is null. */
    explicit AcousticRaytracer(std::shared_ptr<SparseVoxelOctree> octree);

    /**
     * @brief Traces the direct path from source to receiver.
     * @throws std::invalid_argument when source and receiver coincide.
     */
    [[nodiscard]] AcousticTraceResult trace_direct_path(
        const glm::vec3& source_position_meters, const glm::vec3& receiver_position_meters) const;

   private:
    std::shared_ptr<SparseVoxelOctree> octree_;
};

}  // namespace voxel4d
