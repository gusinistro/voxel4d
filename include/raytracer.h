#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

#include "octree.h"

/** @brief A normalized ray in world space. */
struct Ray {
    glm::vec3 origin{0.0F};
    glm::vec3 direction{0.0F};
};

/** @brief Result of tracing one ray through occupied leaf voxels. */
struct RayHitResult {
    bool hit{false};
    glm::vec3 hit_point{0.0F};
    float distance{0.0F};
    std::shared_ptr<OctreeNode> voxel{};
};

/**
 * @brief Ray tracer using a regular-grid DDA at the Octree leaf resolution.
 *
 * The Octree stores attributes sparsely, while the traversal uses the finest
 * leaf-cell size to provide deterministic point queries for this PoC.
 */
class VoxelRaytracer {
   public:
    explicit VoxelRaytracer(std::shared_ptr<SparseVoxelOctree> octree);

    [[nodiscard]] RayHitResult trace_ray(const Ray& ray, float max_distance = 1000.0F) const;
    [[nodiscard]] std::vector<RayHitResult> trace_rays(const std::vector<Ray>& rays,
                                                       float max_distance = 1000.0F) const;

    [[nodiscard]] glm::vec3 calculate_lighting(const glm::vec3& position,
                                               const glm::vec3& normal) const;

   private:
    std::shared_ptr<SparseVoxelOctree> octree_;

    [[nodiscard]] std::vector<std::shared_ptr<OctreeNode>> dda_traverse(const Ray& ray,
                                                                        float max_distance) const;
    [[nodiscard]] bool ray_box_intersect(const Ray& ray, const glm::vec3& box_min,
                                         const glm::vec3& box_max, float& t_near,
                                         float& t_far) const;
};
