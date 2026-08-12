#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

#include "octree.h"

/** @brief A sampled RGB-D pixel. Depth is measured along the camera ray. */
struct PixelData {
    int x{0};
    int y{0};
    float depth_meters{0.0F};
    glm::vec3 color{0.0F};
};

/** @brief Pinhole-camera intrinsics and extrinsics used by the PoC. */
struct Camera {
    glm::vec3 position{0.0F};
    glm::vec3 direction{0.0F, 0.0F, -1.0F};
    glm::vec3 up{0.0F, 1.0F, 0.0F};
    float fov_degrees{60.0F};
    int width{0};
    int height{0};
    float near_plane_meters{0.1F};
    float far_plane_meters{100.0F};
};

/** @brief Projects RGB-D pixels into the Sparse Voxel Octree. */
class Voxelizer {
   public:
    explicit Voxelizer(std::shared_ptr<SparseVoxelOctree> octree);

    /** @return Number of in-bounds pixel samples inserted into the Octree. */
    [[nodiscard]] std::size_t voxelize_frame(const Camera& camera,
                                             const std::vector<PixelData>& pixels) const;

    [[nodiscard]] glm::vec3 pixel_to_3d(const Camera& camera, int x, int y,
                                        float depth_meters) const;

    [[nodiscard]] glm::mat4 get_projection_matrix(const Camera& camera) const;
    [[nodiscard]] glm::mat4 get_view_matrix(const Camera& camera) const;

   private:
    std::shared_ptr<SparseVoxelOctree> octree_;

    [[nodiscard]] static glm::vec3 screen_to_ndc(int x, int y, int width, int height);
    static void validate_camera(const Camera& camera);
};
