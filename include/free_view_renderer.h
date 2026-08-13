#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

#include "execution_runtime.h"
#include "multiview_calibration.h"
#include "octree.h"

namespace voxel4d {

/** @brief A CPU-generated linear RGB image from a calibrated virtual camera. */
struct RenderedImage {
    int width_pixels{0};
    int height_pixels{0};
    std::vector<glm::vec3> linear_rgb{};

    [[nodiscard]] bool is_valid() const;
    [[nodiscard]] const glm::vec3& pixel(int pixel_x, int pixel_y) const;
};

/**
 * @brief Renders first occupied SVO voxels from an arbitrary calibrated camera pose.
 *
 * Every pixel emits a calibrated DDA ray and uses the first occupied voxel's
 * stored color. This is a deterministic reference renderer, not splatting,
 * photorealistic shading, anti-aliasing, Gaussian rendering, or a GPU renderer.
 */
class FreeViewRenderer {
   public:
    /** @throws std::invalid_argument when octree is null. */
    explicit FreeViewRenderer(std::shared_ptr<SparseVoxelOctree> octree);

    /** @throws std::invalid_argument when camera or max distance is invalid. */
    [[nodiscard]] RenderedImage render(const CalibratedCamera& camera, float max_distance_meters,
                                       const glm::vec3& background_linear = glm::vec3(0.0F),
                                       const ExecutionRuntime& runtime = ExecutionRuntime{}) const;

    /** @return false when the image contract is invalid or the file cannot be written. */
    [[nodiscard]] static bool write_ppm(const RenderedImage& image, const std::string& output_path);

   private:
    std::shared_ptr<SparseVoxelOctree> octree_;
};

}  // namespace voxel4d
