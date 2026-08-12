#pragma once

#include <string>
#include <vector>

#include "voxelizer.h"

/**
 * @brief Generates deterministic RGB-D observations of a moving sphere.
 *
 * The generator is intentionally simple and exists to exercise the full PoC
 * pipeline without external cameras, sensor drivers, or datasets.
 */
class SyntheticDataGenerator {
   public:
    SyntheticDataGenerator(int width = 640, int height = 480);

    void generate_moving_object_sequence(int num_frames, const std::string& output_directory) const;

    [[nodiscard]] std::vector<PixelData> generate_camera_frame(const Camera& camera,
                                                               const glm::vec3& object_position,
                                                               const glm::vec3& object_velocity,
                                                               float object_radius,
                                                               const glm::vec3& object_color) const;

    [[nodiscard]] std::vector<Camera> generate_camera_setup(int num_cameras) const;

    [[nodiscard]] bool save_frame(const std::vector<PixelData>& pixels,
                                  const std::string& filename) const;
    [[nodiscard]] std::vector<PixelData> load_frame(const std::string& filename) const;

   private:
    int width_;
    int height_;

    [[nodiscard]] static float ray_sphere_intersection(const glm::vec3& ray_origin,
                                                       const glm::vec3& ray_direction,
                                                       const glm::vec3& sphere_center,
                                                       float radius);
};
