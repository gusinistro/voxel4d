#include "voxelizer.h"

#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <stdexcept>

Voxelizer::Voxelizer(std::shared_ptr<SparseVoxelOctree> octree) : octree_(std::move(octree)) {
    if (!octree_) {
        throw std::invalid_argument("octree must not be null");
    }
}

glm::vec3 Voxelizer::screen_to_ndc(int x, int y, int width, int height) {
    return glm::vec3((2.0F * static_cast<float>(x)) / static_cast<float>(width) - 1.0F,
                     1.0F - (2.0F * static_cast<float>(y)) / static_cast<float>(height), 1.0F);
}

void Voxelizer::validate_camera(const Camera& camera) {
    if (camera.width <= 0 || camera.height <= 0) {
        throw std::invalid_argument("camera image dimensions must be positive");
    }
    if (camera.fov_degrees <= 0.0F || camera.fov_degrees >= 180.0F) {
        throw std::invalid_argument("camera fov_degrees must be within (0, 180)");
    }
    if (camera.near_plane_meters <= 0.0F || camera.far_plane_meters <= camera.near_plane_meters) {
        throw std::invalid_argument("camera clipping planes are invalid");
    }
    if (glm::length(camera.direction) == 0.0F || glm::length(camera.up) == 0.0F) {
        throw std::invalid_argument("camera direction and up must be non-zero");
    }
}

glm::mat4 Voxelizer::get_view_matrix(const Camera& camera) const {
    validate_camera(camera);
    return glm::lookAt(camera.position, camera.position + glm::normalize(camera.direction),
                       glm::normalize(camera.up));
}

glm::mat4 Voxelizer::get_projection_matrix(const Camera& camera) const {
    validate_camera(camera);
    const float aspect = static_cast<float>(camera.width) / static_cast<float>(camera.height);
    return glm::perspective(glm::radians(camera.fov_degrees), aspect, camera.near_plane_meters,
                            camera.far_plane_meters);
}

glm::vec3 Voxelizer::pixel_to_3d(const Camera& camera, int x, int y, float depth_meters) const {
    validate_camera(camera);
    if (x < 0 || x >= camera.width || y < 0 || y >= camera.height) {
        throw std::out_of_range("pixel coordinate lies outside camera dimensions");
    }
    if (depth_meters < camera.near_plane_meters || depth_meters > camera.far_plane_meters) {
        throw std::out_of_range("pixel depth lies outside camera clipping planes");
    }

    const glm::vec3 ndc = screen_to_ndc(x, y, camera.width, camera.height);
    const float aspect = static_cast<float>(camera.width) / static_cast<float>(camera.height);
    const float tangent = std::tan(glm::radians(camera.fov_degrees * 0.5F));
    const glm::vec3 direction = glm::normalize(camera.direction);
    const glm::vec3 right = glm::normalize(glm::cross(direction, glm::normalize(camera.up)));
    const glm::vec3 up = glm::normalize(glm::cross(right, direction));
    const glm::vec3 ray_direction =
        glm::normalize(direction + right * ndc.x * aspect * tangent + up * ndc.y * tangent);

    return camera.position + ray_direction * depth_meters;
}

std::size_t Voxelizer::voxelize_frame(const Camera& camera,
                                      const std::vector<PixelData>& pixels) const {
    validate_camera(camera);

    std::size_t inserted_count = 0;
    for (const PixelData& pixel : pixels) {
        const glm::vec3 world_position = pixel_to_3d(camera, pixel.x, pixel.y, pixel.depth_meters);

        VoxelAttribute attribute{};
        attribute.color = pixel.color;
        attribute.intensity = pixel.depth_meters;
        attribute.temperature = 293.15F;
        attribute.velocity = glm::vec3(0.0F);
        attribute.density = 1.0F;
        attribute.semantic_label = 1;

        if (octree_->insert(world_position, attribute)) {
            ++inserted_count;
        }
    }
    return inserted_count;
}
