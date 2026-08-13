#include "free_view_renderer.h"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <glm/common.hpp>
#include <limits>
#include <stdexcept>

#include "raytracer.h"

namespace {

bool is_finite(const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

unsigned char linear_to_byte(const float value) {
    const float clamped = glm::clamp(value, 0.0F, 1.0F);
    return static_cast<unsigned char>(std::lround(clamped * 255.0F));
}

}  // namespace

namespace voxel4d {

bool RenderedImage::is_valid() const {
    if (width_pixels <= 0 || height_pixels <= 0 ||
        width_pixels > std::numeric_limits<int>::max() / height_pixels) {
        return false;
    }
    return linear_rgb.size() ==
           static_cast<std::size_t>(width_pixels) * static_cast<std::size_t>(height_pixels);
}

const glm::vec3& RenderedImage::pixel(const int pixel_x, const int pixel_y) const {
    if (!is_valid() || pixel_x < 0 || pixel_y < 0 || pixel_x >= width_pixels ||
        pixel_y >= height_pixels) {
        throw std::out_of_range("rendered image pixel is out of bounds");
    }
    const std::size_t index =
        static_cast<std::size_t>(pixel_y) * static_cast<std::size_t>(width_pixels) +
        static_cast<std::size_t>(pixel_x);
    return linear_rgb.at(index);
}

FreeViewRenderer::FreeViewRenderer(std::shared_ptr<SparseVoxelOctree> octree)
    : octree_(std::move(octree)) {
    if (!octree_) {
        throw std::invalid_argument("octree must not be null");
    }
}

RenderedImage FreeViewRenderer::render(const CalibratedCamera& camera,
                                       const float max_distance_meters,
                                       const glm::vec3& background_linear,
                                       const ExecutionRuntime& runtime) const {
    if (!camera.is_valid() || !std::isfinite(max_distance_meters) || max_distance_meters <= 0.0F ||
        !is_finite(background_linear)) {
        throw std::invalid_argument("camera, render distance, and background must be valid");
    }

    const int width = camera.intrinsics.width_pixels;
    const int height = camera.intrinsics.height_pixels;
    if (width > std::numeric_limits<int>::max() / height) {
        throw std::invalid_argument("rendered image dimensions overflow indexable range");
    }
    RenderedImage image{
        width, height,
        std::vector<glm::vec3>(static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
                               background_linear)};
    const VoxelRaytracer raytracer(octree_);
    runtime.for_each_index(image.linear_rgb.size(), [&](const std::size_t index) {
        const int pixel_x = static_cast<int>(index % static_cast<std::size_t>(width));
        const int pixel_y = static_cast<int>(index / static_cast<std::size_t>(width));
        const glm::vec3 direction = camera.pixel_to_unit_ray_world(pixel_x, pixel_y);
        const RayHitResult hit = raytracer.trace_ray(
            Ray{camera.world_from_camera.translation_meters, direction}, max_distance_meters);
        if (hit.hit && hit.voxel) {
            image.linear_rgb.at(index) = hit.voxel->attribute.color;
        }
    });
    return image;
}

bool FreeViewRenderer::write_ppm(const RenderedImage& image, const std::string& output_path) {
    if (!image.is_valid() || output_path.empty()) {
        return false;
    }

    std::ofstream output(output_path, std::ios::binary);
    if (!output) {
        return false;
    }
    output << "P6\n" << image.width_pixels << ' ' << image.height_pixels << "\n255\n";
    for (const glm::vec3& color : image.linear_rgb) {
        const unsigned char bytes[] = {linear_to_byte(color.r), linear_to_byte(color.g),
                                       linear_to_byte(color.b)};
        output.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
    }
    return static_cast<bool>(output);
}

}  // namespace voxel4d
