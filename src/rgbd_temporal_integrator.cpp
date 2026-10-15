#include "rgbd_temporal_integrator.h"

#include <cmath>
#include <glm/geometric.hpp>
#include <limits>
#include <memory>
#include <stdexcept>

namespace voxel4d {
namespace {

constexpr std::size_t kMaximumOctreeDepth = 30U;

[[nodiscard]] glm::vec3 camera_point(const CalibratedCamera& camera, const int x_pixels,
                                     const int y_pixels, const float depth_meters,
                                     const DepthConvention convention) {
    const CameraIntrinsics& intrinsics = camera.intrinsics;
    const float normalized_x =
        (static_cast<float>(x_pixels) - intrinsics.principal_point_x_pixels) /
        intrinsics.focal_length_x_pixels;
    const float normalized_y =
        -(static_cast<float>(y_pixels) - intrinsics.principal_point_y_pixels) /
        intrinsics.focal_length_y_pixels;
    const glm::vec3 optical_axis_ray(normalized_x, normalized_y, -1.0F);
    if (convention == DepthConvention::kOpticalAxis) {
        return optical_axis_ray * depth_meters;
    }
    return glm::normalize(optical_axis_ray) * depth_meters;
}

[[nodiscard]] glm::vec3 sample_color(const PnmImage& image, const int x_pixels,
                                     const int y_pixels) {
    const float scale = 1.0F / static_cast<float>(image.maximum_value);
    if (image.channel_count == 1) {
        const float value = static_cast<float>(image.sample_at(x_pixels, y_pixels, 0)) * scale;
        return glm::vec3(value);
    }
    return glm::vec3(static_cast<float>(image.sample_at(x_pixels, y_pixels, 0)) * scale,
                     static_cast<float>(image.sample_at(x_pixels, y_pixels, 1)) * scale,
                     static_cast<float>(image.sample_at(x_pixels, y_pixels, 2)) * scale);
}

}  // namespace

RgbdTemporalIntegrator::RgbdTemporalIntegrator(const std::size_t octree_depth,
                                               const float root_size_meters)
    : octree_depth_(octree_depth), root_size_meters_(root_size_meters) {
    if (octree_depth_ > kMaximumOctreeDepth || !std::isfinite(root_size_meters_) ||
        root_size_meters_ <= 0.0F) {
        throw std::invalid_argument("RGB-D integrator geometry configuration is invalid");
    }
}

RgbdIntegrationResult RgbdTemporalIntegrator::integrate(
    const DecodedRgbdFrame& frame, const SensorPose& world_from_camera,
    const TimestampNanoseconds timestamp_nanoseconds, TemporalVoxelMap& temporal_map) const {
    if (!frame.is_valid() || !world_from_camera.is_valid() || timestamp_nanoseconds < 0) {
        throw std::invalid_argument("RGB-D integration inputs must be valid");
    }
    const CalibratedCamera& camera = *frame.recorded_frame.camera;
    auto octree = std::make_shared<SparseVoxelOctree>(glm::vec3(0.0F), root_size_meters_,
                                                      static_cast<int>(octree_depth_));
    RgbdIntegrationResult result{};
    for (int y = 0; y < frame.depth.height_pixels; ++y) {
        for (int x = 0; x < frame.depth.width_pixels; ++x) {
            const float depth_meters = frame.depth_meters_at(x, y);
            if (!std::isfinite(depth_meters) || depth_meters <= 0.0F) {
                continue;
            }
            ++result.valid_depth_samples;
            const glm::vec3 point_camera =
                camera_point(camera, x, y, depth_meters, frame.depth_convention);
            const glm::vec3 point_world = world_from_camera.transform_point_to_world(point_camera);
            VoxelAttribute attribute{};
            attribute.color = sample_color(frame.color, x, y);
            attribute.intensity = depth_meters;
            attribute.temperature = 293.15F;
            attribute.density = 1.0F;
            attribute.confidence = 1.0F;
            attribute.last_observed_position_meters = point_world;
            attribute.last_observed_timestamp_nanoseconds = timestamp_nanoseconds;
            attribute.semantic_label = 1;
            if (octree->insert(point_world, attribute)) {
                ++result.inserted_samples;
            }
        }
    }
    if (!temporal_map.insert_snapshot(timestamp_nanoseconds, std::move(octree))) {
        throw std::invalid_argument("RGB-D integration timestamp must be strictly increasing");
    }
    return result;
}

}  // namespace voxel4d
