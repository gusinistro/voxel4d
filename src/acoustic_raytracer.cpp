#include "acoustic_raytracer.h"

#include <glm/geometric.hpp>
#include <stdexcept>

#include "doppler_simulator.h"
#include "raytracer.h"

namespace {

constexpr float kMinimumPathLengthMeters = 1.0e-5F;

}  // namespace

namespace voxel4d {

AcousticRaytracer::AcousticRaytracer(std::shared_ptr<SparseVoxelOctree> octree)
    : octree_(std::move(octree)) {
    if (!octree_) {
        throw std::invalid_argument("octree must not be null");
    }
}

AcousticTraceResult AcousticRaytracer::trace_direct_path(
    const glm::vec3& source_position_meters, const glm::vec3& receiver_position_meters) const {
    const glm::vec3 path = receiver_position_meters - source_position_meters;
    const float path_length_meters = glm::length(path);
    if (path_length_meters < kMinimumPathLengthMeters) {
        throw std::invalid_argument("source and receiver must be separated by a non-zero distance");
    }

    AcousticTraceResult result{};
    result.path_length_meters = path_length_meters;
    result.travel_time_seconds = path_length_meters / voxel4d::kSpeedOfSoundMetersPerSecond;

    const VoxelRaytracer raytracer(octree_);
    const RayHitResult hit = raytracer.trace_ray(
        Ray{source_position_meters, path / path_length_meters}, path_length_meters);
    result.blocked = hit.hit;
    result.transmission_gain = result.blocked ? 0.0F : 1.0F;
    result.first_blocking_voxel = hit.voxel;
    return result;
}

}  // namespace voxel4d
