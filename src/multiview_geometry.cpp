#include "multiview_geometry.h"

#include <cmath>
#include <glm/geometric.hpp>
#include <stdexcept>

namespace {

constexpr float kParallelRayThreshold = 1.0e-6F;
constexpr float kMinimumRayDistanceMeters = 1.0e-5F;

bool is_finite(const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

void validate_observation_for_camera(const voxel4d::CalibratedCamera& camera,
                                     const voxel4d::TimedPixelObservation& observation) {
    if (!camera.is_valid() || !observation.is_valid() ||
        observation.camera_id != camera.camera_id ||
        observation.pixel_x >= camera.intrinsics.width_pixels ||
        observation.pixel_y >= camera.intrinsics.height_pixels) {
        throw std::invalid_argument(
            "observation must match a valid calibrated camera and be in bounds");
    }
}

}  // namespace

namespace voxel4d {

glm::vec2 MultiviewGeometry::project_world_to_pixel(const CalibratedCamera& camera,
                                                    const glm::vec3& point_world_meters) {
    if (!camera.is_valid() || !is_finite(point_world_meters)) {
        throw std::invalid_argument("camera and world point must be valid and finite");
    }

    const glm::vec3 point_camera =
        camera.world_from_camera.transform_point_to_sensor(point_world_meters);
    const float depth_meters = -point_camera.z;
    if (!(depth_meters > kMinimumRayDistanceMeters)) {
        throw std::invalid_argument("world point must lie in front of the calibrated camera");
    }

    const float pixel_x = camera.intrinsics.focal_length_x_pixels * point_camera.x / depth_meters +
                          camera.intrinsics.principal_point_x_pixels;
    const float pixel_y = camera.intrinsics.principal_point_y_pixels -
                          camera.intrinsics.focal_length_y_pixels * point_camera.y / depth_meters;
    return glm::vec2(pixel_x, pixel_y);
}

float MultiviewGeometry::reprojection_error_pixels(const CalibratedCamera& camera,
                                                   const TimedPixelObservation& observation,
                                                   const glm::vec3& point_world_meters) {
    const glm::vec2 projection = project_world_to_pixel(camera, point_world_meters);
    const glm::vec2 measurement(static_cast<float>(observation.pixel_x),
                                static_cast<float>(observation.pixel_y));
    return glm::length(projection - measurement);
}

TriangulatedPoint MultiviewGeometry::triangulate_two_view(
    const CalibratedCamera& first_camera, const TimedPixelObservation& first_observation,
    const CalibratedCamera& second_camera, const TimedPixelObservation& second_observation) {
    validate_observation_for_camera(first_camera, first_observation);
    validate_observation_for_camera(second_camera, second_observation);
    if (first_camera.camera_id == second_camera.camera_id) {
        throw std::invalid_argument("two-view triangulation requires distinct cameras");
    }

    const glm::vec3 first_origin = first_camera.world_from_camera.translation_meters;
    const glm::vec3 second_origin = second_camera.world_from_camera.translation_meters;
    const glm::vec3 first_direction =
        first_camera.pixel_to_unit_ray_world(first_observation.pixel_x, first_observation.pixel_y);
    const glm::vec3 second_direction = second_camera.pixel_to_unit_ray_world(
        second_observation.pixel_x, second_observation.pixel_y);
    const glm::vec3 origin_difference = first_origin - second_origin;

    const float first_dot_first = glm::dot(first_direction, first_direction);
    const float direction_dot = glm::dot(first_direction, second_direction);
    const float second_dot_second = glm::dot(second_direction, second_direction);
    const float first_origin_dot = glm::dot(first_direction, origin_difference);
    const float second_origin_dot = glm::dot(second_direction, origin_difference);
    const float denominator = first_dot_first * second_dot_second - direction_dot * direction_dot;
    if (std::abs(denominator) < kParallelRayThreshold) {
        throw std::invalid_argument("camera rays are parallel or nearly parallel");
    }

    const float first_distance =
        (direction_dot * second_origin_dot - second_dot_second * first_origin_dot) / denominator;
    const float second_distance =
        (first_dot_first * second_origin_dot - direction_dot * first_origin_dot) / denominator;
    if (!(first_distance > kMinimumRayDistanceMeters) ||
        !(second_distance > kMinimumRayDistanceMeters)) {
        throw std::invalid_argument("triangulated point lies behind at least one camera");
    }

    const glm::vec3 first_point = first_origin + first_direction * first_distance;
    const glm::vec3 second_point = second_origin + second_direction * second_distance;
    const glm::vec3 midpoint = (first_point + second_point) * 0.5F;
    const float first_error = reprojection_error_pixels(first_camera, first_observation, midpoint);
    const float second_error =
        reprojection_error_pixels(second_camera, second_observation, midpoint);
    return TriangulatedPoint{midpoint, glm::length(first_point - second_point),
                             (first_error + second_error) * 0.5F};
}

}  // namespace voxel4d
