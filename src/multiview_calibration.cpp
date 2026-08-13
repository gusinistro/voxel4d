#include "multiview_calibration.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <glm/geometric.hpp>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace {

bool is_finite(const float value) {
    return std::isfinite(value);
}

std::uint64_t negative_magnitude(const voxel4d::TimestampNanoseconds value) {
    return value >= 0 ? 0U : static_cast<std::uint64_t>(-(value + 1)) + 1U;
}

bool timestamp_span_exceeds(const voxel4d::TimestampNanoseconds minimum,
                            const voxel4d::TimestampNanoseconds maximum,
                            const voxel4d::TimestampNanoseconds tolerance) {
    const std::uint64_t allowed_span = static_cast<std::uint64_t>(tolerance);
    if (minimum >= 0 || maximum < 0) {
        const std::uint64_t low =
            minimum >= 0 ? static_cast<std::uint64_t>(minimum) : negative_magnitude(maximum);
        const std::uint64_t high =
            minimum >= 0 ? static_cast<std::uint64_t>(maximum) : negative_magnitude(minimum);
        return high - low > allowed_span;
    }

    const std::uint64_t negative_part = negative_magnitude(minimum);
    const std::uint64_t positive_part = static_cast<std::uint64_t>(maximum);
    if (negative_part > std::numeric_limits<std::uint64_t>::max() - positive_part) {
        return true;
    }
    return negative_part + positive_part > allowed_span;
}

}  // namespace

namespace voxel4d {

bool CameraIntrinsics::is_valid() const {
    return width_pixels > 0 && height_pixels > 0 && focal_length_x_pixels > 0.0F &&
           focal_length_y_pixels > 0.0F && is_finite(focal_length_x_pixels) &&
           is_finite(focal_length_y_pixels) && is_finite(principal_point_x_pixels) &&
           is_finite(principal_point_y_pixels);
}

bool CalibratedCamera::is_valid() const {
    return !camera_id.empty() && intrinsics.is_valid() && world_from_camera.is_valid();
}

glm::vec3 CalibratedCamera::pixel_to_unit_ray_world(const int pixel_x, const int pixel_y) const {
    if (!is_valid() || pixel_x < 0 || pixel_y < 0 || pixel_x >= intrinsics.width_pixels ||
        pixel_y >= intrinsics.height_pixels) {
        throw std::invalid_argument("calibrated camera and pixel must be valid and in bounds");
    }

    const float camera_x = (static_cast<float>(pixel_x) - intrinsics.principal_point_x_pixels) /
                           intrinsics.focal_length_x_pixels;
    const float camera_y = -(static_cast<float>(pixel_y) - intrinsics.principal_point_y_pixels) /
                           intrinsics.focal_length_y_pixels;
    const glm::vec3 ray_camera = glm::normalize(glm::vec3(camera_x, camera_y, -1.0F));
    return glm::normalize(world_from_camera.transform_vector_to_world(ray_camera));
}

bool TimedPixelObservation::is_valid() const {
    return !camera_id.empty() && pixel_x >= 0 && pixel_y >= 0;
}

ObservationSynchronizer::ObservationSynchronizer(const TimestampNanoseconds tolerance_nanoseconds,
                                                 const std::size_t expected_camera_count)
    : tolerance_nanoseconds_(tolerance_nanoseconds), expected_camera_count_(expected_camera_count) {
    if (tolerance_nanoseconds_ < 0 || expected_camera_count_ < 2U) {
        throw std::invalid_argument(
            "synchronizer tolerance must be non-negative and require two cameras");
    }
}

SynchronizedObservationGroup ObservationSynchronizer::synchronize(
    std::vector<TimedPixelObservation> observations) const {
    if (observations.size() != expected_camera_count_) {
        throw std::invalid_argument("observation count does not match expected camera count");
    }

    std::unordered_set<std::string> camera_ids;
    std::vector<TimestampNanoseconds> timestamps;
    timestamps.reserve(observations.size());
    for (const TimedPixelObservation& observation : observations) {
        if (!observation.is_valid() || !camera_ids.insert(observation.camera_id).second) {
            throw std::invalid_argument(
                "observations must be valid and have unique camera identifiers");
        }
        timestamps.push_back(observation.timestamp_nanoseconds);
    }

    std::sort(timestamps.begin(), timestamps.end());
    if (timestamp_span_exceeds(timestamps.front(), timestamps.back(), tolerance_nanoseconds_)) {
        throw std::invalid_argument("observation timestamps exceed synchronization tolerance");
    }

    std::sort(observations.begin(), observations.end(),
              [](const TimedPixelObservation& left, const TimedPixelObservation& right) {
                  return left.camera_id < right.camera_id;
              });
    return SynchronizedObservationGroup{timestamps.at(timestamps.size() / 2U),
                                        std::move(observations)};
}

TimestampNanoseconds ObservationSynchronizer::tolerance_nanoseconds() const {
    return tolerance_nanoseconds_;
}

std::size_t ObservationSynchronizer::expected_camera_count() const {
    return expected_camera_count_;
}

}  // namespace voxel4d
