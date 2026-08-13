#pragma once

#include <cstddef>
#include <glm/glm.hpp>
#include <string>
#include <vector>

#include "sensor_pose.h"
#include "time_types.h"

namespace voxel4d {

/** @brief Pinhole intrinsics in pixel units, independent of image resolution. */
struct CameraIntrinsics {
    int width_pixels{0};
    int height_pixels{0};
    float focal_length_x_pixels{0.0F};
    float focal_length_y_pixels{0.0F};
    float principal_point_x_pixels{0.0F};
    float principal_point_y_pixels{0.0F};

    [[nodiscard]] bool is_valid() const;
};

/** @brief A calibrated pinhole camera with a world-from-camera pose. */
struct CalibratedCamera {
    std::string camera_id{};
    CameraIntrinsics intrinsics{};
    SensorPose world_from_camera{};

    [[nodiscard]] bool is_valid() const;

    /**
     * @brief Returns a normalized world-space ray for an in-bounds image pixel.
     *
     * Camera coordinates use +x right, +y up, and -z forward to match the
     * existing Voxel4D camera convention.
     */
    [[nodiscard]] glm::vec3 pixel_to_unit_ray_world(int pixel_x, int pixel_y) const;
};

/** @brief A calibrated pixel observation with a camera-specific timestamp. */
struct TimedPixelObservation {
    std::string camera_id{};
    TimestampNanoseconds timestamp_nanoseconds{0};
    int pixel_x{0};
    int pixel_y{0};

    [[nodiscard]] bool is_valid() const;
};

/** @brief A deterministic group of observations considered simultaneous within a tolerance. */
struct SynchronizedObservationGroup {
    TimestampNanoseconds reference_timestamp_nanoseconds{0};
    std::vector<TimedPixelObservation> observations{};
};

/**
 * @brief Validates and groups one observation from each expected camera.
 *
 * This is a tolerance-based contract for replayed or synthetic data. It does
 * not estimate clock offsets, interpolate sensor samples, or buffer a stream.
 */
class ObservationSynchronizer {
   public:
    /** @throws std::invalid_argument when tolerance is negative or expected count is below two. */
    ObservationSynchronizer(TimestampNanoseconds tolerance_nanoseconds,
                            std::size_t expected_camera_count);

    /**
     * @throws std::invalid_argument when observations are invalid, duplicate a camera,
     *         have an unexpected count, or exceed the configured temporal tolerance.
     */
    [[nodiscard]] SynchronizedObservationGroup synchronize(
        std::vector<TimedPixelObservation> observations) const;

    [[nodiscard]] TimestampNanoseconds tolerance_nanoseconds() const;
    [[nodiscard]] std::size_t expected_camera_count() const;

   private:
    TimestampNanoseconds tolerance_nanoseconds_{0};
    std::size_t expected_camera_count_{0U};
};

}  // namespace voxel4d
