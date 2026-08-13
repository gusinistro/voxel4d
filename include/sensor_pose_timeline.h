#pragma once

#include <cstddef>
#include <deque>
#include <optional>

#include "sensor_pose.h"
#include "time_types.h"

namespace voxel4d {

/** @brief One world-from-sensor pose associated with a monotonic observation time. */
struct TimedSensorPose {
    TimestampNanoseconds timestamp_nanoseconds{0};
    SensorPose world_from_sensor{};
};

/**
 * @brief Bounded, strictly ordered history of sensor poses.
 *
 * The timeline preserves supplied pose samples. It does not estimate poses,
 * interpolate motion, or synchronize clocks from different sensors.
 */
class SensorPoseTimeline {
   public:
    /** @throws std::invalid_argument when max_samples is zero. */
    explicit SensorPoseTimeline(std::size_t max_samples);

    /**
     * @return false when the timestamp is not strictly newer than the latest sample.
     * @throws std::invalid_argument when pose is invalid.
     */
    [[nodiscard]] bool insert(TimestampNanoseconds timestamp_nanoseconds,
                              const SensorPose& world_from_sensor);

    /** @return The latest retained pose, or std::nullopt when empty. */
    [[nodiscard]] std::optional<TimedSensorPose> latest() const;

    /** @return The newest retained pose at or before the query time, or std::nullopt. */
    [[nodiscard]] std::optional<TimedSensorPose> at_or_before(
        TimestampNanoseconds timestamp_nanoseconds) const;

    [[nodiscard]] std::size_t size() const;
    [[nodiscard]] std::size_t max_samples() const;

   private:
    std::size_t max_samples_;
    std::deque<TimedSensorPose> samples_;
};

}  // namespace voxel4d
