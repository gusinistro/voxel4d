#include "sensor_pose_timeline.h"

#include <stdexcept>

namespace voxel4d {

SensorPoseTimeline::SensorPoseTimeline(const std::size_t max_samples) : max_samples_(max_samples) {
    if (max_samples_ == 0U) {
        throw std::invalid_argument("max_samples must be greater than zero");
    }
}

bool SensorPoseTimeline::insert(const TimestampNanoseconds timestamp_nanoseconds,
                                const SensorPose& world_from_sensor) {
    if (!world_from_sensor.is_valid()) {
        throw std::invalid_argument("world_from_sensor must be a valid rigid pose");
    }
    if (!samples_.empty() && timestamp_nanoseconds <= samples_.back().timestamp_nanoseconds) {
        return false;
    }

    samples_.push_back(TimedSensorPose{timestamp_nanoseconds, world_from_sensor.normalized()});
    if (samples_.size() > max_samples_) {
        samples_.pop_front();
    }
    return true;
}

std::optional<TimedSensorPose> SensorPoseTimeline::latest() const {
    if (samples_.empty()) {
        return std::nullopt;
    }
    return samples_.back();
}

std::optional<TimedSensorPose> SensorPoseTimeline::at_or_before(
    const TimestampNanoseconds timestamp_nanoseconds) const {
    for (auto sample = samples_.rbegin(); sample != samples_.rend(); ++sample) {
        if (sample->timestamp_nanoseconds <= timestamp_nanoseconds) {
            return *sample;
        }
    }
    return std::nullopt;
}

std::size_t SensorPoseTimeline::size() const {
    return samples_.size();
}

std::size_t SensorPoseTimeline::max_samples() const {
    return max_samples_;
}

}  // namespace voxel4d
