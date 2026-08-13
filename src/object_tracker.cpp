#include "object_tracker.h"

#include <algorithm>
#include <cmath>
#include <glm/geometric.hpp>
#include <glm/gtx/norm.hpp>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

bool is_finite(const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

}  // namespace

namespace voxel4d {

bool ObjectObservation::is_valid() const {
    return is_finite(center_world_meters) && is_finite(half_extent_meters) &&
           half_extent_meters.x >= 0.0F && half_extent_meters.y >= 0.0F &&
           half_extent_meters.z >= 0.0F && semantic_label > 0 && std::isfinite(confidence) &&
           confidence >= 0.0F && confidence <= 1.0F;
}

ObjectTracker::ObjectTracker(const float association_distance_meters)
    : association_distance_meters_(association_distance_meters) {
    if (!std::isfinite(association_distance_meters_) || association_distance_meters_ <= 0.0F) {
        throw std::invalid_argument("association distance must be finite and positive");
    }
}

std::vector<ObjectTrackId> ObjectTracker::update(
    const TimestampNanoseconds timestamp_nanoseconds,
    const std::vector<ObjectObservation>& observations) {
    if (timestamp_nanoseconds < 0 || (last_update_timestamp_nanoseconds_ >= 0 &&
                                      timestamp_nanoseconds < last_update_timestamp_nanoseconds_)) {
        throw std::invalid_argument("object tracker timestamps must be non-negative and monotonic");
    }
    for (const ObjectObservation& observation : observations) {
        if (!observation.is_valid()) {
            throw std::invalid_argument("object observations must satisfy their public contract");
        }
    }

    const float maximum_squared_distance =
        association_distance_meters_ * association_distance_meters_;
    std::vector<bool> used_track_indices(tracks_.size(), false);
    std::vector<ObjectTrackId> assigned_ids;
    assigned_ids.reserve(observations.size());

    for (const ObjectObservation& observation : observations) {
        std::size_t best_track_index = tracks_.size();
        float best_squared_distance = maximum_squared_distance;
        for (std::size_t index = 0U; index < tracks_.size(); ++index) {
            const ObjectTrack& track = tracks_.at(index);
            if (used_track_indices.at(index) ||
                track.semantic_label != observation.semantic_label) {
                continue;
            }
            const float squared_distance =
                glm::length2(observation.center_world_meters - track.center_world_meters);
            if (squared_distance <= best_squared_distance) {
                best_squared_distance = squared_distance;
                best_track_index = index;
            }
        }

        if (best_track_index == tracks_.size()) {
            if (next_track_id_ == std::numeric_limits<ObjectTrackId>::max()) {
                throw std::overflow_error("object track identifier space exhausted");
            }
            tracks_.push_back(ObjectTrack{next_track_id_, timestamp_nanoseconds,
                                          observation.center_world_meters,
                                          observation.half_extent_meters, glm::vec3(0.0F),
                                          observation.semantic_label, observation.confidence, 1U});
            assigned_ids.push_back(next_track_id_);
            ++next_track_id_;
            used_track_indices.push_back(true);
            continue;
        }

        ObjectTrack& track = tracks_.at(best_track_index);
        if (timestamp_nanoseconds > track.last_timestamp_nanoseconds) {
            const float elapsed_seconds =
                static_cast<float>(timestamp_nanoseconds - track.last_timestamp_nanoseconds) *
                1.0e-9F;
            track.velocity_world_meters_per_second =
                (observation.center_world_meters - track.center_world_meters) / elapsed_seconds;
        }
        track.last_timestamp_nanoseconds = timestamp_nanoseconds;
        track.center_world_meters = observation.center_world_meters;
        track.half_extent_meters = observation.half_extent_meters;
        track.confidence = observation.confidence;
        ++track.observation_count;
        used_track_indices.at(best_track_index) = true;
        assigned_ids.push_back(track.id);
    }

    last_update_timestamp_nanoseconds_ = timestamp_nanoseconds;
    return assigned_ids;
}

std::optional<ObjectTrack> ObjectTracker::get_track(const ObjectTrackId id) const {
    const auto iterator = std::find_if(tracks_.begin(), tracks_.end(),
                                       [id](const ObjectTrack& track) { return track.id == id; });
    if (iterator == tracks_.end()) {
        return std::nullopt;
    }
    return *iterator;
}

std::vector<ObjectTrack> ObjectTracker::tracks_for_semantic_label(const int semantic_label) const {
    std::vector<ObjectTrack> matching_tracks;
    for (const ObjectTrack& track : tracks_) {
        if (track.semantic_label == semantic_label) {
            matching_tracks.push_back(track);
        }
    }
    return matching_tracks;
}

const std::vector<ObjectTrack>& ObjectTracker::tracks() const {
    return tracks_;
}

}  // namespace voxel4d
