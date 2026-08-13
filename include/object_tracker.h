#pragma once

#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>
#include <optional>
#include <vector>

#include "time_types.h"

namespace voxel4d {

using ObjectTrackId = std::uint64_t;

/** @brief A spatial observation used to associate a semantic object across frames. */
struct ObjectObservation {
    glm::vec3 center_world_meters{0.0F};
    glm::vec3 half_extent_meters{0.0F};
    int semantic_label{0};
    float confidence{0.0F};

    [[nodiscard]] bool is_valid() const;
};

/** @brief Latest state of a deterministically associated object track. */
struct ObjectTrack {
    ObjectTrackId id{0U};
    TimestampNanoseconds last_timestamp_nanoseconds{0};
    glm::vec3 center_world_meters{0.0F};
    glm::vec3 half_extent_meters{0.0F};
    glm::vec3 velocity_world_meters_per_second{0.0F};
    int semantic_label{0};
    float confidence{0.0F};
    std::size_t observation_count{0U};
};

/**
 * @brief Greedy nearest-neighbor tracker for temporally ordered semantic observations.
 *
 * The tracker preserves stable IDs for matching labels within a configured
 * world-space radius and produces one velocity estimate from consecutive track
 * centers. It is not a detector, segmenter, Kalman filter, multi-hypothesis
 * tracker, re-identification system, or occlusion-complete tracking system.
 */
class ObjectTracker {
   public:
    /** @throws std::invalid_argument when association distance is not finite and positive. */
    explicit ObjectTracker(float association_distance_meters);

    /**
     * @return Track identifiers in the same order as input observations.
     * @throws std::invalid_argument when timestamp is negative/non-monotonic or observations are
     * invalid.
     */
    [[nodiscard]] std::vector<ObjectTrackId> update(
        TimestampNanoseconds timestamp_nanoseconds,
        const std::vector<ObjectObservation>& observations);

    [[nodiscard]] std::optional<ObjectTrack> get_track(ObjectTrackId id) const;
    [[nodiscard]] std::vector<ObjectTrack> tracks_for_semantic_label(int semantic_label) const;
    [[nodiscard]] const std::vector<ObjectTrack>& tracks() const;

   private:
    float association_distance_meters_{0.0F};
    TimestampNanoseconds last_update_timestamp_nanoseconds_{-1};
    ObjectTrackId next_track_id_{1U};
    std::vector<ObjectTrack> tracks_{};
};

}  // namespace voxel4d
