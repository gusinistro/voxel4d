#include "object_tracker.h"

#include <stdexcept>
#include <vector>

#include "test_support.h"

namespace {

voxel4d::ObjectObservation make_observation(const glm::vec3& center, const int label,
                                            const float confidence = 0.9F) {
    return voxel4d::ObjectObservation{center, glm::vec3(0.5F), label, confidence};
}

}  // namespace

int main() {
    voxel4d::test::TestContext test;

    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(voxel4d::ObjectTracker(0.0F)); },
        "Object tracker must reject a non-positive association distance");

    voxel4d::ObjectTracker tracker(2.0F);
    const std::vector<voxel4d::ObjectTrackId> first_ids = tracker.update(
        1000000000,
        {make_observation(glm::vec3(0.0F), 1), make_observation(glm::vec3(5.0F, 0.0F, 0.0F), 2)});
    test.expect(first_ids.size() == 2U && first_ids.at(0) != first_ids.at(1),
                "Initial observations must create distinct tracks");

    const std::vector<voxel4d::ObjectTrackId> second_ids =
        tracker.update(2000000000, {make_observation(glm::vec3(1.0F, 0.0F, 0.0F), 1),
                                    make_observation(glm::vec3(5.5F, 0.0F, 0.0F), 2)});
    test.expect(second_ids == first_ids,
                "Nearby observations with matching labels must retain stable track IDs");
    const std::optional<voxel4d::ObjectTrack> first_track = tracker.get_track(first_ids.at(0));
    test.expect(first_track.has_value(), "Created object track must be queryable by identity");
    if (first_track) {
        test.expect_near(first_track->velocity_world_meters_per_second.x, 1.0F, 1.0e-5F,
                         "Track velocity must be estimated from consecutive centers");
        test.expect(first_track->observation_count == 2U,
                    "Matched track must retain its observation history count");
    }

    const std::vector<voxel4d::ObjectTrackId> third_ids =
        tracker.update(3000000000, {make_observation(glm::vec3(1.2F, 0.0F, 0.0F), 2)});
    test.expect(third_ids.at(0) != first_ids.at(0),
                "A different semantic label must not merge into an existing track");
    test.expect(tracker.tracks_for_semantic_label(2).size() == 2U,
                "Semantic layer query must expose separately tracked objects for one label");

    test.expect_throws<std::invalid_argument>(
        [&] { static_cast<void>(tracker.update(2500000000, {})); },
        "Object tracker must reject globally out-of-order timestamps");
    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(tracker.update(4000000000, {make_observation(glm::vec3(0.0F), 0)}));
        },
        "Object tracker must reject an invalid semantic observation");

    return test.failures() == 0 ? 0 : 1;
}
