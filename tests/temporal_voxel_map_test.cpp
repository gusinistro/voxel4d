#include "temporal_voxel_map.h"

#include <exception>
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "octree.h"

namespace {

int failures = 0;

void expect(const bool condition, const char* const message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        ++failures;
    }
}

std::shared_ptr<SparseVoxelOctree> make_snapshot() {
    return std::make_shared<SparseVoxelOctree>(glm::vec3(0.0F), 2.0F, 1);
}

}  // namespace

int main() {
    try {
        bool rejected_zero_capacity = false;
        try {
            static_cast<void>(TemporalVoxelMap(0));
        } catch (const std::invalid_argument&) {
            rejected_zero_capacity = true;
        }
        expect(rejected_zero_capacity, "zero retention capacity must be rejected");

        TemporalVoxelMap map(2);
        expect(map.get_max_snapshots() == 2U, "configured retention capacity must be exposed");
        expect(map.get_snapshot_count() == 0U, "a new map must be empty");
        expect(map.get_latest_snapshot() == nullptr,
               "an empty map must not have a latest snapshot");

        const auto first = make_snapshot();
        const auto second = make_snapshot();
        const auto third = make_snapshot();

        expect(map.insert_snapshot(100, first), "first snapshot must be inserted");
        expect(map.get_snapshot_count() == 1U, "first insertion must update the snapshot count");
        expect(map.get_latest_snapshot() == first,
               "first insertion must become the latest snapshot");
        expect(map.get_snapshot_at_or_before(99) == nullptr,
               "a lookup before the first snapshot must return nullptr");
        expect(map.get_snapshot_at_or_before(100) == first,
               "an exact historical lookup must return its snapshot");
        expect(map.get_snapshot_at_or_before(150) == first,
               "a later historical lookup must return the current snapshot");

        expect(!map.insert_snapshot(100, second), "duplicate timestamps must be rejected");
        expect(!map.insert_snapshot(99, second), "out-of-order timestamps must be rejected");
        expect(map.get_snapshot_count() == 1U,
               "rejected timestamp insertions must preserve retained snapshots");
        expect(map.get_latest_snapshot() == first,
               "rejected timestamp insertions must preserve the latest snapshot");

        bool rejected_null_snapshot = false;
        try {
            static_cast<void>(map.insert_snapshot(200, nullptr));
        } catch (const std::invalid_argument&) {
            rejected_null_snapshot = true;
        }
        expect(rejected_null_snapshot, "null snapshots must be rejected");
        expect(map.get_snapshot_count() == 1U,
               "a rejected null snapshot must preserve retained snapshots");

        expect(map.insert_snapshot(200, second), "a newer snapshot must be inserted");
        expect(map.insert_snapshot(300, third), "a later snapshot must be inserted");
        expect(map.get_snapshot_count() == 2U, "retention must keep at most two snapshots");
        expect(map.get_latest_snapshot() == third,
               "the final insertion must become the latest snapshot");
        expect(map.get_snapshot_at_or_before(199) == nullptr,
               "the evicted snapshot must not be returned by historical lookup");
        expect(map.get_snapshot_at_or_before(200) == second,
               "the oldest retained snapshot must remain queryable");
        expect(map.get_snapshot_at_or_before(250) == second,
               "lookup must select the newest snapshot at or before the query");
        expect(map.get_snapshot_at_or_before(300) == third,
               "the newest snapshot must be returned by exact lookup");
    } catch (const std::exception& error) {
        std::cerr << "Unexpected exception: " << error.what() << '\n';
        return 2;
    }

    if (failures != 0) {
        std::cerr << failures << " temporal voxel map expectation(s) failed\n";
        return 1;
    }

    std::cout << "Temporal voxel map tests passed\n";
    return 0;
}
