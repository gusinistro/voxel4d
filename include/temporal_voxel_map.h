#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>

#include "octree.h"

/**
 * @brief Bounded, ordered collection of timestamped spatial voxel snapshots.
 *
 * This CPU-only PoC container preserves discrete Octree snapshots. It does not
 * estimate motion, synchronize sensors, interpolate between frames, or merge
 * evidence across timestamps. Timestamps are monotonic timeline values in
 * nanoseconds and must be inserted in strictly increasing order.
 */
class TemporalVoxelMap {
   public:
    using TimestampNanoseconds = std::int64_t;

    /**
     * @param max_snapshots Maximum number of retained snapshots; must be positive.
     * @throws std::invalid_argument when max_snapshots is zero.
     */
    explicit TemporalVoxelMap(std::size_t max_snapshots);

    /**
     * @brief Stores a spatial snapshot and evicts the oldest snapshot if necessary.
     * @return false when timestamp_nanoseconds is not strictly newer than the latest snapshot.
     * @throws std::invalid_argument when octree is null.
     */
    [[nodiscard]] bool insert_snapshot(TimestampNanoseconds timestamp_nanoseconds,
                                       std::shared_ptr<SparseVoxelOctree> octree);

    /** @return The most recently retained snapshot, or nullptr when the map is empty. */
    [[nodiscard]] std::shared_ptr<const SparseVoxelOctree> get_latest_snapshot() const;

    /**
     * @return The newest retained snapshot at or before timestamp_nanoseconds, or nullptr.
     */
    [[nodiscard]] std::shared_ptr<const SparseVoxelOctree> get_snapshot_at_or_before(
        TimestampNanoseconds timestamp_nanoseconds) const;

    /** @return Number of currently retained snapshots. */
    [[nodiscard]] std::size_t get_snapshot_count() const;

    /** @return Configured upper bound for retained snapshots. */
    [[nodiscard]] std::size_t get_max_snapshots() const;

   private:
    struct Snapshot {
        TimestampNanoseconds timestamp_nanoseconds;
        std::shared_ptr<SparseVoxelOctree> octree;
    };

    std::size_t max_snapshots_;
    std::deque<Snapshot> snapshots_;
};
