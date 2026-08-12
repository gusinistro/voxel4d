#include "temporal_voxel_map.h"

#include <stdexcept>

TemporalVoxelMap::TemporalVoxelMap(const std::size_t max_snapshots)
    : max_snapshots_(max_snapshots) {
    if (max_snapshots_ == 0U) {
        throw std::invalid_argument("max_snapshots must be greater than zero");
    }
}

bool TemporalVoxelMap::insert_snapshot(const TimestampNanoseconds timestamp_nanoseconds,
                                       std::shared_ptr<SparseVoxelOctree> octree) {
    if (!octree) {
        throw std::invalid_argument("octree must not be null");
    }

    if (!snapshots_.empty() && timestamp_nanoseconds <= snapshots_.back().timestamp_nanoseconds) {
        return false;
    }

    snapshots_.push_back(Snapshot{timestamp_nanoseconds, std::move(octree)});
    if (snapshots_.size() > max_snapshots_) {
        snapshots_.pop_front();
    }
    return true;
}

std::shared_ptr<const SparseVoxelOctree> TemporalVoxelMap::get_latest_snapshot() const {
    if (snapshots_.empty()) {
        return nullptr;
    }
    return snapshots_.back().octree;
}

std::shared_ptr<const SparseVoxelOctree> TemporalVoxelMap::get_snapshot_at_or_before(
    const TimestampNanoseconds timestamp_nanoseconds) const {
    for (auto snapshot = snapshots_.rbegin(); snapshot != snapshots_.rend(); ++snapshot) {
        if (snapshot->timestamp_nanoseconds <= timestamp_nanoseconds) {
            return snapshot->octree;
        }
    }
    return nullptr;
}

std::size_t TemporalVoxelMap::get_snapshot_count() const {
    return snapshots_.size();
}

std::size_t TemporalVoxelMap::get_max_snapshots() const {
    return max_snapshots_;
}
