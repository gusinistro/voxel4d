# Temporal Voxel Map Design

## Purpose

This increment adds a minimal, deterministic **time axis** around the existing spatial `SparseVoxelOctree`. A `TemporalVoxelMap` stores discrete, timestamped octree snapshots with a bounded retention policy. It makes frame ordering, timestamp units, and retention behavior explicit without claiming motion estimation, sensor synchronization, temporal interpolation, or dynamic-scene tracking.

The map is designed as a CPU-only C++17 baseline. It deliberately stores one Octree per retained frame so that the current spatial contracts remain unchanged and every frame can be replayed independently.

## Contract

| Item | Definition |
|---|---|
| Timestamp unit | Signed integer nanoseconds in a monotonic timeline. |
| Snapshot | A `std::shared_ptr<SparseVoxelOctree>` paired with one timestamp. |
| Ordering | Timestamps must be strictly increasing; duplicate or earlier timestamps are rejected. |
| Retention | The constructor receives a positive maximum snapshot count. Inserting beyond that count evicts the oldest snapshot. |
| Lookup | `get_snapshot_at_or_before` returns the newest retained snapshot whose timestamp is less than or equal to the query, or `nullptr` if none qualifies. |
| Future lookup | `get_latest_snapshot` returns the newest retained snapshot, or `nullptr` when the map is empty. |
| Threading | This first increment is not thread-safe. Callers own synchronization. |

## Intended workflow

1. A frame source creates an Octree for one observation time.
2. The existing `Voxelizer` fuses one or more observations into that frame Octree.
3. The caller inserts that spatial snapshot into `TemporalVoxelMap` using a strictly increasing timestamp.
4. Consumers query the latest snapshot or the snapshot active at a historical time.

The map does not alter pixel-to-voxel projection or merge observations across time. It provides the deterministic bookkeeping layer required before pose estimation, sensor-time alignment, confidence aging, or motion-aware fusion can be evaluated.

## Validation criteria

The automated unit test must verify all of the following:

| Case | Expected result |
|---|---|
| Positive retention capacity | Construction succeeds. |
| Zero retention capacity | Construction throws `std::invalid_argument`. |
| First snapshot insertion | Succeeds and becomes the latest snapshot. |
| Duplicate timestamp | Rejected without changing stored snapshots. |
| Out-of-order timestamp | Rejected without changing stored snapshots. |
| Historical lookup | Returns the newest snapshot at or before the query time. |
| Earlier-than-first lookup | Returns `nullptr`. |
| Bounded retention | Inserting one additional snapshot evicts only the oldest snapshot. |
| Null snapshot | Rejected with `std::invalid_argument`. |

## Non-goals

This is not a temporal voxel compression format, a rolling-fusion implementation, a visual-odometry system, a multi-sensor time synchronizer, or a scene-flow estimator. Those remain separate future increments and must retain numerical and replayable validation.
