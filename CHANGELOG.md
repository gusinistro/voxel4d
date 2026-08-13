# Changelog

All notable changes to Voxel4D are documented in this file. The project follows the principles of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and intends to use semantic-versioning conventions while it remains pre-1.0.

## [Unreleased]

### Added

- `TemporalVoxelMap`, a bounded CPU-side container of strictly ordered nanosecond-timestamped SVO snapshots with deterministic oldest-snapshot eviction and historical lookup.
- A reusable `Voxel4D::core` CMake target that separates the spatial-temporal implementation from the demonstration executable.
- A CTest unit executable covering temporal retention, historical lookup, ordering rejection, null-snapshot rejection, and input validation.
- An explicit temporal-map design note that separates implemented snapshot bookkeeping from unimplemented synchronization, motion, interpolation, and temporal fusion.
- Validated `SensorPose` and `SensorPoseTimeline` contracts with rigid transforms, bounded timestamped pose history, and sensor-space/world-space conversion.
- `VisualOdometryEstimator`, a deterministic least-squares rigid alignment baseline for supplied synthetic 3D correspondences.
- Common `SensorObservation` envelopes and deterministic adapters for synthetic RGB-D, LiDAR, radar, thermal, and IMU data.
- `MultiSensorFuser` with world-space sample transformation, confidence, modality provenance, timestamps, RGB-D color, LiDAR/radar intensity, radar velocity, and thermal temperature attributes.
- `AcousticRaytracer` for direct voxel-blocking queries and geometric sound travel time.
- `SphericalHarmonicsL1` for real first-order directional radiance accumulation and evaluation.
- `ExecutionRuntime` with CPU serial, CPU parallel, explicit capabilities, worker-failure propagation, and visible fallback contracts for future GPU, NPU, and APU backends.
- Calibrated multiview camera contracts, bounded observation synchronization, two-view ray triangulation, and pinhole reprojection diagnostics.
- `ObjectTracker` for deterministic semantic-label-aware object identity, state, and one-step velocity across timestamps.
- Per-modality temporal voxel velocity derived from ordered sample positions without replacing radar-measured velocity.
- `FreeViewRenderer`, a CPU DDA first-hit calibrated virtual-camera renderer with PPM output and serial/parallel execution support.
- `RecordedObservationCsv` for strict portable replay of recorded spatial and IMU observation envelopes.
- Optional semantic-inference provider contracts, including explicit unavailable and replayed-label providers without a bundled learned model.
- `voxel4d_render_benchmark`, a reproducible fixed-workload CPU serial/parallel rendering benchmark.
- Unit coverage for spatial traversal, Doppler, sensor poses, rigid odometry, sensor observations, multisensor fusion, acoustic visibility, spherical harmonics, execution runtime behavior, multiview geometry, object tracking, temporal velocity, free-view rendering, CSV replay, and semantic interfaces.

### Changed

- The demonstration now executes a twelve-step integrated scenario covering calibrated synchronization and triangulation, track identity, strict observation replay, optional labels, free-view output, and CPU timing in addition to temporal fusion, odometry, DDA, acoustics, lighting, and Doppler.
- `VoxelAttribute` now records confidence, contributing modality bitmask, last observed position, timestamp, semantic label, and temporal velocity in addition to spatial appearance and radar-motion fields.

## [0.1.0] - 2026-08-11

### Added

- A deterministic two-camera synthetic RGB-D sequence generator.
- Pixel-to-voxel fusion into a bounded Sparse Voxel Octree.
- Leaf-grid DDA ray traversal with first-occupied-cell hit reporting.
- Classical acoustic Doppler sampling and a longitudinal relativistic optical Doppler helper.
- CMake build configuration, CTest smoke test, warning-as-error option, and optional AddressSanitizer/UndefinedBehaviorSanitizer support.
- Open-source project policies: MIT license, contribution guide, code of conduct, security policy, governance note, citation metadata, and GitHub templates.

### Changed

- Standardized source code, API documentation, console output, and primary project documentation in English.
- Added explicit SI-style unit names in public APIs and input validation across camera, Octree, voxelization, CSV, ray, and Doppler paths.
- Replaced fixed-step ray marching with a discrete DDA implementation at the finest leaf resolution.
- Reclassified conceptual research claims as future directions when they are not implemented in this PoC.

### Fixed

- Corrected RGB-D camera-ray reconstruction so fusion follows the same normalized ray convention as synthetic data generation.
- Prevented uninitialized voxel attributes from causing false ray intersections.
- Rejected out-of-bounds Octree searches and insertions.
- Added safe handling for zero ray-direction components during slab intersection.
- Removed broken references to absent assets and documents from the README.

[0.1.0]: https://github.com/gusinistro/voxel4d/releases/tag/v0.1.0
