# Changelog

All notable changes to Voxel4D are documented in this file. The project follows the principles of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and intends to use semantic-versioning conventions while it remains pre-1.0.

## [Unreleased]

### Added

- `TemporalVoxelMap`, a bounded CPU-side container of strictly ordered nanosecond-timestamped SVO snapshots with deterministic oldest-snapshot eviction and historical lookup.
- A reusable `Voxel4D::core` CMake target that separates the spatial-temporal implementation from the demonstration executable.
- A CTest unit executable covering temporal retention, historical lookup, ordering rejection, null-snapshot rejection, and input validation.
- An explicit temporal-map design note that separates implemented snapshot bookkeeping from unimplemented synchronization, motion, interpolation, and temporal fusion.

### Changed

- The demonstration now voxelizes each selected frame into an independent Octree and traces and samples the latest retained temporal snapshot.

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
