# TUM RGB-D Temporal Reconstruction Implementation Plan

> **For agentic workers:** Implement natively in this session, task-by-task, with tests before production code.

**Goal:** Integrate recorded RGB-D frames into timestamped SVO snapshots using the existing axial-depth geometry and estimated RGB-D odometry poses, producing reproducible per-frame reconstruction metrics.

**Architecture:** Add a small reusable `RgbdTemporalIntegrator` that converts valid decoded RGB-D pixels to world-space voxel attributes using calibrated pinhole intrinsics and a supplied world-from-camera pose. Each frame creates one bounded SVO snapshot in `TemporalVoxelMap`; the TUM CLI estimates relative motion between adjacent frames, composes the camera pose, integrates the current frame, and writes CSV metrics. The implementation remains CPU-only, dependency-light, and explicit about invalid depth and out-of-bounds samples.

**Tech Stack:** C++17, GLM, existing `TumRgbdDataset`, `ImageVisualOdometry`, `SparseVoxelOctree`, `TemporalVoxelMap`, CMake/CTest.

**Spec:** Existing `docs/objectives-status.md`, `docs/recorded-rgbd-input.md`, and `docs/architecture.md`.

## Global Constraints

- Preserve the C++17 and optional-libpng build model.
- Keep warnings as errors and sanitizer-clean execution.
- Do not claim dense production reconstruction, real-time throughput, or robust SLAM.
- Use axial depth as optical-axis Z for TUM frames and reject zero/non-finite depth.
- Retain deterministic CPU behavior and reproducible CSV output.

## Review Focus

- Axial depth must produce the correct camera-space point at off-axis pixels.
- Invalid/zero depth must be skipped rather than inserted as geometry.
- A pose composition must place integrated points in world coordinates consistently.
- Empty/out-of-bounds frames must remain valid with zero inserted samples.
- The CLI must report failed odometry/integration states without silently fabricating poses.

---

### Task 1: Reusable RGB-D temporal integrator

**Files:**
- Create: `include/rgbd_temporal_integrator.h`
- Create: `src/rgbd_temporal_integrator.cpp`
- Test: `tests/rgbd_temporal_integrator_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- `RgbdIntegrationResult { std::size_t valid_depth_samples; std::size_t inserted_samples; }`
- `RgbdTemporalIntegrator(std::size_t octree_depth, float voxel_map_size_meters)`
- `integrate(const DecodedRgbdFrame&, const SensorPose&, TimestampNanoseconds, TemporalVoxelMap&) -> RgbdIntegrationResult`

- [ ] Write failing tests for axial off-axis projection, invalid-depth skipping, and timestamped snapshot insertion.
- [ ] Run the focused test and observe failure because the integrator does not exist.
- [ ] Implement minimal validation, optical-axis pinhole lifting, camera-to-world transform, attribute creation, and snapshot insertion.
- [ ] Run focused and full tests; refactor only while green.
- [ ] Commit `feat: integrate decoded rgbd frames into temporal voxel snapshots`.

### Task 2: TUM temporal reconstruction CLI

**Files:**
- Create: `tools/tum_reconstruction_eval.cpp`
- Modify: `CMakeLists.txt`, `README.md`, `docs/current-capabilities.md`, `docs/objectives-status.md`, `CHANGELOG.md`

**Interfaces:**
- Executable `voxel4d_tum_reconstruction_eval` with `--dataset`, `--output`, `--max-frames`, and optional odometry settings matching the existing evaluator.
- CSV fields include frame timestamp, status, valid depth samples, inserted samples, feature counts, and estimated world pose.

- [ ] Add CLI smoke/help and output-contract test using a tiny synthetic/in-memory fixture where possible.
- [ ] Run the test to observe failure because the executable does not exist.
- [ ] Implement sequential TUM loading, initial identity pose, relative VO composition, per-frame integration, bounded map snapshots, and explicit failure rows.
- [ ] Document that this is offline sparse temporal integration, not dense SLAM.
- [ ] Run build, CTest, sanitizers, format, and CLI help.
- [ ] Commit `feat: add reproducible tum temporal reconstruction evaluation`.

### Final verification

- [ ] Recompute existing TUM evidence without modifying the committed baseline CSV.
- [ ] Run all 22+ tests, ASan/UBSan, format, demo, and new CLI help.
- [ ] Push branch, open PR, and wait for CI/CodeQL before reporting completion.
