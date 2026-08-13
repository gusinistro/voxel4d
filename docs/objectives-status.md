# Objectives Status Matrix

## Interpretation of completion

Voxel4D has a complete **deterministic foundation PoC**, but it has not completed every capability envisioned during the project discussion. This matrix separates implemented behavior from partially implemented research baselines and unimplemented production capabilities. A row is marked complete only when the public source, tests, and documentation support the claim.

| Historical objective | Current status | Evidence in the repository | Remaining work |
|---|---|---|---|
| Sparse Voxel Octree | Implemented baseline | Bounded `SparseVoxelOctree`, leaf attributes, DDA tests. | Compact storage, streaming, memory pooling, and GPU-resident hierarchy. |
| Pixel-to-voxel reconstruction | Implemented synthetic baseline | RGB-D ray projection, synthetic frames, per-frame SVO fusion. | Real calibrated image ingest, depth uncertainty, and dense multiview-only reconstruction. |
| Voxel raytracing | Implemented baseline | Leaf-grid 3D DDA with first-occupied-voxel result and a CPU free-view reference renderer. | Hierarchical skipping, coherent ray batches, and accelerator kernels. |
| Voxel soundtracing | Partial | Direct voxel blocking and geometric propagation time. | Reflections, diffraction, attenuation, impulse responses, and frequency bands. |
| Spherical harmonics | Partial | Real L1 accumulation and directional evaluation. | Higher bands, transport integration, and renderer use. |
| Finite light/sound speed and Doppler | Partial | Acoustic direct-path time, acoustic Doppler, and longitudinal optical helper. | Scene-integrated transient optics/acoustics and material models. |
| 4D temporal reconstruction | Partial | Timestamped SVO and pose histories, per-modality temporal velocity, and bounded snapshots. | Interpolation, decay, clock alignment, dynamic reconstruction, and uncertainty. |
| Visual odometry | Partial | Rigid motion fit for supplied synthetic 3D correspondences. | Image features, matching, outlier rejection, real camera poses, and loop closure. |
| Multiple cameras at different resolutions | Partial | Calibrated pinhole contracts, bounded synchronization, two-view triangulation, reprojection, and a two-camera fixture. | Calibration estimation, resolution-aware confidence, robust matching, and larger camera arrays. |
| LiDAR, thermal, radar, and IMU fusion | Partial | Validated synthetic observation envelopes, deterministic attribute fusion, and strict recorded-CSV replay. | Real device adapters, calibration, time synchronization, and probabilistic fusion. |
| Object isolation and persistence | Partial | Label-aware nearest-neighbor tracks, stable identities, centers, extent, and one-step velocity. | Detection/segmentation, track lifecycle, re-identification, occlusion handling, and render-layer controls. |
| Novel free-viewpoint rendering | Partial | Calibrated virtual camera, DDA first-hit reference rendering, PPM output, and serial/parallel CPU execution. | Appearance fusion, splatting/rasterization, shading, anti-aliasing, hole filling, and perceptual validation. |
| AI support | Partial | Explicit provider contract, unavailable state, and deterministic replay of externally supplied labels. | Learned model provider, model I/O, dataset evaluation, privacy controls, and accelerator runtime adapters. |
| CPU, GPU, NPU, APU, and data-center scalability | Partial | CPU serial/parallel rendering, capability report, safe failure propagation, explicit accelerator fallback, and reproducible CPU benchmark. | Actual GPU/NPU/APU backends, memory budgeting, distributed execution, and target-device measurements. |
| Real-time production use | Not implemented | Deterministic CPU tests and small synthetic demo. | End-to-end latency control, instrumentation, real data, backpressure, and deployment profiles. |

## Prioritized implementation sequence

The deterministic multiview milestone is now implemented. The next engineering sequence must move from test fixtures to measured real-world inputs without disguising absent dependencies: first add checked adapters for recorded datasets and device-specific calibration artifacts; then add robust image correspondences, outlier rejection, and uncertainty-aware fusion; then add learned providers and accelerated kernels behind the existing explicit contracts. Each transition must retain the CPU reference path and reproducible validation.

## Definition of the next completion milestone

The next milestone is complete when the repository can ingest a documented, recorded multiview dataset with supplied calibration; measure synchronization and reprojection quality; derive robust correspondences rather than receive them; report reconstruction and tracking metrics; and retain the same scenario under the CPU reference path. It must not claim support for a physical sensor, learned model, GPU, NPU, or APU until that adapter or backend is implemented and tested on the relevant target.
