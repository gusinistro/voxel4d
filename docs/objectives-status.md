# Objectives Status Matrix

## Interpretation of completion

Voxel4D has a complete **deterministic foundation PoC**, but it has not completed every capability envisioned during the project discussion. This matrix separates implemented behavior from partially implemented research baselines and unimplemented production capabilities. A row is marked complete only when the public source, tests, and documentation support the claim.

| Historical objective | Current status | Evidence in the repository | Remaining work |
|---|---|---|---|
| Sparse Voxel Octree | Implemented baseline | Bounded `SparseVoxelOctree`, leaf attributes, DDA tests. | Compact storage, streaming, memory pooling, and GPU-resident hierarchy. |
| Pixel-to-voxel reconstruction | Partial recorded-data baseline | RGB-D ray projection, synthetic frames, strict P5/P6 loading, TUM RGB-D PNG loading with explicit axial-depth geometry, calibration binding, and per-frame SVO fusion. | Real capture adapters beyond replayed datasets, depth uncertainty, and dense multiview-only reconstruction. |
| Voxel raytracing | Implemented baseline | Leaf-grid 3D DDA with first-occupied-voxel result and a CPU free-view reference renderer. | Hierarchical skipping, coherent ray batches, and accelerator kernels. |
| Voxel soundtracing | Partial | Direct voxel blocking and geometric propagation time. | Reflections, diffraction, attenuation, impulse responses, and frequency bands. |
| Spherical harmonics | Partial | Real L1 accumulation and directional evaluation. | Higher bands, transport integration, and renderer use. |
| Finite light/sound speed and Doppler | Partial | Acoustic direct-path time, acoustic Doppler, and longitudinal optical helper. | Scene-integrated transient optics/acoustics and material models. |
| 4D temporal reconstruction | Partial | Timestamped SVO and pose histories, per-modality temporal velocity, bounded snapshots, and positive occupied-hit log-odds evidence. | Free-space evidence, interpolation, decay, clock alignment, dynamic reconstruction, and uncertainty. |
| Visual odometry | Partial recorded-data baseline | Rigid motion fit for supplied 3D correspondences; dependency-free RGB-D image gradient features, local patch matching, median displacement filtering, and 3D pose fitting; and a reproducible TUM `freiburg1_xyz` replay over 50 pairs with 100.0% estimates, 0.027330 m median translation error, and 0.893230° median rotation error. | Robust descriptors, RANSAC, photometric calibration, long-trajectory evaluation, loop closure, and live camera capture. |
| Multiple cameras at different resolutions | Partial | Calibrated pinhole contracts, bounded synchronization, two-view triangulation, reprojection, and a two-camera fixture. | Calibration estimation, resolution-aware confidence, robust matching, and larger camera arrays. |
| LiDAR, thermal, radar, and IMU fusion | Partial | Validated synthetic observation envelopes, deterministic attribute fusion, strict recorded-CSV replay, and one-sided occupied-hit log-odds evidence. | Real device adapters, calibration, free-space evidence, time synchronization, and uncertainty-aware probabilistic fusion. |
| Object isolation and persistence | Partial | Label-aware nearest-neighbor tracks, stable identities, centers, extent, and one-step velocity. | Detection/segmentation, track lifecycle, re-identification, occlusion handling, and render-layer controls. |
| Novel free-viewpoint rendering | Partial | Calibrated virtual camera, DDA first-hit reference rendering, PPM output, and serial/parallel CPU execution. | Appearance fusion, splatting/rasterization, shading, anti-aliasing, hole filling, and perceptual validation. |
| AI support | Partial | Explicit provider contract, unavailable state, and deterministic replay of externally supplied labels. | Learned model provider, model I/O, dataset evaluation, privacy controls, and accelerator runtime adapters. |
| CPU, GPU, NPU, APU, and data-center scalability | Partial | CPU serial/parallel rendering, capability report, safe failure propagation, explicit accelerator fallback, and parameterized CPU benchmark measured with two and six workers. | Actual GPU/NPU/APU backends, memory budgeting, distributed execution, and target-device measurements. |
| Real-time production use | Not implemented | Deterministic CPU tests, a small synthetic demo, and a 50-pair offline TUM RGB-D replay. | End-to-end latency control, instrumentation, streaming real data, backpressure, and deployment profiles. |

## Prioritized implementation sequence

The deterministic multiview milestone and first recorded RGB-D replay are now implemented. The next engineering sequence must strengthen the measured baseline without disguising absent dependencies: first expand replay evaluation to longer trajectories and add checked device-specific calibration artifacts; then add robust image correspondences, outlier rejection, and uncertainty-aware fusion; then add learned providers and accelerated kernels behind the existing explicit contracts. Each transition must retain the CPU reference path and reproducible validation.

## Definition of the next completion milestone

The next milestone is complete when the repository measures synchronization and reprojection quality on a documented recorded multiview dataset with supplied calibration, derives robust correspondences rather than relying on local patches alone, reports reconstruction and tracking metrics, and retains the same scenario under the CPU reference path. It must not claim support for a physical sensor, learned model, GPU, NPU, or APU until that adapter or backend is implemented and tested on the relevant target.
