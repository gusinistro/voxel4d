# Current Capabilities and Hardware Path

## Implemented baseline

Voxel4D now provides a deterministic, CPU-first research pipeline built from composable C++17 modules. Every capability in this document is exercised by automated tests or by the end-to-end demonstration. The project remains a proof of concept and does not claim production readiness.

| Area | Implemented behavior | Explicit boundary |
|---|---|---|
| Spatial representation | Bounded Sparse Voxel Octree with leaf occupancy, confidence, modality provenance, and last-observed timestamp. | The Octree allocates sibling children on refinement and is not a compact, GPU-resident SVO. |
| Time | Bounded, strictly ordered SVO snapshots and bounded sensor-pose histories using nanosecond timestamps. | No interpolation, sensor clock alignment, temporal decay, or long-duration storage. |
| RGB-D | Deterministic synthetic RGB-D generation, CSV replay, camera-ray projection, and voxel insertion. | No physical camera capture, calibration solve, depth denoising, or photometric uncertainty model. |
| Visual odometry | Least-squares rigid alignment of supplied, finite 3D correspondences. | No feature extraction, image matching, RANSAC, loop closure, or real-camera odometry. |
| Multisensor data | Validated synthetic envelopes for RGB-D, LiDAR, radar, thermal, and IMU with poses, timestamps, units, confidence, and sensor identifiers. | No device drivers, external clock synchronization, extrinsic calibration estimation, or real sensor data ingestion. |
| Fusion | World-space insertion with per-voxel confidence, modality bitmask, timestamp, RGB-D color, LiDAR/radar intensity, radar velocity, and thermal temperature. | This is deterministic attribute fusion, not TSDF, occupancy probability, Bayesian filtering, or SLAM. |
| Acoustic path | Direct-path voxel blocking, geometric travel time, and classical Doppler helpers. | No reflections, diffraction, reverberation, frequency-dependent attenuation, or wave simulation. |
| Lighting | Real first-order spherical-harmonic accumulation and evaluation. | No higher-order bands, light transport, environment-map loading, or renderer integration. |

## Execution architecture

`ExecutionRuntime` separates **requested backend intent** from the **active backend**. It currently supports portable CPU serial execution and C++ standard-library CPU parallel execution for independent index ranges. Requests for GPU, NPU, or APU execution are accepted by the API and report a deterministic CPU-serial fallback until dedicated backends exist.

| Target environment | Current route | Future backend responsibility |
|---|---|---|
| Older CPU / embedded CPU | `kCpuSerial`; lowest implementation and memory-complexity baseline. | Keep scalar code path and bounded memory policies. |
| Modern consumer or server CPU | `kCpuParallel`; independent work ranges over the C++ thread runtime. | Add task scheduling, SIMD, cache-aware data layout, and benchmark-guided worker control. |
| GPU | Explicit `kGpu` request currently falls back to CPU serial. | Add GPU-resident compact voxel storage, batched traversal, compute kernels, and device-memory budgeting. |
| NPU | Explicit `kNpu` request currently falls back to CPU serial. | Restrict use to learned inference or post-processing adapters, not geometry correctness. |
| APU / heterogeneous SoC | Explicit `kApu` request currently falls back to CPU serial. | Add a platform-specific scheduler and shared-memory capability checks. |

The fallback is intentional. It prevents accidental claims that an unavailable accelerator is active and preserves an inspectable path for obsolete, consumer, professional, and data-center CPU hardware.

## Validation contract

The CMake configuration builds a reusable `Voxel4D::core` target, the demonstration executable, and independent CTest executables. Local validation uses warnings as errors, clang-format verification, AddressSanitizer, and UndefinedBehaviorSanitizer. GitHub Actions repeats build/test, sanitizer, formatting, and CodeQL checks for the public repository.

## Next production research layers

The next work must retain replayable input and numerical tests while adding calibrated hardware ingestion, robust image correspondence and outlier handling, clock synchronization, uncertainty-aware fusion, compact voxel storage, and actual accelerator backends. Learned components should remain optional downstream consumers until their input/output contracts and evaluations are reproducible.
