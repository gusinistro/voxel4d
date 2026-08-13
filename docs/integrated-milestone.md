# Integrated Multiview Milestone

## What is now demonstrable

The `voxel4d_poc` executable performs a deterministic twelve-step scenario that connects the core modules instead of exercising them only in isolation. It generates two synthetic RGB-D streams, maintains bounded SVO snapshots and poses, fuses simulated multissensor envelopes, synchronizes a calibrated two-camera observation pair, triangulates and reprojects a target, preserves one semantic object track, replays recorded observations from CSV, exposes externally supplied semantic labels through an optional provider interface, estimates rigid motion, traces visual and acoustic paths, evaluates L1 spherical harmonics, renders an unrecorded virtual camera viewpoint, and prints CPU timing for serial and parallel rendering.

The demonstration writes reproducible artifacts to its working-directory `data/` folder. `recorded_observations.csv` is a strict replay file for the synthetic sensor envelopes, while `free_view.ppm` is a binary PPM image rendered from a virtual calibrated camera that did not supply the source RGB-D images.

## Reproducible benchmark protocol

The optional `voxel4d_render_benchmark` target measures the same 96 × 72 free-view rendering workload for three iterations under CPU serial and two-worker CPU parallel runtimes. It prints CSV-compatible columns rather than claiming a universal performance result.

```bash
cmake -S . -B build \
  -DVOXEL4D_WARNINGS_AS_ERRORS=ON \
  -DVOXEL4D_BUILD_BENCHMARKS=ON
cmake --build build --parallel
./build/voxel4d_render_benchmark
```

> Benchmark values are **host-, compiler-, runtime-, and workload-specific**. They are not performance promises, cross-device comparisons, or evidence of GPU/NPU/APU acceleration. Repeat the protocol on the intended target hardware and record compiler version, worker count, thermal state, and input workload before comparing results.

## Data and deployment boundary

| Area | Implemented contract | Required before real deployment |
|---|---|---|
| Camera input | Synthetic RGB-D CSV plus calibrated pinhole interfaces. | Device capture, intrinsics/extrinsics calibration, exposure control, depth validation, and clock alignment. |
| Sensor input | Synthetic RGB-D, LiDAR, radar, thermal, and IMU envelopes plus strict recorded CSV replay. | Vendor-neutral file adapters, hardware drivers, integrity checks, calibration, and timing diagnostics. |
| Multiview geometry | Two-view pinhole triangulation and reprojection for synchronized observations. | Robust feature matching, outlier rejection, bundle adjustment, multi-camera calibration, and quality metrics. |
| Object state | Greedy label-aware nearest-neighbor tracks and one-step velocity estimates. | Detection/segmentation, track lifecycle policy, occlusion reasoning, re-identification, and uncertainty filtering. |
| Rendering | First-hit colored voxel reference raycasting to PPM. | Visibility-aware scene reconstruction, appearance fusion, anti-aliasing, material/light models, compression, and real-time presentation. |
| AI interface | Explicit unavailable provider and replayed-label provider contracts. | Versioned model artifacts, local or remote inference provider, privacy controls, evaluation data, bias/error analysis, and rollback behavior. |
| Hardware | Portable CPU serial/parallel execution and explicit unavailable accelerator capability reporting. | Device-specific kernels, memory budgets, telemetry, determinism checks, fallback tests, and deployment validation. |

## Safe extension rule

Any integration with a physical sensor, model runtime, GPU API, NPU SDK, or remote service should be added behind a validated adapter. The adapter must make availability, units, calibration state, timestamp semantics, failure behavior, and fallback behavior visible to the caller. The geometry and reconstruction baseline must continue to work without optional accelerators or inference providers.
