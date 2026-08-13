# Recorded RGB-D Input Baseline

## Supported portable baseline

Voxel4D can now bind a strict multiview manifest to supplied calibrated cameras and decode a dependency-free RGB-D pair composed of binary PNM files. This is an ingestion baseline for reproducible experiments, not a claim of universal camera or dataset support.

| Artifact | Required format | Validation |
|---|---|---|
| Multiview manifest | UTF-8 CSV with exact header `camera_id,timestamp_nanoseconds,color_path,depth_path` | Non-empty paths, non-negative timestamps, global time ordering, strictly increasing timestamps per camera, and a known calibration identifier. |
| Color frame | Binary P6 PPM or P5 PGM | Dimensions must equal the associated camera intrinsics. |
| Depth frame | Binary P5 PGM, 8-bit or big-endian 16-bit samples | Dimensions must equal color and calibration; caller supplies a positive meters-per-unit scale. |
| Calibration | `CalibratedCamera` provided by the application | Valid pinhole intrinsics, rigid pose, and a unique `camera_id`. |
| TUM RGB-D sequence | `rgb.txt` + `depth.txt`, 8-bit RGB PNG, 16-bit grayscale PNG, optional `groundtruth.txt` | RGB/depth association within a caller-supplied tolerance; image dimensions and units checked against fr1 calibration; requires the optional system libpng build path. |

A manifest must be written in chronological order. It intentionally keeps file references separate from image decoding so that future ROS bags, vendor SDKs, PNG/TIFF, video, and depth-stream adapters can preserve the same calibration and timestamp contract.

## Example manifest

```csv
camera_id,timestamp_nanoseconds,color_path,depth_path
left,1000000000,frames/left_000001.ppm,depth/left_000001.pgm
right,1002000000,frames/right_000001.ppm,depth/right_000001.pgm
left,1033333333,frames/left_000002.ppm,depth/left_000002.pgm
right,1035333333,frames/right_000002.ppm,depth/right_000002.pgm
```

## Depth convention

`DecodedRgbdFrame` declares its geometry through `DepthConvention`. `PnmImageCodec::load_calibrated_rgbd` treats a stored depth value multiplied by `depth_scale_meters_per_unit` as **distance along the normalized camera ray** (`kAlongUnitRay`). `TumRgbdDataset::load_frame` declares TUM's 16-bit samples as positive **optical-axis depth** (`kOpticalAxis`) and applies the documented scale of `1/5000 m` per unit. The image-odometry baseline lifts each convention with its respective pinhole geometry: a normalized ray for the former and `(x, y, -1) × z` for the latter, where `x` and `y` are normalized image coordinates. Zero depth is invalid in both cases.

## Image odometry baseline

`ImageVisualOdometry` selects image-gradient features, searches a local window using 3 × 3 photometric patches, removes displacement outliers with a median rule, lifts valid depth samples, then passes 3D correspondences to the rigid estimator. It is deliberately narrow: it expects modest motion, stable brightness, compatible intrinsics, and at least three valid inlier features.

> This implementation is a transparent CPU reference path. It does not provide scale-free monocular odometry, calibrated feature descriptors, RANSAC, rolling-shutter compensation, exposure normalization, loop closure, or field-grade pose confidence.

## TUM RGB-D replay evaluation

The optional `voxel4d_tum_odometry_eval` executable evaluates consecutive associated pairs and writes one CSV row per pair. It uses the `freiburg1_xyz` sequence from the TUM RGB-D benchmark, whose official files specify pre-registered 640 × 480 RGB/depth PNG images, a depth factor of 5000, and timestamped ground-truth poses.[1] The data are **not** distributed in this repository; obtain and retain them under the benchmark's CC BY 4.0 terms.[2]

```bash
cmake -S . -B build -DVOXEL4D_WARNINGS_AS_ERRORS=ON -DBUILD_TESTING=ON
cmake --build build --parallel
./build/voxel4d_tum_odometry_eval \
  --dataset /absolute/path/rgbd_dataset_freiburg1_xyz \
  --max-pairs 50 \
  --output validation/tum_fr1_xyz_first50.csv
```

The committed reference run uses fr1 intrinsics `(fx=517.3, fy=516.5, cx=318.6, cy=255.3)`, a 20 ms RGB/depth association tolerance, a 20 ms nearest-ground-truth tolerance, 300 gradient features, a 5-pixel search radius, a 0.05 mean-squared luminance-patch threshold, and a 1-pixel median-flow inlier tolerance. Ground truth is converted into the same `current_from_previous` coordinate convention as the estimator before error calculation.

| Recorded validation | Result |
|---|---:|
| Sequence and subset | TUM `freiburg1_xyz`, first 50 consecutive associated pairs |
| Pairs with reference pose | 50 / 50 |
| Successful odometry estimates | 50 / 50 (100.0%) |
| Translational relative-pose error | 0.031653 m mean; 0.027330 m median |
| Rotational relative-pose error | 1.095915° mean; 0.893230° median |
| Observed range | 0.003316–0.100401 m translation; 0.153233–3.559645° rotation |
| Per-pair evidence | [`validation/tum_fr1_xyz_first50.csv`](../validation/tum_fr1_xyz_first50.csv) |

> This is a small, deterministic CPU baseline measurement rather than a benchmark claim against SLAM systems. It covers only the first 50 adjacent pairs, uses a simple photometric matcher without RANSAC or global optimization, and must not be extrapolated to long trajectories, difficult lighting, or real-time throughput.

## References

[1]: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats "TUM RGB-D File Formats"
[2]: https://creativecommons.org/licenses/by/4.0/ "Creative Commons Attribution 4.0 International"

## Next adapter requirements

A new dataset or device adapter should preserve raw timestamps, report any clock conversion, identify the camera calibration used for each frame, validate image dimensions and units, expose dropped or invalid frames, and provide a CPU-reference replay test. Hardware capture or a vendor SDK must not be declared supported until its adapter is implemented and tested with a recorded sample.
