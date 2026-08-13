# Recorded RGB-D Input Baseline

## Supported portable baseline

Voxel4D can now bind a strict multiview manifest to supplied calibrated cameras and decode a dependency-free RGB-D pair composed of binary PNM files. This is an ingestion baseline for reproducible experiments, not a claim of universal camera or dataset support.

| Artifact | Required format | Validation |
|---|---|---|
| Multiview manifest | UTF-8 CSV with exact header `camera_id,timestamp_nanoseconds,color_path,depth_path` | Non-empty paths, non-negative timestamps, global time ordering, strictly increasing timestamps per camera, and a known calibration identifier. |
| Color frame | Binary P6 PPM or P5 PGM | Dimensions must equal the associated camera intrinsics. |
| Depth frame | Binary P5 PGM, 8-bit or big-endian 16-bit samples | Dimensions must equal color and calibration; caller supplies a positive meters-per-unit scale. |
| Calibration | `CalibratedCamera` provided by the application | Valid pinhole intrinsics, rigid pose, and a unique `camera_id`. |

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

`PnmImageCodec::load_calibrated_rgbd` treats a stored depth value multiplied by `depth_scale_meters_per_unit` as **distance along the normalized camera ray**. The current image-odometry reference implementation uses the same convention when lifting matched pixels to camera-space 3D points. A sensor that reports optical-axis depth must be converted before invoking this baseline or handled by a dedicated adapter.

## Image odometry baseline

`ImageVisualOdometry` selects image-gradient features, searches a local window using 3 × 3 photometric patches, removes displacement outliers with a median rule, lifts valid depth samples, then passes 3D correspondences to the rigid estimator. It is deliberately narrow: it expects modest motion, stable brightness, compatible intrinsics, and at least three valid inlier features.

> This implementation is a transparent CPU reference path. It does not provide scale-free monocular odometry, calibrated feature descriptors, RANSAC, rolling-shutter compensation, exposure normalization, loop closure, or field-grade pose confidence.

## Next adapter requirements

A new dataset or device adapter should preserve raw timestamps, report any clock conversion, identify the camera calibration used for each frame, validate image dimensions and units, expose dropped or invalid frames, and provide a CPU-reference replay test. Hardware capture or a vendor SDK must not be declared supported until its adapter is implemented and tested with a recorded sample.
