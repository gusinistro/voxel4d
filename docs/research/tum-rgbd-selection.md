# TUM RGB-D Selection Notes

## Selected validation target

Voxel4D will target the **TUM RGB-D SLAM Dataset and Benchmark**, beginning with `freiburg1_xyz`. The official download page recommends the `xyz` sequences for first experiments because the motion is relatively small and the covered volume is limited, which is appropriate for a transparent reference odometry baseline.[1] Use of the data should cite the benchmark publication by Sturm *et al.*.[4]

| Requirement | Official TUM format | Voxel4D adapter implication |
|---|---|---|
| RGB image | 640 × 480, 8-bit RGB PNG | Decode color PNG and preserve its timestamp. |
| Depth image | 640 × 480, 16-bit monochrome PNG | Decode depth PNG, treat zero as invalid, apply a 1/5000 m scale, and declare values as optical-axis depth. |
| RGB/depth registration | Already pre-registered 1:1 | Validate identical dimensions; do not infer a separate depth extrinsic. |
| Ground truth | `timestamp tx ty tz qx qy qz qw` | Parse pose records and compare estimated relative motion only after timestamp association. |
| Intrinsics for baseline | `freiburg1` RGB calibration: `fx=517.3`, `fy=516.5`, `cx=318.6`, `cy=255.3` | Supply documented camera intrinsics; do not claim distortion correction. |
| License | CC BY 4.0 unless stated otherwise | Retain attribution and cite the benchmark publication. |

The official specification states that TUM depth values are scaled by 5000 and that a value of zero means missing data.[2] Voxel4D represents the decoded TUM sample as **optical-axis depth** rather than normalized-ray distance, then performs the corresponding pinhole lifting in RGB-D odometry. The PNM loader remains useful for controlled experiments with its explicit normalized-ray-distance convention, while the TUM adapter adds PNG decoding and explicit zero-depth rejection.

## Scope boundary

This choice validates the recorded RGB-D ingest and reference odometry paths against a public Kinect sequence. It does not validate multi-camera synchronization, LiDAR, radar, thermal, IMU, a physical sensor driver, GPU/NPU/APU execution, or a learned model.

## References

[1]: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download "TUM RGB-D Dataset Download"
[2]: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats "TUM RGB-D File Formats"
[3]: https://cvg.cit.tum.de/data/datasets/rgbd-dataset "TUM RGB-D SLAM Dataset and Benchmark"
[4]: https://cvg.cit.tum.de/_media/spezial/bib/sturm12iros.pdf "A Benchmark for the Evaluation of RGB-D SLAM Systems"
