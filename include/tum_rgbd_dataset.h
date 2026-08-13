#pragma once

#include <string>
#include <vector>

#include "pnm_image.h"

namespace voxel4d {

/** @brief Timestamped TUM RGB-D ground-truth pose record. */
struct TumGroundTruthPose {
    double timestamp_seconds{0.0};
    SensorPose world_from_camera{};

    [[nodiscard]] bool is_valid() const;
};

/** @brief A TUM RGB/depth association resolved to paths under one sequence directory. */
struct TumRgbdAssociation {
    double color_timestamp_seconds{0.0};
    double depth_timestamp_seconds{0.0};
    std::string color_path{};
    std::string depth_path{};

    [[nodiscard]] bool is_valid() const;
};

/** @brief PNG decoder used only when Voxel4D is built with system libpng support. */
class PngImageCodec {
   public:
    [[nodiscard]] static bool is_available();
    [[nodiscard]] static PnmImage read_rgb8(const std::string& file_path);
    [[nodiscard]] static PnmImage read_gray16(const std::string& file_path);
};

/**
 * @brief Adapter for the public TUM RGB-D association and ground-truth text format.
 *
 * It accepts a sequence directory containing `rgb.txt`, `depth.txt`, and
 * optionally `groundtruth.txt`. The TUM 16-bit depth scale is 1/5000 m per unit.
 */
class TumRgbdDataset {
   public:
    static constexpr float kDepthScaleMetersPerUnit = 1.0F / 5000.0F;

    [[nodiscard]] static std::vector<TumRgbdAssociation> read_associations(
        const std::string& sequence_directory, double maximum_timestamp_difference_seconds);
    [[nodiscard]] static std::vector<TumGroundTruthPose> read_ground_truth(
        const std::string& sequence_directory);

    [[nodiscard]] static DecodedRgbdFrame load_frame(const TumRgbdAssociation& association,
                                                     const CalibratedCamera& camera);
};

}  // namespace voxel4d
