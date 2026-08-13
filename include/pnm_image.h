#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "recorded_multiview_dataset.h"

namespace voxel4d {

/** @brief Portable P5/P6 image decoded to unsigned 16-bit sample values. */
struct PnmImage {
    int width_pixels{0};
    int height_pixels{0};
    int channel_count{0};
    std::uint16_t maximum_value{0U};
    std::vector<std::uint16_t> samples{};

    [[nodiscard]] bool is_valid() const;
    [[nodiscard]] std::uint16_t sample_at(int x_pixels, int y_pixels, int channel) const;
};

/** @brief Geometric convention used by the decoded depth values. */
enum class DepthConvention {
    /** The depth sample is distance along the corresponding normalized camera ray. */
    kAlongUnitRay,
    /** The depth sample is the positive magnitude on the camera optical axis. */
    kOpticalAxis,
};

/** @brief A decoded RGB-D pair associated with one calibrated recorded camera frame. */
struct DecodedRgbdFrame {
    CalibratedRecordedFrame recorded_frame{};
    PnmImage color{};
    PnmImage depth{};
    float depth_scale_meters_per_unit{0.0F};
    DepthConvention depth_convention{DepthConvention::kAlongUnitRay};

    [[nodiscard]] bool is_valid() const;
    /** @return Depth in meters in the declared `depth_convention`. */
    [[nodiscard]] float depth_meters_at(int x_pixels, int y_pixels) const;
};

/**
 * @brief Dependency-free P5/P6 decoding and strict calibrated RGB-D loading.
 *
 * The loader accepts only binary PNM P5/P6 files. It checks image dimensions
 * against the supplied calibrated camera and never guesses depth units.
 */
class PnmImageCodec {
   public:
    [[nodiscard]] static PnmImage read(const std::string& file_path);
    [[nodiscard]] static bool write(const PnmImage& image, const std::string& file_path);

    [[nodiscard]] static DecodedRgbdFrame load_calibrated_rgbd(
        const CalibratedRecordedFrame& recorded_frame, float depth_scale_meters_per_unit);
};

}  // namespace voxel4d
