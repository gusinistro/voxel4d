#pragma once

#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

#include "pnm_image.h"
#include "visual_odometry.h"

namespace voxel4d {

/** @brief A matched image feature after local patch matching and displacement filtering. */
struct ImageFeatureMatch {
    glm::ivec2 previous_pixel{0};
    glm::ivec2 current_pixel{0};
    float patch_error{0.0F};

    [[nodiscard]] bool is_valid() const;
};

/** @brief Result of dependency-free RGB-D visual odometry. */
struct ImageVisualOdometryResult {
    bool success{false};
    std::size_t candidate_feature_count{0U};
    std::size_t matched_feature_count{0U};
    std::size_t inlier_feature_count{0U};
    glm::vec2 median_pixel_displacement{0.0F};
    VisualOdometryResult rigid_motion{};
};

/**
 * @brief Reference RGB-D odometry using image gradients, 3x3 patch matching, and median filtering.
 *
 * This intentionally dependency-free baseline expects matching calibrated cameras, P5/P6 color
 * images, depth stored as distance along a normalized camera ray, and modest inter-frame
 * displacement. It is not a replacement for robust feature descriptors, RANSAC, optical flow,
 * bundle adjustment, or SLAM.
 */
class ImageVisualOdometry {
   public:
    /** @throws std::invalid_argument for invalid configuration. */
    ImageVisualOdometry(int maximum_features, int search_radius_pixels, float maximum_patch_error,
                        int displacement_inlier_tolerance_pixels);

    [[nodiscard]] ImageVisualOdometryResult estimate_rgbd_motion(
        const DecodedRgbdFrame& previous_frame, const DecodedRgbdFrame& current_frame) const;

   private:
    [[nodiscard]] float luminance_at(const PnmImage& image, int x_pixels, int y_pixels) const;
    [[nodiscard]] std::vector<glm::ivec2> select_features(const PnmImage& image) const;
    [[nodiscard]] std::vector<ImageFeatureMatch> match_features(
        const PnmImage& previous_image, const PnmImage& current_image,
        const std::vector<glm::ivec2>& features) const;

    int maximum_features_{0};
    int search_radius_pixels_{0};
    float maximum_patch_error_{0.0F};
    int displacement_inlier_tolerance_pixels_{0};
};

}  // namespace voxel4d
