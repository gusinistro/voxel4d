#include "image_visual_odometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace voxel4d {
namespace {

struct ScoredFeature {
    glm::ivec2 pixel{0};
    float score{0.0F};
};

float median(std::vector<float> values) {
    if (values.empty()) {
        throw std::invalid_argument("Cannot calculate a median from an empty collection");
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    if (values.size() % 2U == 0U) {
        return (values.at(middle - 1U) + values.at(middle)) * 0.5F;
    }
    return values.at(middle);
}

glm::vec3 point_in_camera_meters(const DecodedRgbdFrame& frame, const glm::ivec2 pixel) {
    const CameraIntrinsics& intrinsics = frame.recorded_frame.camera->intrinsics;
    const float x = (static_cast<float>(pixel.x) - intrinsics.principal_point_x_pixels) /
                    intrinsics.focal_length_x_pixels;
    const float y = -(static_cast<float>(pixel.y) - intrinsics.principal_point_y_pixels) /
                    intrinsics.focal_length_y_pixels;
    const glm::vec3 ray = glm::normalize(glm::vec3(x, y, -1.0F));
    const float depth_meters = frame.depth_meters_at(pixel.x, pixel.y);
    if (!std::isfinite(depth_meters) || depth_meters <= 0.0F) {
        throw std::invalid_argument("RGB-D correspondence has invalid depth");
    }
    return ray * depth_meters;
}

bool compatible_cameras(const DecodedRgbdFrame& previous, const DecodedRgbdFrame& current) {
    const CameraIntrinsics& a = previous.recorded_frame.camera->intrinsics;
    const CameraIntrinsics& b = current.recorded_frame.camera->intrinsics;
    return previous.recorded_frame.camera->camera_id == current.recorded_frame.camera->camera_id &&
           a.width_pixels == b.width_pixels && a.height_pixels == b.height_pixels &&
           a.focal_length_x_pixels == b.focal_length_x_pixels &&
           a.focal_length_y_pixels == b.focal_length_y_pixels &&
           a.principal_point_x_pixels == b.principal_point_x_pixels &&
           a.principal_point_y_pixels == b.principal_point_y_pixels;
}

}  // namespace

bool ImageFeatureMatch::is_valid() const {
    return previous_pixel.x >= 0 && previous_pixel.y >= 0 && current_pixel.x >= 0 &&
           current_pixel.y >= 0 && std::isfinite(patch_error) && patch_error >= 0.0F;
}

ImageVisualOdometry::ImageVisualOdometry(const int maximum_features, const int search_radius_pixels,
                                         const float maximum_patch_error,
                                         const int displacement_inlier_tolerance_pixels)
    : maximum_features_(maximum_features),
      search_radius_pixels_(search_radius_pixels),
      maximum_patch_error_(maximum_patch_error),
      displacement_inlier_tolerance_pixels_(displacement_inlier_tolerance_pixels) {
    if (maximum_features_ < 3 || search_radius_pixels_ < 0 ||
        !std::isfinite(maximum_patch_error_) || maximum_patch_error_ < 0.0F ||
        displacement_inlier_tolerance_pixels_ < 0) {
        throw std::invalid_argument("Image visual odometry configuration is invalid");
    }
}

float ImageVisualOdometry::luminance_at(const PnmImage& image, const int x_pixels,
                                        const int y_pixels) const {
    if (!image.is_valid()) {
        throw std::invalid_argument("Image visual odometry requires a valid PNM image");
    }
    const float scale = 1.0F / static_cast<float>(image.maximum_value);
    if (image.channel_count == 1) {
        return static_cast<float>(image.sample_at(x_pixels, y_pixels, 0)) * scale;
    }
    const float red = static_cast<float>(image.sample_at(x_pixels, y_pixels, 0)) * scale;
    const float green = static_cast<float>(image.sample_at(x_pixels, y_pixels, 1)) * scale;
    const float blue = static_cast<float>(image.sample_at(x_pixels, y_pixels, 2)) * scale;
    return 0.2126F * red + 0.7152F * green + 0.0722F * blue;
}

std::vector<glm::ivec2> ImageVisualOdometry::select_features(const PnmImage& image) const {
    if (!image.is_valid()) {
        throw std::invalid_argument("Image visual odometry requires valid input images");
    }
    const int margin = search_radius_pixels_ + 1;
    if (image.width_pixels < margin * 2 + 1 || image.height_pixels < margin * 2 + 1) {
        return {};
    }
    std::vector<ScoredFeature> candidates;
    for (int y = margin; y < image.height_pixels - margin; y += 2) {
        for (int x = margin; x < image.width_pixels - margin; x += 2) {
            const float gradient_x = luminance_at(image, x + 1, y) - luminance_at(image, x - 1, y);
            const float gradient_y = luminance_at(image, x, y + 1) - luminance_at(image, x, y - 1);
            const float score = gradient_x * gradient_x + gradient_y * gradient_y;
            if (score > 0.0F) {
                candidates.push_back(ScoredFeature{glm::ivec2(x, y), score});
            }
        }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const ScoredFeature& left, const ScoredFeature& right) {
                  if (left.score != right.score) {
                      return left.score > right.score;
                  }
                  if (left.pixel.y != right.pixel.y) {
                      return left.pixel.y < right.pixel.y;
                  }
                  return left.pixel.x < right.pixel.x;
              });

    std::vector<glm::ivec2> selected;
    selected.reserve(static_cast<std::size_t>(maximum_features_));
    constexpr int kMinimumSpacingPixels = 3;
    for (const ScoredFeature& candidate : candidates) {
        bool sufficiently_separated = true;
        for (const glm::ivec2& existing : selected) {
            const glm::ivec2 delta = candidate.pixel - existing;
            if (delta.x * delta.x + delta.y * delta.y <
                kMinimumSpacingPixels * kMinimumSpacingPixels) {
                sufficiently_separated = false;
                break;
            }
        }
        if (sufficiently_separated) {
            selected.push_back(candidate.pixel);
            if (selected.size() == static_cast<std::size_t>(maximum_features_)) {
                break;
            }
        }
    }
    return selected;
}

std::vector<ImageFeatureMatch> ImageVisualOdometry::match_features(
    const PnmImage& previous_image, const PnmImage& current_image,
    const std::vector<glm::ivec2>& features) const {
    if (!previous_image.is_valid() || !current_image.is_valid() ||
        previous_image.width_pixels != current_image.width_pixels ||
        previous_image.height_pixels != current_image.height_pixels) {
        throw std::invalid_argument("Feature matching requires valid equal-sized images");
    }

    std::map<std::pair<int, int>, ImageFeatureMatch> best_by_current_pixel;
    for (const glm::ivec2& feature : features) {
        float lowest_error = std::numeric_limits<float>::infinity();
        glm::ivec2 best_pixel(-1);
        for (int offset_y = -search_radius_pixels_; offset_y <= search_radius_pixels_; ++offset_y) {
            for (int offset_x = -search_radius_pixels_; offset_x <= search_radius_pixels_;
                 ++offset_x) {
                const glm::ivec2 candidate = feature + glm::ivec2(offset_x, offset_y);
                if (candidate.x <= 0 || candidate.x >= current_image.width_pixels - 1 ||
                    candidate.y <= 0 || candidate.y >= current_image.height_pixels - 1) {
                    continue;
                }
                float squared_error = 0.0F;
                for (int patch_y = -1; patch_y <= 1; ++patch_y) {
                    for (int patch_x = -1; patch_x <= 1; ++patch_x) {
                        const float difference =
                            luminance_at(previous_image, feature.x + patch_x, feature.y + patch_y) -
                            luminance_at(current_image, candidate.x + patch_x,
                                         candidate.y + patch_y);
                        squared_error += difference * difference;
                    }
                }
                const float mean_error = squared_error / 9.0F;
                if (mean_error < lowest_error) {
                    lowest_error = mean_error;
                    best_pixel = candidate;
                }
            }
        }
        if (best_pixel.x >= 0 && lowest_error <= maximum_patch_error_) {
            const ImageFeatureMatch match{feature, best_pixel, lowest_error};
            const std::pair<int, int> key{best_pixel.x, best_pixel.y};
            const auto existing = best_by_current_pixel.find(key);
            if (existing == best_by_current_pixel.end() ||
                match.patch_error < existing->second.patch_error) {
                best_by_current_pixel[key] = match;
            }
        }
    }

    std::vector<ImageFeatureMatch> matches;
    matches.reserve(best_by_current_pixel.size());
    for (const auto& item : best_by_current_pixel) {
        matches.push_back(item.second);
    }
    return matches;
}

ImageVisualOdometryResult ImageVisualOdometry::estimate_rgbd_motion(
    const DecodedRgbdFrame& previous_frame, const DecodedRgbdFrame& current_frame) const {
    if (!previous_frame.is_valid() || !current_frame.is_valid() ||
        !compatible_cameras(previous_frame, current_frame)) {
        throw std::invalid_argument("RGB-D odometry requires compatible valid calibrated frames");
    }
    const std::vector<glm::ivec2> features = select_features(previous_frame.color);
    const std::vector<ImageFeatureMatch> matches =
        match_features(previous_frame.color, current_frame.color, features);

    ImageVisualOdometryResult result{};
    result.candidate_feature_count = features.size();
    result.matched_feature_count = matches.size();
    if (matches.size() < 3U) {
        return result;
    }

    std::vector<float> displacements_x;
    std::vector<float> displacements_y;
    displacements_x.reserve(matches.size());
    displacements_y.reserve(matches.size());
    for (const ImageFeatureMatch& match : matches) {
        displacements_x.push_back(
            static_cast<float>(match.current_pixel.x - match.previous_pixel.x));
        displacements_y.push_back(
            static_cast<float>(match.current_pixel.y - match.previous_pixel.y));
    }
    result.median_pixel_displacement = glm::vec2(median(displacements_x), median(displacements_y));

    std::vector<PointCorrespondence3D> correspondences;
    for (const ImageFeatureMatch& match : matches) {
        const float displacement_x =
            static_cast<float>(match.current_pixel.x - match.previous_pixel.x);
        const float displacement_y =
            static_cast<float>(match.current_pixel.y - match.previous_pixel.y);
        if (std::fabs(displacement_x - result.median_pixel_displacement.x) >
                static_cast<float>(displacement_inlier_tolerance_pixels_) ||
            std::fabs(displacement_y - result.median_pixel_displacement.y) >
                static_cast<float>(displacement_inlier_tolerance_pixels_)) {
            continue;
        }
        try {
            correspondences.push_back(
                PointCorrespondence3D{point_in_camera_meters(previous_frame, match.previous_pixel),
                                      point_in_camera_meters(current_frame, match.current_pixel)});
        } catch (const std::invalid_argument&) {
            continue;
        }
    }
    result.inlier_feature_count = correspondences.size();
    if (correspondences.size() < 3U) {
        return result;
    }
    result.rigid_motion = VisualOdometryEstimator().estimate_rigid_motion(correspondences);
    result.success = result.rigid_motion.success;
    return result;
}

}  // namespace voxel4d
