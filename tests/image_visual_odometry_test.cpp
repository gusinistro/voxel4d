#include "image_visual_odometry.h"

#include <string>
#include <vector>

#include "test_support.h"

namespace {

constexpr int kWidth = 32;
constexpr int kHeight = 24;

voxel4d::CalibratedCamera make_camera() {
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = "camera";
    camera.intrinsics = voxel4d::CameraIntrinsics{kWidth, kHeight, 70.0F, 70.0F, 16.0F, 12.0F};
    return camera;
}

voxel4d::PnmImage make_pattern(const int horizontal_shift) {
    voxel4d::PnmImage image{kWidth, kHeight, 3, 255U, {}};
    image.samples.resize(static_cast<std::size_t>(kWidth * kHeight * 3));
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const int source_x = x - horizontal_shift;
            const std::uint16_t value =
                source_x >= 0 && source_x < kWidth
                    ? static_cast<std::uint16_t>((source_x * 29 + y * 47 + source_x * y * 3) % 256)
                    : static_cast<std::uint16_t>(0U);
            const std::size_t offset =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(kWidth) +
                 static_cast<std::size_t>(x)) *
                3U;
            image.samples.at(offset) = value;
            image.samples.at(offset + 1U) = value;
            image.samples.at(offset + 2U) = value;
        }
    }
    return image;
}

voxel4d::DecodedRgbdFrame make_frame(
    const voxel4d::CalibratedCamera* camera, const voxel4d::PnmImage& color,
    const voxel4d::TimestampNanoseconds timestamp,
    const voxel4d::DepthConvention depth_convention = voxel4d::DepthConvention::kAlongUnitRay) {
    voxel4d::PnmImage depth{kWidth, kHeight, 1, 1000U, {}};
    depth.samples.assign(static_cast<std::size_t>(kWidth * kHeight), 1000U);
    voxel4d::DecodedRgbdFrame frame{
        voxel4d::CalibratedRecordedFrame{
            camera, voxel4d::RecordedCameraFrame{"camera", timestamp, "color.ppm", "depth.pgm"}},
        color, depth, 0.001F};
    frame.depth_convention = depth_convention;
    return frame;
}

}  // namespace

int main() {
    voxel4d::test::TestContext test;
    const voxel4d::CalibratedCamera camera = make_camera();
    const voxel4d::DecodedRgbdFrame previous = make_frame(&camera, make_pattern(0), 100);
    const voxel4d::DecodedRgbdFrame current = make_frame(&camera, make_pattern(2), 200);

    const voxel4d::ImageVisualOdometry estimator(30, 3, 0.0001F, 0);
    const voxel4d::ImageVisualOdometryResult result =
        estimator.estimate_rgbd_motion(previous, current);
    test.expect(result.candidate_feature_count >= 3U && result.matched_feature_count >= 3U,
                "Image odometry must select and match multiple textured RGB-D features");
    test.expect(result.inlier_feature_count >= 3U && result.success,
                "Image odometry must estimate a rigid motion from consistent RGB-D matches");
    test.expect_near(
        result.median_pixel_displacement.x, 2.0F, 0.01F,
        "Image odometry median displacement must recover the synthetic horizontal shift");
    test.expect_near(
        result.median_pixel_displacement.y, 0.0F, 0.01F,
        "Image odometry median displacement must retain the synthetic vertical position");

    const voxel4d::DecodedRgbdFrame axial_previous =
        make_frame(&camera, make_pattern(0), 300, voxel4d::DepthConvention::kOpticalAxis);
    const voxel4d::DecodedRgbdFrame axial_current =
        make_frame(&camera, make_pattern(2), 400, voxel4d::DepthConvention::kOpticalAxis);
    const voxel4d::ImageVisualOdometryResult axial_result =
        estimator.estimate_rgbd_motion(axial_previous, axial_current);
    test.expect(
        axial_result.success && axial_result.rigid_motion.root_mean_square_error_meters < 1.0e-4F,
        "Axial depth must produce a rigid planar translation under pinhole lifting");
    test.expect_near(axial_result.rigid_motion.current_from_previous.translation_meters.x,
                     2.0F / 70.0F, 1.0e-4F,
                     "Axial depth lifting must recover the expected optical-axis translation");

    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(voxel4d::ImageVisualOdometry(2, 1, 0.1F, 1)); },
        "Image odometry must require enough features for a rigid estimate");

    return test.failures() == 0 ? 0 : 1;
}
