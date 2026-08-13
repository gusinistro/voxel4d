#include "pnm_image.h"

#include <filesystem>
#include <string>

#include "test_support.h"

namespace {

voxel4d::CalibratedCamera make_camera() {
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = "camera";
    camera.intrinsics = voxel4d::CameraIntrinsics{2, 1, 1.0F, 1.0F, 0.5F, 0.5F};
    return camera;
}

}  // namespace

int main() {
    voxel4d::test::TestContext test;
    const std::filesystem::path temporary_directory = std::filesystem::temp_directory_path();
    const std::filesystem::path color_path = temporary_directory / "voxel4d_color_test.ppm";
    const std::filesystem::path depth_path = temporary_directory / "voxel4d_depth_test.pgm";

    const voxel4d::PnmImage color{2, 1, 3, 255U, {255U, 0U, 0U, 0U, 128U, 255U}};
    const voxel4d::PnmImage depth{2, 1, 1, 1000U, {250U, 1000U}};
    test.expect(voxel4d::PnmImageCodec::write(color, color_path.string()) &&
                    voxel4d::PnmImageCodec::write(depth, depth_path.string()),
                "Valid P5/P6 images must be writable");
    const voxel4d::PnmImage decoded_color = voxel4d::PnmImageCodec::read(color_path.string());
    const voxel4d::PnmImage decoded_depth = voxel4d::PnmImageCodec::read(depth_path.string());
    test.expect(decoded_color.channel_count == 3 && decoded_color.sample_at(1, 0, 2) == 255U,
                "P6 round-trip must preserve three-channel samples");
    test.expect(decoded_depth.maximum_value == 1000U && decoded_depth.sample_at(1, 0, 0) == 1000U,
                "P5 16-bit round-trip must preserve depth samples");

    const voxel4d::CalibratedCamera camera = make_camera();
    const voxel4d::RecordedCameraFrame recorded{"camera", 100, color_path.string(),
                                                depth_path.string()};
    const voxel4d::CalibratedRecordedFrame calibrated{&camera, recorded};
    const voxel4d::DecodedRgbdFrame rgbd =
        voxel4d::PnmImageCodec::load_calibrated_rgbd(calibrated, 0.001F);
    test.expect(rgbd.is_valid(), "Calibrated RGB-D loader must preserve valid dimensions");
    test.expect_near(rgbd.depth_meters_at(1, 0), 1.0F, 1.0e-6F,
                     "Calibrated RGB-D loader must retain the declared depth scale");
    test.expect_throws<std::invalid_argument>(
        [&] { static_cast<void>(voxel4d::PnmImageCodec::load_calibrated_rgbd(calibrated, 0.0F)); },
        "RGB-D loader must reject an undefined depth scale");
    test.expect_throws<std::out_of_range>(
        [&] { static_cast<void>(decoded_depth.sample_at(2, 0, 0)); },
        "PNM image must reject out-of-range sampling");

    std::filesystem::remove(color_path);
    std::filesystem::remove(depth_path);
    return test.failures() == 0 ? 0 : 1;
}
