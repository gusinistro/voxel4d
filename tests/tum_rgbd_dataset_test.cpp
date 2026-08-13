#include "tum_rgbd_dataset.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "test_support.h"

#ifndef VOXEL4D_HAVE_PNG
#define VOXEL4D_HAVE_PNG 0
#endif

#if VOXEL4D_HAVE_PNG
#include <png.h>
#endif

namespace {

std::filesystem::path make_fixture_directory() {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "voxel4d_tum_rgbd_dataset_test";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    {
        std::ofstream rgb(directory / "rgb.txt");
        rgb << "# timestamp color file\n1.000000 rgb/0001.png\n2.000000 rgb/0002.png\n";
    }
    {
        std::ofstream depth(directory / "depth.txt");
        depth << "# timestamp depth file\n1.010000 depth/0001.png\n2.020000 depth/0002.png\n";
    }
    {
        std::ofstream ground_truth(directory / "groundtruth.txt");
        ground_truth << "# timestamp tx ty tz qx qy qz qw\n"
                     << "1.000000 1 2 3 0 0 0 1\n"
                     << "2.000000 2 2 3 0 0 0 1\n";
    }
    return directory;
}

}  // namespace

int main() {
    voxel4d::test::TestContext test;
    const std::filesystem::path fixture_directory = make_fixture_directory();

    const std::vector<voxel4d::TumRgbdAssociation> associations =
        voxel4d::TumRgbdDataset::read_associations(fixture_directory.string(), 0.03);
    test.expect(
        associations.size() == 2U && associations.at(0).is_valid() &&
            associations.at(0).color_path == (fixture_directory / "rgb/0001.png").string(),
        "TUM adapter must match timestamp-near RGB and depth files under the sequence directory");
    test.expect_near(voxel4d::TumRgbdDataset::kDepthScaleMetersPerUnit, 0.0002F, 1.0e-8F,
                     "TUM adapter must expose the documented 1/5000 meter depth scale");

    const std::vector<voxel4d::TumGroundTruthPose> ground_truth =
        voxel4d::TumRgbdDataset::read_ground_truth(fixture_directory.string());
    test.expect(ground_truth.size() == 2U && ground_truth.at(0).is_valid(),
                "TUM adapter must parse valid timestamped ground-truth poses");
    test.expect_near(ground_truth.at(0).world_from_camera.translation_meters.x, 1.0F, 1.0e-6F,
                     "TUM adapter must preserve ground-truth translation");

#if VOXEL4D_HAVE_PNG
    const std::filesystem::path color_path = fixture_directory / "color.png";
    const std::filesystem::path depth_path = fixture_directory / "depth.png";
    png_image color_writer{};
    color_writer.version = PNG_IMAGE_VERSION;
    color_writer.width = 2U;
    color_writer.height = 1U;
    color_writer.format = PNG_FORMAT_RGB;
    const std::array<png_byte, 6U> color_samples{10U, 20U, 30U, 40U, 50U, 60U};
    test.expect(png_image_write_to_file(&color_writer, color_path.string().c_str(), 0,
                                        color_samples.data(), 0, nullptr) != 0,
                "Test fixture must write an 8-bit RGB PNG");

    png_image depth_writer{};
    depth_writer.version = PNG_IMAGE_VERSION;
    depth_writer.width = 2U;
    depth_writer.height = 1U;
    depth_writer.format = PNG_FORMAT_LINEAR_Y;
    const std::array<png_uint_16, 2U> depth_samples{5000U, 0U};
    test.expect(png_image_write_to_file(&depth_writer, depth_path.string().c_str(), 0,
                                        depth_samples.data(), 0, nullptr) != 0,
                "Test fixture must write a 16-bit depth PNG");
    const voxel4d::PnmImage decoded_color = voxel4d::PngImageCodec::read_rgb8(color_path.string());
    const voxel4d::PnmImage decoded_depth =
        voxel4d::PngImageCodec::read_gray16(depth_path.string());
    voxel4d::CalibratedCamera fixture_camera{};
    fixture_camera.camera_id = "tum-camera";
    fixture_camera.intrinsics = voxel4d::CameraIntrinsics{2, 1, 1.0F, 1.0F, 0.5F, 0.0F};
    const voxel4d::TumRgbdAssociation png_association{1.0, 1.0, color_path.string(),
                                                      depth_path.string()};
    const voxel4d::DecodedRgbdFrame loaded_frame =
        voxel4d::TumRgbdDataset::load_frame(png_association, fixture_camera);
    test.expect(voxel4d::PngImageCodec::is_available() && decoded_color.channel_count == 3 &&
                    decoded_color.sample_at(1, 0, 2) == 60U && decoded_depth.channel_count == 1 &&
                    decoded_depth.sample_at(0, 0, 0) == 5000U &&
                    decoded_depth.sample_at(1, 0, 0) == 0U &&
                    loaded_frame.depth_convention == voxel4d::DepthConvention::kOpticalAxis,
                "TUM loader must retain PNG samples and declare its 16-bit depth as axial");
    test.expect_near(loaded_frame.depth_meters_at(0, 0), 1.0F, 1.0e-6F,
                     "TUM loader must apply its documented depth scale");
#else
    test.expect(!voxel4d::PngImageCodec::is_available(),
                "PNG capability must report unavailable without libpng");
#endif

    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(
                voxel4d::TumRgbdDataset::read_associations(fixture_directory.string(), -0.1));
        },
        "TUM adapter must reject a negative association tolerance");
    test.expect_throws<std::runtime_error>(
        [&] { static_cast<void>(voxel4d::TumRgbdDataset::read_associations("/missing/tum", 0.1)); },
        "TUM adapter must reject a missing sequence directory");

    std::filesystem::remove_all(fixture_directory);
    return test.failures() == 0 ? 0 : 1;
}
