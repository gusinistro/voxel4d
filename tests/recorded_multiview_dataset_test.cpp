#include "recorded_multiview_dataset.h"

#include <filesystem>
#include <string>
#include <vector>

#include "test_support.h"

namespace {

voxel4d::CalibratedCamera make_camera(const std::string& identifier, const float x_meters) {
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = identifier;
    camera.intrinsics = voxel4d::CameraIntrinsics{64, 48, 50.0F, 50.0F, 32.0F, 24.0F};
    camera.world_from_camera.translation_meters = glm::vec3(x_meters, 0.0F, 0.0F);
    return camera;
}

}  // namespace

int main() {
    voxel4d::test::TestContext test;
    const std::filesystem::path manifest_path =
        std::filesystem::temp_directory_path() / "voxel4d_multiview_manifest_test.csv";
    const std::vector<voxel4d::RecordedCameraFrame> frames{
        voxel4d::RecordedCameraFrame{"left", 100, "left_000100.ppm", "left_000100.pgm"},
        voxel4d::RecordedCameraFrame{"right", 102, "right_000102.ppm", "right_000102.pgm"},
        voxel4d::RecordedCameraFrame{"left", 200, "left_000200.ppm", "left_000200.pgm"},
    };
    test.expect(
        voxel4d::RecordedMultiviewDataset::write_manifest_csv(manifest_path.string(), frames),
        "Valid recorded frame manifests must be writable");
    const std::vector<voxel4d::RecordedCameraFrame> replayed =
        voxel4d::RecordedMultiviewDataset::read_manifest_csv(manifest_path.string());
    test.expect(replayed.size() == frames.size() && replayed.at(1).camera_id == "right" &&
                    replayed.at(2).timestamp_nanoseconds == 200,
                "Manifest round-trip must preserve camera identifiers and timestamps");

    const std::vector<voxel4d::CalibratedCamera> cameras{make_camera("left", -0.5F),
                                                         make_camera("right", 0.5F)};
    const std::vector<voxel4d::CalibratedRecordedFrame> bound =
        voxel4d::RecordedMultiviewDataset::bind_calibrations(replayed, cameras);
    test.expect(bound.size() == replayed.size() && bound.at(0).is_valid() &&
                    bound.at(1).camera->camera_id == "right",
                "Manifest binding must retain valid associations to supplied calibrations");

    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(voxel4d::RecordedMultiviewDataset::bind_calibrations(
                {voxel4d::RecordedCameraFrame{"unknown", 100, "color.ppm", "depth.pgm"}}, cameras));
        },
        "Binding must reject unknown recorded camera identifiers");
    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(voxel4d::RecordedMultiviewDataset::bind_calibrations(
                {voxel4d::RecordedCameraFrame{"left", 200, "left1.ppm", "left1.pgm"},
                 voxel4d::RecordedCameraFrame{"left", 200, "left2.ppm", "left2.pgm"}},
                cameras));
        },
        "Binding must reject non-increasing timestamps for the same camera");
    test.expect(!voxel4d::RecordedMultiviewDataset::write_manifest_csv(
                    manifest_path.string(),
                    {voxel4d::RecordedCameraFrame{"left", 100, "color,invalid.ppm", "depth.pgm"}}),
                "Writer must reject unescaped comma-containing paths in strict CSV mode");

    std::filesystem::remove(manifest_path);
    return test.failures() == 0 ? 0 : 1;
}
