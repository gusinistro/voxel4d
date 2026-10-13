#include "rgbd_temporal_integrator.h"

#include <glm/glm.hpp>

#include "test_support.h"

namespace {
voxel4d::DecodedRgbdFrame make_frame(const voxel4d::CalibratedCamera& camera) {
    voxel4d::RecordedCameraFrame recorded{"test-camera", 1000000000, "color.pnm", "depth.pgm"};
    voxel4d::DecodedRgbdFrame frame{};
    frame.recorded_frame = voxel4d::CalibratedRecordedFrame{&camera, recorded};
    frame.color = voxel4d::PnmImage{2, 1, 3, 255U, {255U, 0U, 0U, 0U, 255U, 0U}};
    frame.depth = voxel4d::PnmImage{2, 1, 1, 1000U, {1000U, 0U}};
    frame.depth_scale_meters_per_unit = 0.001F;
    frame.depth_convention = voxel4d::DepthConvention::kOpticalAxis;
    return frame;
}
}  // namespace

int main() {
    voxel4d::test::TestContext test;
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = "test-camera";
    camera.intrinsics = voxel4d::CameraIntrinsics{2, 1, 1.0F, 1.0F, 0.5F, 0.0F};
    const voxel4d::DecodedRgbdFrame frame = make_frame(camera);
    TemporalVoxelMap map(2U);
    voxel4d::RgbdTemporalIntegrator integrator(2U, 4.0F);

    voxel4d::SensorPose pose{};
    pose.translation_meters = glm::vec3(1.0F, 0.0F, 0.0F);
    const voxel4d::RgbdIntegrationResult result =
        integrator.integrate(frame, pose, 1000000000, map);

    test.expect(result.valid_depth_samples == 1U,
                "RGB-D integration must count only positive finite depth samples");
    test.expect(result.inserted_samples == 1U,
                "RGB-D integration must insert the valid sample into the SVO");
    test.expect(map.get_snapshot_count() == 1U,
                "RGB-D integration must append one timestamped snapshot");
    const auto snapshot = map.get_latest_snapshot();
    test.expect(snapshot != nullptr, "RGB-D integration must retain the generated snapshot");
    if (snapshot) {
        const auto leaf = snapshot->search(glm::vec3(0.5F, 0.0F, -1.0F));
        test.expect(leaf != nullptr && leaf->attribute.density > 0.0F,
                    "Axial depth must be lifted and transformed into world-space geometry");
    }

    return test.failures() == 0 ? 0 : 1;
}
