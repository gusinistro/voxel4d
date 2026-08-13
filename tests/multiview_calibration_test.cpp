#include "multiview_calibration.h"

#include <glm/gtc/quaternion.hpp>
#include <stdexcept>
#include <vector>

#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;

    voxel4d::CameraIntrinsics intrinsics{};
    intrinsics.width_pixels = 100;
    intrinsics.height_pixels = 80;
    intrinsics.focal_length_x_pixels = 100.0F;
    intrinsics.focal_length_y_pixels = 100.0F;
    intrinsics.principal_point_x_pixels = 50.0F;
    intrinsics.principal_point_y_pixels = 40.0F;
    test.expect(intrinsics.is_valid(), "Positive finite pinhole intrinsics must be valid");

    voxel4d::CalibratedCamera camera{};
    camera.camera_id = "camera-a";
    camera.intrinsics = intrinsics;
    test.expect(camera.is_valid(), "Calibrated camera with identity pose must be valid");
    const glm::vec3 center_ray = camera.pixel_to_unit_ray_world(50, 40);
    test.expect_near(center_ray.x, 0.0F, 1.0e-6F,
                     "Principal-point pixel must have no horizontal ray component");
    test.expect_near(center_ray.y, 0.0F, 1.0e-6F,
                     "Principal-point pixel must have no vertical ray component");
    test.expect_near(center_ray.z, -1.0F, 1.0e-6F,
                     "Principal-point pixel must follow the negative camera z convention");
    const glm::vec3 right_ray = camera.pixel_to_unit_ray_world(60, 40);
    test.expect(right_ray.x > 0.0F,
                "Pixels right of the principal point must produce positive x rays");
    test.expect_throws<std::invalid_argument>(
        [&] { static_cast<void>(camera.pixel_to_unit_ray_world(100, 40)); },
        "Out-of-bounds calibrated pixels must be rejected");

    voxel4d::CameraIntrinsics invalid_intrinsics = intrinsics;
    invalid_intrinsics.focal_length_x_pixels = 0.0F;
    test.expect(!invalid_intrinsics.is_valid(), "Zero focal length must invalidate intrinsics");

    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(voxel4d::ObservationSynchronizer(-1, 2U)); },
        "Negative synchronization tolerance must be rejected");
    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(voxel4d::ObservationSynchronizer(0, 1U)); },
        "Synchronization must require at least two cameras");

    const voxel4d::ObservationSynchronizer synchronizer(10, 2U);
    const voxel4d::SynchronizedObservationGroup group =
        synchronizer.synchronize({voxel4d::TimedPixelObservation{"camera-b", 105, 4, 3},
                                  voxel4d::TimedPixelObservation{"camera-a", 100, 2, 1}});
    test.expect(group.reference_timestamp_nanoseconds == 105,
                "Even-sized synchronization group must use its upper temporal median");
    test.expect(group.observations.at(0).camera_id == "camera-a" &&
                    group.observations.at(1).camera_id == "camera-b",
                "Synchronization must order groups deterministically by camera identifier");

    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(
                synchronizer.synchronize({voxel4d::TimedPixelObservation{"camera-a", 100, 0, 0},
                                          voxel4d::TimedPixelObservation{"camera-a", 101, 0, 0}}));
        },
        "Synchronization must reject duplicate camera identifiers");
    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(
                synchronizer.synchronize({voxel4d::TimedPixelObservation{"camera-a", 100, 0, 0},
                                          voxel4d::TimedPixelObservation{"camera-b", 111, 0, 0}}));
        },
        "Synchronization must reject observations outside tolerance");
    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(
                synchronizer.synchronize({voxel4d::TimedPixelObservation{"camera-a", 100, -1, 0},
                                          voxel4d::TimedPixelObservation{"camera-b", 100, 0, 0}}));
        },
        "Synchronization must reject invalid pixel observations");

    return test.failures() == 0 ? 0 : 1;
}
