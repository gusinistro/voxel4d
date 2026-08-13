#include "multiview_geometry.h"

#include <stdexcept>

#include "test_support.h"

namespace {

voxel4d::CalibratedCamera make_camera(const std::string& camera_id, const glm::vec3& position) {
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = camera_id;
    camera.intrinsics = voxel4d::CameraIntrinsics{100, 80, 100.0F, 100.0F, 50.0F, 40.0F};
    camera.world_from_camera.translation_meters = position;
    return camera;
}

}  // namespace

int main() {
    voxel4d::test::TestContext test;

    const voxel4d::CalibratedCamera left = make_camera("left", glm::vec3(-1.0F, 0.0F, 0.0F));
    const voxel4d::CalibratedCamera right = make_camera("right", glm::vec3(1.0F, 0.0F, 0.0F));
    const glm::vec3 known_point(0.0F, 0.0F, -5.0F);
    const glm::vec2 left_pixel =
        voxel4d::MultiviewGeometry::project_world_to_pixel(left, known_point);
    const glm::vec2 right_pixel =
        voxel4d::MultiviewGeometry::project_world_to_pixel(right, known_point);
    test.expect_near(left_pixel.x, 70.0F, 1.0e-5F,
                     "Left-camera projection must respect baseline and focal length");
    test.expect_near(right_pixel.x, 30.0F, 1.0e-5F,
                     "Right-camera projection must respect baseline and focal length");
    test.expect_near(left_pixel.y, 40.0F, 1.0e-5F,
                     "Centered world point must project to the vertical principal point");

    const voxel4d::TriangulatedPoint estimate = voxel4d::MultiviewGeometry::triangulate_two_view(
        left, voxel4d::TimedPixelObservation{"left", 100, 70, 40}, right,
        voxel4d::TimedPixelObservation{"right", 102, 30, 40});
    test.expect_near(estimate.position_world_meters.x, known_point.x, 1.0e-5F,
                     "Triangulation must recover world x for a stereo target");
    test.expect_near(estimate.position_world_meters.y, known_point.y, 1.0e-5F,
                     "Triangulation must recover world y for a stereo target");
    test.expect_near(estimate.position_world_meters.z, known_point.z, 1.0e-5F,
                     "Triangulation must recover world z for a stereo target");
    test.expect_near(estimate.ray_separation_meters, 0.0F, 1.0e-5F,
                     "Exact stereo rays must meet without separation");
    test.expect_near(estimate.mean_reprojection_error_pixels, 0.0F, 1.0e-5F,
                     "Exact stereo rays must have zero reprojection error");

    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(voxel4d::MultiviewGeometry::triangulate_two_view(
                left, voxel4d::TimedPixelObservation{"left", 100, 50, 40}, right,
                voxel4d::TimedPixelObservation{"right", 100, 50, 40}));
        },
        "Parallel camera rays must be rejected");
    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(voxel4d::MultiviewGeometry::triangulate_two_view(
                left, voxel4d::TimedPixelObservation{"right", 100, 70, 40}, right,
                voxel4d::TimedPixelObservation{"right", 100, 30, 40}));
        },
        "Observations must match their calibrated camera identifiers");
    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(voxel4d::MultiviewGeometry::project_world_to_pixel(
                left, glm::vec3(0.0F, 0.0F, 1.0F)));
        },
        "Points behind the camera must be rejected during reprojection");

    return test.failures() == 0 ? 0 : 1;
}
