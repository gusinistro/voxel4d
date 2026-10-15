#include "visual_odometry.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <limits>
#include <stdexcept>
#include <vector>

#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;
    voxel4d::VisualOdometryEstimator estimator;

    voxel4d::SensorPose expected_current_from_previous{};
    expected_current_from_previous.translation_meters = glm::vec3(1.5F, -2.0F, 0.75F);
    expected_current_from_previous.world_from_sensor =
        glm::angleAxis(glm::radians(35.0F), glm::normalize(glm::vec3(1.0F, -2.0F, 3.0F)));

    const std::vector<glm::vec3> previous_points{
        glm::vec3(-1.0F, -1.0F, -1.0F), glm::vec3(1.0F, -1.0F, -1.0F),
        glm::vec3(-1.0F, 1.0F, -1.0F),  glm::vec3(1.0F, 1.0F, -1.0F),
        glm::vec3(-1.0F, -1.0F, 1.0F),  glm::vec3(1.0F, -1.0F, 1.0F),
        glm::vec3(-1.0F, 1.0F, 1.0F),   glm::vec3(1.0F, 1.0F, 1.0F),
    };

    std::vector<voxel4d::PointCorrespondence3D> correspondences;
    correspondences.reserve(previous_points.size());
    for (const glm::vec3& previous_point : previous_points) {
        correspondences.push_back(voxel4d::PointCorrespondence3D{
            previous_point,
            expected_current_from_previous.transform_point_to_world(previous_point)});
    }

    const voxel4d::VisualOdometryResult estimate = estimator.estimate_rigid_motion(correspondences);
    test.expect(estimate.success, "Non-degenerate synthetic correspondences must estimate motion");
    test.expect(estimate.correspondence_count == correspondences.size(),
                "Visual odometry result must report correspondence count");
    test.expect_near(estimate.root_mean_square_error_meters, 0.0F, 1.0e-4F,
                     "Exact synthetic correspondences must have near-zero RMS error");

    const glm::vec3 probe_previous(0.25F, -0.5F, 1.25F);
    const glm::vec3 expected_probe =
        expected_current_from_previous.transform_point_to_world(probe_previous);
    const glm::vec3 estimated_probe =
        estimate.current_from_previous.transform_point_to_world(probe_previous);
    test.expect_near(estimated_probe.x, expected_probe.x, 1.0e-4F,
                     "Estimated rigid transform must recover probe x");
    test.expect_near(estimated_probe.y, expected_probe.y, 1.0e-4F,
                     "Estimated rigid transform must recover probe y");
    test.expect_near(estimated_probe.z, expected_probe.z, 1.0e-4F,
                     "Estimated rigid transform must recover probe z");

    const voxel4d::VisualOdometryResult insufficient =
        estimator.estimate_rigid_motion({correspondences.at(0), correspondences.at(1)});
    test.expect(!insufficient.success, "Fewer than three correspondences must not estimate motion");

    const std::vector<voxel4d::PointCorrespondence3D> degenerate{
        {glm::vec3(1.0F), glm::vec3(2.0F)},
        {glm::vec3(1.0F), glm::vec3(2.0F)},
        {glm::vec3(1.0F), glm::vec3(2.0F)},
    };
    test.expect(!estimator.estimate_rigid_motion(degenerate).success,
                "Coincident correspondences must be rejected as degenerate");

    const std::vector<voxel4d::PointCorrespondence3D> non_finite{
        {glm::vec3(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F), glm::vec3(0.0F)},
        {glm::vec3(0.0F), glm::vec3(0.0F)},
        {glm::vec3(1.0F), glm::vec3(1.0F)},
    };
    test.expect_throws<std::invalid_argument>(
        [&] { static_cast<void>(estimator.estimate_rigid_motion(non_finite)); },
        "Non-finite correspondences must be rejected");

    std::vector<voxel4d::PointCorrespondence3D> outlier_case{
        {{0.0F, 0.0F, 1.0F}, {0.25F, -0.10F, 1.05F}},
        {{0.5F, 0.0F, 1.2F}, {0.75F, -0.10F, 1.25F}},
        {{0.0F, 0.4F, 1.5F}, {0.25F, 0.30F, 1.55F}},
        {{-0.3F, -0.2F, 0.8F}, {-0.05F, -0.30F, 0.85F}},
        {{0.2F, 0.1F, 1.1F}, {4.0F, -3.0F, 2.0F}},
    };
    const voxel4d::VisualOdometryResult robust =
        estimator.estimate_rigid_motion_robust(outlier_case, 0.01F);
    test.expect(robust.success, "Robust rigid estimation must reject one gross outlier");
    test.expect(robust.correspondence_count == 4U,
                "Robust rigid estimation must report only geometric inliers");
    test.expect_near(robust.current_from_previous.translation_meters.x, 0.25F, 1.0e-4F,
                     "Robust rigid estimation must recover translation despite an outlier");
    test.expect(robust.root_mean_square_error_meters < 1.0e-4F,
                "Robust rigid estimation must report low inlier residual");

    return test.failures() == 0 ? 0 : 1;
}
