#include "sensor_pose.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <stdexcept>

#include "sensor_pose_timeline.h"
#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;

    const voxel4d::SensorPose identity{};
    test.expect(identity.is_valid(), "Identity sensor pose must be valid");
    const glm::vec3 point_sensor(1.0F, -2.0F, 3.0F);
    const glm::vec3 identity_world = identity.transform_point_to_world(point_sensor);
    test.expect_near(identity_world.x, point_sensor.x, 1.0e-6F,
                     "Identity pose must preserve point x");
    test.expect_near(identity_world.y, point_sensor.y, 1.0e-6F,
                     "Identity pose must preserve point y");
    test.expect_near(identity_world.z, point_sensor.z, 1.0e-6F,
                     "Identity pose must preserve point z");

    voxel4d::SensorPose rotated{};
    rotated.translation_meters = glm::vec3(2.0F, 0.0F, 0.0F);
    rotated.world_from_sensor = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0F, 0.0F, 1.0F));
    const glm::vec3 transformed = rotated.transform_point_to_world(glm::vec3(1.0F, 0.0F, 0.0F));
    test.expect_near(transformed.x, 2.0F, 1.0e-5F,
                     "Sensor pose rotation and translation must transform x");
    test.expect_near(transformed.y, 1.0F, 1.0e-5F,
                     "Sensor pose rotation and translation must transform y");
    test.expect_near(transformed.z, 0.0F, 1.0e-5F,
                     "Sensor pose rotation and translation must transform z");

    const glm::vec3 recovered = rotated.transform_point_to_sensor(transformed);
    test.expect_near(recovered.x, 1.0F, 1.0e-5F, "Inverse sensor transform must recover sensor x");
    test.expect_near(recovered.y, 0.0F, 1.0e-5F, "Inverse sensor transform must recover sensor y");
    test.expect_near(recovered.z, 0.0F, 1.0e-5F, "Inverse sensor transform must recover sensor z");

    voxel4d::SensorPose world_from_sensor{};
    world_from_sensor.translation_meters = glm::vec3(1.0F, 0.0F, 0.0F);
    voxel4d::SensorPose sensor_from_child{};
    sensor_from_child.translation_meters = glm::vec3(0.0F, 2.0F, 0.0F);
    const voxel4d::SensorPose world_from_child = world_from_sensor.compose(sensor_from_child);
    const glm::vec3 child_origin = world_from_child.transform_point_to_world(glm::vec3(0.0F));
    test.expect_near(child_origin.x, 1.0F, 1.0e-6F,
                     "Pose composition must preserve parent translation");
    test.expect_near(child_origin.y, 2.0F, 1.0e-6F,
                     "Pose composition must apply child translation");

    voxel4d::SensorPose invalid{};
    invalid.world_from_sensor = glm::quat(0.0F, 0.0F, 0.0F, 0.0F);
    test.expect(!invalid.is_valid(), "Zero-length pose rotation must be invalid");
    test.expect_throws<std::invalid_argument>([&] { static_cast<void>(invalid.normalized()); },
                                              "Invalid pose normalization must throw");

    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(voxel4d::SensorPoseTimeline(0)); },
        "Pose timeline must reject a zero retention capacity");

    voxel4d::SensorPoseTimeline timeline(2);
    test.expect(timeline.insert(100, identity), "First pose sample must be inserted");
    test.expect(!timeline.insert(100, identity), "Duplicate pose timestamp must be rejected");
    test.expect(!timeline.insert(99, identity), "Out-of-order pose timestamp must be rejected");
    test.expect_throws<std::invalid_argument>(
        [&] { static_cast<void>(timeline.insert(200, invalid)); },
        "Timeline must reject invalid poses");
    test.expect(timeline.size() == 1U, "Rejected pose samples must not change timeline size");

    test.expect(timeline.insert(200, rotated), "Newer pose sample must be inserted");
    test.expect(timeline.insert(300, world_from_child), "Later pose sample must be inserted");
    test.expect(timeline.size() == 2U, "Timeline must enforce bounded retention");
    test.expect(!timeline.at_or_before(199).has_value(),
                "Evicted pose samples must not be returned by historical lookup");
    const auto middle = timeline.at_or_before(250);
    test.expect(middle.has_value() && middle->timestamp_nanoseconds == 200,
                "Historical lookup must select the newest retained pose at or before the query");
    const auto latest = timeline.latest();
    test.expect(latest.has_value() && latest->timestamp_nanoseconds == 300,
                "Latest pose lookup must return the newest retained sample");

    return test.failures() == 0 ? 0 : 1;
}
