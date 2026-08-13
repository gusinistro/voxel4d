#include <memory>
#include <stdexcept>

#include "multi_sensor_fuser.h"
#include "test_support.h"

namespace {

voxel4d::SensorObservation make_rgbd_observation(const voxel4d::TimestampNanoseconds timestamp,
                                                 const glm::vec3& position) {
    voxel4d::SpatialSample sample{};
    sample.position_sensor_meters = position;
    sample.color_linear = glm::vec3(0.2F, 0.4F, 0.6F);
    sample.intensity = 0.5F;
    sample.temperature_kelvin = 300.0F;
    sample.confidence = 1.0F;
    sample.semantic_label = 1;

    voxel4d::SensorObservation observation{};
    observation.modality = voxel4d::SensorModality::kRgbd;
    observation.sensor_id = "camera-0";
    observation.timestamp_nanoseconds = timestamp;
    observation.observation_confidence = 1.0F;
    observation.spatial_samples.push_back(sample);
    return observation;
}

}  // namespace

int main() {
    voxel4d::test::TestContext test;

    const auto octree = std::make_shared<SparseVoxelOctree>(glm::vec3(0.0F), 10.0F, 2);
    const voxel4d::MultiSensorFuser fuser(octree);
    test.expect(fuser.fuse(make_rgbd_observation(0, glm::vec3(0.0F))) == 1U,
                "First temporal RGB-D observation must fuse");
    test.expect(fuser.fuse(make_rgbd_observation(1000000000, glm::vec3(1.0F, 0.0F, 0.0F))) == 1U,
                "Second temporally ordered RGB-D observation must fuse");

    const auto leaf = octree->search(glm::vec3(1.0F, 0.0F, 0.0F));
    test.expect(leaf != nullptr, "Temporally fused voxel must remain queryable");
    if (leaf) {
        test.expect_near(
            leaf->attribute.temporal_velocity_meters_per_second.x, 1.0F, 1.0e-5F,
            "Temporal voxel fusion must estimate x velocity from successive positions");
        test.expect_near(leaf->attribute.velocity.x, 0.0F, 1.0e-5F,
                         "Temporal velocity must not overwrite radar velocity storage");
        test.expect(leaf->attribute.last_observed_timestamp_nanoseconds == 1000000000,
                    "Temporal voxel fusion must retain the latest timestamp");
    }

    test.expect_throws<std::invalid_argument>(
        [&] { static_cast<void>(fuser.fuse(make_rgbd_observation(500000000, glm::vec3(0.5F)))); },
        "Temporal voxel fusion must reject stale observations of an existing modality");

    return test.failures() == 0 ? 0 : 1;
}
