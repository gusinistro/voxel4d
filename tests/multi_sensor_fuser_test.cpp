#include "multi_sensor_fuser.h"

#include <memory>
#include <stdexcept>
#include <vector>

#include "synthetic_sensor_adapter.h"
#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;

    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(voxel4d::MultiSensorFuser(nullptr)); },
        "Multi-sensor fuser must reject a null Octree");

    const auto octree = std::make_shared<SparseVoxelOctree>(glm::vec3(0.0F), 20.0F, 4);
    const voxel4d::MultiSensorFuser fuser(octree);
    voxel4d::SyntheticSensorAdapter adapter;
    const glm::vec3 object_center(3.0F, -2.0F, 5.0F);
    const glm::vec3 object_velocity(1.0F, 0.5F, -0.25F);
    const std::vector<voxel4d::SensorObservation> observations =
        adapter.generate_frame(1000, voxel4d::SensorPose{}, object_center, object_velocity);

    std::size_t total_fused = 0U;
    for (const voxel4d::SensorObservation& observation : observations) {
        total_fused += fuser.fuse(observation);
    }
    test.expect(total_fused == 4U, "One synthetic sample from each spatial modality must be fused");

    const auto leaf = octree->search(object_center);
    test.expect(leaf != nullptr,
                "Fused world-space location must be represented by an Octree leaf");
    if (leaf) {
        const std::uint32_t expected_mask =
            voxel4d::MultiSensorFuser::modality_mask(voxel4d::SensorModality::kRgbd) |
            voxel4d::MultiSensorFuser::modality_mask(voxel4d::SensorModality::kLidar) |
            voxel4d::MultiSensorFuser::modality_mask(voxel4d::SensorModality::kRadar) |
            voxel4d::MultiSensorFuser::modality_mask(voxel4d::SensorModality::kThermal);
        test.expect(leaf->attribute.density > 0.0F, "Fused leaf must be occupied");
        test.expect(leaf->attribute.confidence > 0.0F && leaf->attribute.confidence <= 1.0F,
                    "Fused confidence must remain in the unit interval");
        test.expect(leaf->attribute.occupancy_observation_count == 4U &&
                        leaf->attribute.occupancy_log_odds > 0.0F &&
                        occupancy_probability(leaf->attribute.occupancy_log_odds) > 0.5F,
                    "Occupied sensor hits must accumulate positive bounded probabilistic evidence");
        test.expect(leaf->attribute.source_modality_mask == expected_mask,
                    "Fused leaf must retain all contributing modality bits");
        test.expect(leaf->attribute.last_observed_timestamp_nanoseconds == 1000,
                    "Fused leaf must retain the latest observation timestamp");
        test.expect_near(leaf->attribute.color.x, 0.8F, 1.0e-5F,
                         "RGB-D fusion must retain linear color attributes");
        test.expect_near(leaf->attribute.velocity.x, object_velocity.x, 1.0e-5F,
                         "Radar fusion must transform and retain world velocity x");
        test.expect(leaf->attribute.temperature > 296.15F,
                    "Thermal fusion must update the temperature attribute");
    }

    voxel4d::SensorObservation invalid{};
    invalid.sensor_id = "invalid";
    invalid.observation_confidence = 1.0F;
    test.expect_throws<std::invalid_argument>([&] { static_cast<void>(fuser.fuse(invalid)); },
                                              "Fuser must reject invalid observation envelopes");

    voxel4d::SensorObservation out_of_bounds = observations.front();
    out_of_bounds.world_from_sensor.translation_meters = glm::vec3(100.0F, 0.0F, 0.0F);
    test.expect(fuser.fuse(out_of_bounds) == 0U,
                "Fuser must ignore spatial samples outside the Octree bounds");

    return test.failures() == 0 ? 0 : 1;
}
