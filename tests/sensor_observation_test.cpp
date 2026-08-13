#include "sensor_observation.h"

#include <limits>
#include <stdexcept>
#include <vector>

#include "synthetic_sensor_adapter.h"
#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;

    voxel4d::SpatialSample sample{};
    sample.position_sensor_meters = glm::vec3(1.0F, 2.0F, 3.0F);
    sample.color_linear = glm::vec3(0.5F);
    sample.intensity = 0.75F;
    sample.temperature_kelvin = 300.0F;
    sample.velocity_sensor_meters_per_second = glm::vec3(1.0F, 0.0F, 0.0F);
    sample.confidence = 0.9F;
    test.expect(sample.is_valid(), "Finite spatial sample with bounded confidence must be valid");
    sample.confidence = 1.1F;
    test.expect(!sample.is_valid(), "Spatial sample confidence above one must be invalid");
    sample.confidence = 0.9F;

    voxel4d::SensorObservation rgbd{};
    rgbd.modality = voxel4d::SensorModality::kRgbd;
    rgbd.sensor_id = "camera-0";
    rgbd.observation_confidence = 0.9F;
    rgbd.spatial_samples.push_back(sample);
    test.expect(rgbd.is_valid(), "RGB-D observation with one spatial sample must be valid");
    rgbd.spatial_samples.clear();
    test.expect(!rgbd.is_valid(), "Spatial observation without samples must be invalid");

    voxel4d::SensorObservation imu{};
    imu.modality = voxel4d::SensorModality::kImu;
    imu.sensor_id = "imu-0";
    imu.observation_confidence = 0.95F;
    imu.imu_sample = voxel4d::ImuSample{glm::vec3(0.0F), glm::vec3(0.0F, 0.0F, 0.1F)};
    test.expect(imu.is_valid(), "IMU observation with one valid inertial sample must be valid");
    imu.spatial_samples.push_back(sample);
    test.expect(!imu.is_valid(), "IMU observation with spatial samples must be invalid");
    test.expect(
        std::string(voxel4d::sensor_modality_name(voxel4d::SensorModality::kLidar)) == "LiDAR",
        "Sensor modality names must expose stable human-readable labels");

    voxel4d::SyntheticSensorAdapter adapter;
    const voxel4d::SensorPose world_from_rig{};
    const glm::vec3 center_world(3.0F, -2.0F, 5.0F);
    const glm::vec3 velocity_world(1.0F, 0.5F, -0.25F);
    const std::vector<voxel4d::SensorObservation> observations =
        adapter.generate_frame(123456789, world_from_rig, center_world, velocity_world);
    test.expect(observations.size() == 5U,
                "Synthetic adapter must produce one observation for each modality");

    std::size_t spatial_observation_count = 0U;
    bool found_imu = false;
    for (const voxel4d::SensorObservation& observation : observations) {
        test.expect(observation.is_valid(), "Each synthetic observation must satisfy its contract");
        test.expect(observation.timestamp_nanoseconds == 123456789,
                    "Synthetic observations must preserve the supplied timestamp");
        if (observation.modality == voxel4d::SensorModality::kImu) {
            found_imu = true;
            test.expect(observation.imu_sample.has_value() && observation.spatial_samples.empty(),
                        "Synthetic IMU must contain inertial data only");
            continue;
        }

        ++spatial_observation_count;
        const voxel4d::SpatialSample& generated_sample = observation.spatial_samples.front();
        const glm::vec3 recovered_center = observation.world_from_sensor.transform_point_to_world(
            generated_sample.position_sensor_meters);
        const glm::vec3 recovered_velocity =
            observation.world_from_sensor.transform_vector_to_world(
                generated_sample.velocity_sensor_meters_per_second);
        test.expect_near(recovered_center.x, center_world.x, 1.0e-5F,
                         "Spatial adapter must preserve world point x through sensor pose");
        test.expect_near(recovered_center.y, center_world.y, 1.0e-5F,
                         "Spatial adapter must preserve world point y through sensor pose");
        test.expect_near(recovered_center.z, center_world.z, 1.0e-5F,
                         "Spatial adapter must preserve world point z through sensor pose");
        test.expect_near(recovered_velocity.x, velocity_world.x, 1.0e-5F,
                         "Spatial adapter must preserve world velocity x through sensor pose");
    }
    test.expect(spatial_observation_count == 4U && found_imu,
                "Synthetic adapter must provide four spatial modalities and one IMU modality");

    voxel4d::SensorPose invalid_pose{};
    invalid_pose.world_from_sensor = glm::quat(0.0F, 0.0F, 0.0F, 0.0F);
    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(
                adapter.generate_frame(0, invalid_pose, center_world, velocity_world));
        },
        "Synthetic adapter must reject invalid rig poses");
    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(adapter.generate_frame(
                0, world_from_rig, glm::vec3(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F),
                velocity_world));
        },
        "Synthetic adapter must reject non-finite object positions");

    return test.failures() == 0 ? 0 : 1;
}
