#include "recorded_observation_csv.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "test_support.h"

namespace {

voxel4d::SpatialSample make_spatial_sample(const float x) {
    voxel4d::SpatialSample sample{};
    sample.position_sensor_meters = glm::vec3(x, 1.0F, -2.0F);
    sample.color_linear = glm::vec3(0.2F, 0.4F, 0.6F);
    sample.intensity = 0.8F;
    sample.temperature_kelvin = 300.0F;
    sample.velocity_sensor_meters_per_second = glm::vec3(1.0F, 0.0F, 0.0F);
    sample.confidence = 0.9F;
    sample.semantic_label = 7;
    return sample;
}

}  // namespace

int main() {
    voxel4d::test::TestContext test;

    voxel4d::SensorObservation lidar{};
    lidar.modality = voxel4d::SensorModality::kLidar;
    lidar.sensor_id = "recorded-lidar";
    lidar.timestamp_nanoseconds = 123456789;
    lidar.world_from_sensor.translation_meters = glm::vec3(1.0F, 2.0F, 3.0F);
    lidar.observation_confidence = 0.95F;
    lidar.spatial_samples = {make_spatial_sample(1.0F), make_spatial_sample(2.0F)};

    voxel4d::SensorObservation imu{};
    imu.modality = voxel4d::SensorModality::kImu;
    imu.sensor_id = "recorded-imu";
    imu.timestamp_nanoseconds = 123456790;
    imu.observation_confidence = 0.99F;
    imu.imu_sample = voxel4d::ImuSample{glm::vec3(0.0F, 9.81F, 0.0F), glm::vec3(0.1F)};

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "voxel4d_recorded_observations_test.csv";
    test.expect(voxel4d::RecordedObservationCsv::write(output_path.string(), {lidar, imu}),
                "Recorded observation adapter must serialize valid observations");
    const std::vector<voxel4d::SensorObservation> replayed =
        voxel4d::RecordedObservationCsv::read(output_path.string());
    test.expect(replayed.size() == 2U,
                "Recorded observation reader must reconstruct spatial and IMU envelopes");
    if (replayed.size() == 2U) {
        test.expect(
            replayed.at(0).modality == voxel4d::SensorModality::kLidar &&
                replayed.at(0).spatial_samples.size() == 2U,
            "Adjacent spatial rows with shared metadata must reconstruct one scan observation");
        test.expect_near(replayed.at(0).spatial_samples.at(1).position_sensor_meters.x, 2.0F,
                         1.0e-6F, "Recorded spatial sample positions must round-trip");
        test.expect(replayed.at(1).modality == voxel4d::SensorModality::kImu &&
                        replayed.at(1).imu_sample.has_value(),
                    "Recorded IMU rows must reconstruct an inertial observation");
        test.expect_near(replayed.at(1).imu_sample->linear_acceleration_meters_per_second_squared.y,
                         9.81F, 1.0e-5F, "Recorded IMU acceleration must round-trip");
    }
    std::filesystem::remove(output_path);

    const std::filesystem::path malformed_path =
        std::filesystem::temp_directory_path() / "voxel4d_recorded_observations_bad.csv";
    {
        std::ofstream malformed(malformed_path);
        malformed << "bad-header\n";
    }
    test.expect_throws<std::runtime_error>(
        [&] { static_cast<void>(voxel4d::RecordedObservationCsv::read(malformed_path.string())); },
        "Recorded observation reader must reject an unexpected CSV header");
    std::filesystem::remove(malformed_path);

    voxel4d::SensorObservation invalid = lidar;
    invalid.sensor_id = "contains,comma";
    test.expect(!voxel4d::RecordedObservationCsv::write(output_path.string(), {invalid}),
                "Recorded observation writer must reject unsupported unescaped delimiters");

    return test.failures() == 0 ? 0 : 1;
}
