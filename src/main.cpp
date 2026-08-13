#include <exception>
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "acoustic_raytracer.h"
#include "doppler_simulator.h"
#include "execution_runtime.h"
#include "multi_sensor_fuser.h"
#include "octree.h"
#include "raytracer.h"
#include "sensor_pose_timeline.h"
#include "spherical_harmonics.h"
#include "synthetic_data_generator.h"
#include "synthetic_sensor_adapter.h"
#include "temporal_voxel_map.h"
#include "visual_odometry.h"
#include "voxelizer.h"

int main() {
    try {
        std::cout << "Voxel4D PoC\n"
                  << "Temporal voxel fusion, odometry, acoustic visibility, and low-frequency "
                     "lighting\n\n";

        constexpr float kOctreeSizeMeters = 50.0F;
        constexpr int kOctreeDepth = 8;
        constexpr int kFramesToFuse = 3;
        constexpr TemporalVoxelMap::TimestampNanoseconds kFramePeriodNanoseconds = 100000000;
        const voxel4d::ExecutionRuntime runtime(voxel4d::ExecutionBackend::kCpuSerial);
        TemporalVoxelMap temporal_map(static_cast<std::size_t>(kFramesToFuse));
        voxel4d::SensorPoseTimeline pose_timeline(static_cast<std::size_t>(kFramesToFuse));
        std::cout << "[1/8] Runtime: " << voxel4d::execution_backend_name(runtime.info().active)
                  << "; temporal map retains " << kFramesToFuse << " SVO snapshots\n";

        SyntheticDataGenerator generator(640, 480);
        const std::string output_directory = "data";
        constexpr int kFrameCount = 10;
        generator.generate_moving_object_sequence(kFrameCount, output_directory);
        std::cout << "[2/8] Generated " << kFrameCount
                  << " deterministic RGB-D frames for two cameras\n";

        const std::vector<Camera> cameras = generator.generate_camera_setup(2);
        voxel4d::SyntheticSensorAdapter synthetic_sensor_adapter;
        std::size_t total_rgbd_samples = 0U;
        std::size_t total_multisensor_samples = 0U;
        std::shared_ptr<SparseVoxelOctree> latest_octree;
        for (int frame = 0; frame < kFramesToFuse; ++frame) {
            auto frame_octree = std::make_shared<SparseVoxelOctree>(
                glm::vec3(0.0F), kOctreeSizeMeters, kOctreeDepth);
            Voxelizer voxelizer(frame_octree);
            for (std::size_t camera_index = 0U; camera_index < cameras.size(); ++camera_index) {
                const std::string filename = output_directory + "/frame_" + std::to_string(frame) +
                                             "_cam_" + std::to_string(camera_index) + ".csv";
                const std::vector<PixelData> pixels = generator.load_frame(filename);
                total_rgbd_samples += voxelizer.voxelize_frame(cameras[camera_index], pixels);
            }

            const TemporalVoxelMap::TimestampNanoseconds timestamp_nanoseconds =
                static_cast<TemporalVoxelMap::TimestampNanoseconds>(frame) *
                kFramePeriodNanoseconds;
            const voxel4d::SensorPose world_from_rig{};
            if (!pose_timeline.insert(timestamp_nanoseconds, world_from_rig)) {
                std::cerr << "[3/8] Pose timeline insertion failed\n";
                return 2;
            }

            const voxel4d::MultiSensorFuser fuser(frame_octree);
            const std::vector<voxel4d::SensorObservation> observations =
                synthetic_sensor_adapter.generate_frame(timestamp_nanoseconds, world_from_rig,
                                                        glm::vec3(0.0F),
                                                        glm::vec3(0.5F, 0.0F, 0.0F));
            for (const voxel4d::SensorObservation& observation : observations) {
                total_multisensor_samples += fuser.fuse(observation);
            }

            if (!temporal_map.insert_snapshot(timestamp_nanoseconds, frame_octree)) {
                std::cerr << "[3/8] Temporal snapshot insertion failed\n";
                return 3;
            }
            latest_octree = std::move(frame_octree);
        }

        const auto latest_snapshot = temporal_map.get_latest_snapshot();
        if (!latest_snapshot || latest_snapshot != latest_octree) {
            std::cerr << "[3/8] Temporal map did not retain the latest spatial snapshot\n";
            return 4;
        }
        const auto fused_center_leaf = latest_octree->search(glm::vec3(0.0F));
        if (!fused_center_leaf || fused_center_leaf->attribute.source_modality_mask == 0U) {
            std::cerr << "[3/8] Multisensor fusion did not retain modality provenance\n";
            return 5;
        }
        std::cout << "[3/8] Fused " << total_rgbd_samples << " RGB-D samples and "
                  << total_multisensor_samples << " simulated spatial sensor samples into "
                  << temporal_map.get_snapshot_count() << " snapshots; center modality mask "
                  << fused_center_leaf->attribute.source_modality_mask << '\n';

        voxel4d::SensorPose known_current_from_previous{};
        known_current_from_previous.translation_meters = glm::vec3(0.05F, 0.0F, 0.0F);
        const std::vector<glm::vec3> odometry_points{
            glm::vec3(-1.0F, -1.0F, 0.0F),
            glm::vec3(1.0F, -1.0F, 0.0F),
            glm::vec3(-1.0F, 1.0F, 0.0F),
            glm::vec3(1.0F, 1.0F, 1.0F),
        };
        std::vector<voxel4d::PointCorrespondence3D> correspondences;
        correspondences.reserve(odometry_points.size());
        for (const glm::vec3& point : odometry_points) {
            correspondences.push_back(voxel4d::PointCorrespondence3D{
                point, known_current_from_previous.transform_point_to_world(point)});
        }
        const voxel4d::VisualOdometryResult odometry =
            voxel4d::VisualOdometryEstimator().estimate_rigid_motion(correspondences);
        if (!odometry.success) {
            std::cerr << "[4/8] Deterministic visual odometry estimation failed\n";
            return 6;
        }
        std::cout << "[4/8] Estimated synthetic rigid motion with "
                  << odometry.root_mean_square_error_meters << " m RMS error\n";

        VoxelRaytracer raytracer(latest_octree);
        const Ray test_ray{glm::vec3(-15.0F, 0.0F, 0.0F), glm::vec3(1.0F, 0.0F, 0.0F)};
        const RayHitResult hit = raytracer.trace_ray(test_ray, 100.0F);
        if (!hit.hit) {
            std::cerr << "[5/8] Ray traversal failed to find the reconstructed object\n";
            return 7;
        }
        std::cout << "[5/8] DDA hit at " << hit.distance << " m\n";

        const voxel4d::AcousticTraceResult acoustic =
            voxel4d::AcousticRaytracer(latest_octree)
                .trace_direct_path(glm::vec3(-15.0F, 0.0F, 0.0F), glm::vec3(15.0F, 0.0F, 0.0F));
        std::cout << "[6/8] Direct acoustic path: " << acoustic.path_length_meters << " m, "
                  << acoustic.travel_time_seconds << " s, "
                  << (acoustic.blocked ? "blocked" : "clear") << '\n';

        voxel4d::SphericalHarmonicsL1 lighting;
        lighting.accumulate(glm::vec3(1.0F, 0.0F, 0.0F), glm::vec3(1.0F, 0.8F, 0.6F), 1.0F);
        const glm::vec3 forward_radiance = lighting.evaluate_clamped(glm::vec3(1.0F, 0.0F, 0.0F));
        std::cout << "[7/8] L1 spherical-harmonic forward radiance: (" << forward_radiance.x << ", "
                  << forward_radiance.y << ", " << forward_radiance.z << ")\n";

        DopplerSimulator doppler;
        std::vector<DopplerResult> samples;
        constexpr float kBaseFrequencyHz = 440.0F;
        doppler.sample_sound_doppler_field(latest_octree, glm::vec3(0.0F),
                                           glm::vec3(10.0F, 0.0F, 0.0F), kBaseFrequencyHz, samples);
        if (samples.empty()) {
            std::cerr << "[8/8] Doppler field sampling produced no receiver samples\n";
            return 8;
        }
        std::cout << "[8/8] Sampled " << samples.size()
                  << " receiver positions; first observed frequency: "
                  << (kBaseFrequencyHz + samples.front().frequency_shift_hz) << " Hz\n\n";

        std::cout << "PoC completed successfully. Generated data is available in ./data.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Voxel4D PoC failed: " << error.what() << '\n';
        return 1;
    }
}
