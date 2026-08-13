#include <chrono>
#include <cmath>
#include <exception>
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "acoustic_raytracer.h"
#include "doppler_simulator.h"
#include "execution_runtime.h"
#include "free_view_renderer.h"
#include "multi_sensor_fuser.h"
#include "multiview_calibration.h"
#include "multiview_geometry.h"
#include "object_tracker.h"
#include "octree.h"
#include "raytracer.h"
#include "recorded_observation_csv.h"
#include "semantic_inference.h"
#include "sensor_pose_timeline.h"
#include "spherical_harmonics.h"
#include "synthetic_data_generator.h"
#include "synthetic_sensor_adapter.h"
#include "temporal_voxel_map.h"
#include "visual_odometry.h"
#include "voxelizer.h"

namespace {

voxel4d::CalibratedCamera make_calibrated_camera(const std::string& camera_id,
                                                 const glm::vec3& position_meters) {
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = camera_id;
    camera.intrinsics = voxel4d::CameraIntrinsics{160, 120, 160.0F, 160.0F, 80.0F, 60.0F};
    camera.world_from_camera.translation_meters = position_meters;
    return camera;
}

}  // namespace

int main() {
    try {
        std::cout
            << "Voxel4D PoC\n"
            << "Temporal voxel fusion, multiview geometry, object tracks, free-view rendering, "
               "acoustics, and optional semantic interfaces\n\n";

        constexpr float kOctreeSizeMeters = 50.0F;
        constexpr int kOctreeDepth = 8;
        constexpr int kFramesToFuse = 3;
        constexpr TemporalVoxelMap::TimestampNanoseconds kFramePeriodNanoseconds = 100000000;
        const voxel4d::ExecutionRuntime serial_runtime(voxel4d::ExecutionBackend::kCpuSerial);
        const voxel4d::ExecutionRuntime parallel_runtime(voxel4d::ExecutionBackend::kCpuParallel,
                                                         2U);
        TemporalVoxelMap temporal_map(static_cast<std::size_t>(kFramesToFuse));
        voxel4d::SensorPoseTimeline pose_timeline(static_cast<std::size_t>(kFramesToFuse));
        std::cout << "[1/12] Runtime: "
                  << voxel4d::execution_backend_name(serial_runtime.info().active)
                  << "; CPU parallel available: "
                  << (parallel_runtime.info().active == voxel4d::ExecutionBackend::kCpuParallel
                          ? "yes"
                          : "no")
                  << '\n';

        SyntheticDataGenerator generator(640, 480);
        const std::string output_directory = "data";
        constexpr int kFrameCount = 10;
        generator.generate_moving_object_sequence(kFrameCount, output_directory);
        std::cout << "[2/12] Generated " << kFrameCount
                  << " deterministic RGB-D frames for two cameras\n";

        const std::vector<Camera> cameras = generator.generate_camera_setup(2);
        voxel4d::SyntheticSensorAdapter synthetic_sensor_adapter;
        std::size_t total_rgbd_samples = 0U;
        std::size_t total_multisensor_samples = 0U;
        std::shared_ptr<SparseVoxelOctree> latest_octree;
        std::vector<voxel4d::SensorObservation> latest_observations;
        for (int frame = 0; frame < kFramesToFuse; ++frame) {
            auto frame_octree = std::make_shared<SparseVoxelOctree>(
                glm::vec3(0.0F), kOctreeSizeMeters, kOctreeDepth);
            Voxelizer voxelizer(frame_octree);
            for (std::size_t camera_index = 0U; camera_index < cameras.size(); ++camera_index) {
                const std::string filename = output_directory + "/frame_" + std::to_string(frame) +
                                             "_cam_" + std::to_string(camera_index) + ".csv";
                total_rgbd_samples +=
                    voxelizer.voxelize_frame(cameras[camera_index], generator.load_frame(filename));
            }

            const TemporalVoxelMap::TimestampNanoseconds timestamp_nanoseconds =
                static_cast<TemporalVoxelMap::TimestampNanoseconds>(frame) *
                kFramePeriodNanoseconds;
            const voxel4d::SensorPose world_from_rig{};
            if (!pose_timeline.insert(timestamp_nanoseconds, world_from_rig)) {
                std::cerr << "[3/12] Pose timeline insertion failed\n";
                return 2;
            }

            const voxel4d::MultiSensorFuser fuser(frame_octree);
            std::vector<voxel4d::SensorObservation> observations =
                synthetic_sensor_adapter.generate_frame(timestamp_nanoseconds, world_from_rig,
                                                        glm::vec3(0.0F),
                                                        glm::vec3(0.5F, 0.0F, 0.0F));
            for (const voxel4d::SensorObservation& observation : observations) {
                total_multisensor_samples += fuser.fuse(observation);
            }

            if (!temporal_map.insert_snapshot(timestamp_nanoseconds, frame_octree)) {
                std::cerr << "[3/12] Temporal snapshot insertion failed\n";
                return 3;
            }
            latest_observations = std::move(observations);
            latest_octree = std::move(frame_octree);
        }

        const auto latest_snapshot = temporal_map.get_latest_snapshot();
        const auto fused_center_leaf = latest_octree->search(glm::vec3(0.0F));
        if (!latest_snapshot || latest_snapshot != latest_octree || !fused_center_leaf ||
            fused_center_leaf->attribute.source_modality_mask == 0U) {
            std::cerr << "[3/12] Temporal fusion or modality provenance validation failed\n";
            return 4;
        }
        std::cout << "[3/12] Fused " << total_rgbd_samples << " RGB-D samples and "
                  << total_multisensor_samples << " spatial sensor samples into "
                  << temporal_map.get_snapshot_count() << " snapshots\n";

        const voxel4d::CalibratedCamera left_camera =
            make_calibrated_camera("calibrated-left", glm::vec3(-1.0F, 0.0F, 0.0F));
        const voxel4d::CalibratedCamera right_camera =
            make_calibrated_camera("calibrated-right", glm::vec3(1.0F, 0.0F, 0.0F));
        const glm::vec3 multiview_target(0.0F, 0.0F, -5.0F);
        const glm::vec2 left_pixel =
            voxel4d::MultiviewGeometry::project_world_to_pixel(left_camera, multiview_target);
        const glm::vec2 right_pixel =
            voxel4d::MultiviewGeometry::project_world_to_pixel(right_camera, multiview_target);
        const voxel4d::ObservationSynchronizer synchronizer(5000000, 2U);
        const voxel4d::SynchronizedObservationGroup synchronized = synchronizer.synchronize(
            {voxel4d::TimedPixelObservation{"calibrated-right", 202000000,
                                            static_cast<int>(std::lround(right_pixel.x)),
                                            static_cast<int>(std::lround(right_pixel.y))},
             voxel4d::TimedPixelObservation{"calibrated-left", 200000000,
                                            static_cast<int>(std::lround(left_pixel.x)),
                                            static_cast<int>(std::lround(left_pixel.y))}});
        const voxel4d::TriangulatedPoint triangulated =
            voxel4d::MultiviewGeometry::triangulate_two_view(
                left_camera, synchronized.observations.at(0), right_camera,
                synchronized.observations.at(1));
        std::cout << "[4/12] Synchronized calibrated pair and triangulated point with "
                  << triangulated.mean_reprojection_error_pixels << " px mean reprojection error\n";

        voxel4d::ObjectTracker tracker(2.0F);
        const std::vector<voxel4d::ObjectTrackId> first_track_ids = tracker.update(
            200000000, {voxel4d::ObjectObservation{triangulated.position_world_meters,
                                                   glm::vec3(0.5F), 1, 0.95F}});
        const std::vector<voxel4d::ObjectTrackId> second_track_ids = tracker.update(
            300000000, {voxel4d::ObjectObservation{
                           triangulated.position_world_meters + glm::vec3(0.05F, 0.0F, 0.0F),
                           glm::vec3(0.5F), 1, 0.95F}});
        if (first_track_ids != second_track_ids) {
            std::cerr << "[5/12] Object tracker did not preserve identity\n";
            return 5;
        }
        std::cout << "[5/12] Retained semantic object track " << first_track_ids.front()
                  << " across temporal observations\n";

        const std::string replay_path = output_directory + "/recorded_observations.csv";
        if (!voxel4d::RecordedObservationCsv::write(replay_path, latest_observations)) {
            std::cerr << "[6/12] Recorded observation CSV write failed\n";
            return 6;
        }
        const std::vector<voxel4d::SensorObservation> replayed_observations =
            voxel4d::RecordedObservationCsv::read(replay_path);
        const voxel4d::RecordedLabelInferenceProvider semantic_provider;
        const voxel4d::SemanticInferenceResult semantic_result =
            semantic_provider.infer(replayed_observations.front().spatial_samples);
        std::cout << "[6/12] Replayed " << replayed_observations.size()
                  << " recorded sensor envelopes; optional provider emitted "
                  << semantic_result.predictions.size() << " existing annotations\n";

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
            std::cerr << "[7/12] Deterministic visual odometry estimation failed\n";
            return 7;
        }
        std::cout << "[7/12] Estimated synthetic rigid motion with "
                  << odometry.root_mean_square_error_meters << " m RMS error\n";

        VoxelRaytracer raytracer(latest_octree);
        const Ray test_ray{glm::vec3(-15.0F, 0.0F, 0.0F), glm::vec3(1.0F, 0.0F, 0.0F)};
        const RayHitResult hit = raytracer.trace_ray(test_ray, 100.0F);
        if (!hit.hit) {
            std::cerr << "[8/12] Ray traversal failed to find the reconstructed object\n";
            return 8;
        }
        const voxel4d::AcousticTraceResult acoustic =
            voxel4d::AcousticRaytracer(latest_octree)
                .trace_direct_path(glm::vec3(-15.0F, 0.0F, 0.0F), glm::vec3(15.0F, 0.0F, 0.0F));
        std::cout << "[8/12] DDA hit at " << hit.distance << " m; direct acoustic path is "
                  << (acoustic.blocked ? "blocked" : "clear") << '\n';

        voxel4d::SphericalHarmonicsL1 lighting;
        lighting.accumulate(glm::vec3(1.0F, 0.0F, 0.0F), glm::vec3(1.0F, 0.8F, 0.6F), 1.0F);
        const glm::vec3 forward_radiance = lighting.evaluate_clamped(glm::vec3(1.0F, 0.0F, 0.0F));
        DopplerSimulator doppler;
        std::vector<DopplerResult> samples;
        constexpr float kBaseFrequencyHz = 440.0F;
        doppler.sample_sound_doppler_field(latest_octree, glm::vec3(0.0F),
                                           glm::vec3(10.0F, 0.0F, 0.0F), kBaseFrequencyHz, samples);
        if (samples.empty()) {
            std::cerr << "[9/12] Doppler field sampling produced no receiver samples\n";
            return 9;
        }
        std::cout << "[9/12] L1 radiance " << forward_radiance.x << "; first Doppler frequency "
                  << (kBaseFrequencyHz + samples.front().frequency_shift_hz) << " Hz\n";

        voxel4d::CalibratedCamera free_view_camera =
            make_calibrated_camera("free-view", glm::vec3(0.0F, 0.0F, 12.0F));
        free_view_camera.intrinsics = voxel4d::CameraIntrinsics{96, 72, 80.0F, 80.0F, 48.0F, 36.0F};
        const voxel4d::FreeViewRenderer renderer(latest_octree);
        const auto serial_begin = std::chrono::steady_clock::now();
        const voxel4d::RenderedImage serial_render =
            renderer.render(free_view_camera, 100.0F, glm::vec3(0.02F), serial_runtime);
        const auto serial_end = std::chrono::steady_clock::now();
        const auto parallel_begin = std::chrono::steady_clock::now();
        const voxel4d::RenderedImage parallel_render =
            renderer.render(free_view_camera, 100.0F, glm::vec3(0.02F), parallel_runtime);
        const auto parallel_end = std::chrono::steady_clock::now();
        const std::string free_view_path = output_directory + "/free_view.ppm";
        if (!voxel4d::FreeViewRenderer::write_ppm(parallel_render, free_view_path) ||
            !serial_render.is_valid() || !parallel_render.is_valid()) {
            std::cerr << "[10/12] Free-view renderer output failed\n";
            return 10;
        }
        const auto serial_microseconds =
            std::chrono::duration_cast<std::chrono::microseconds>(serial_end - serial_begin)
                .count();
        const auto parallel_microseconds =
            std::chrono::duration_cast<std::chrono::microseconds>(parallel_end - parallel_begin)
                .count();
        std::cout << "[10/12] Rendered free viewpoint to " << free_view_path << " (serial "
                  << serial_microseconds << " us, parallel " << parallel_microseconds << " us)\n";

        const std::vector<voxel4d::ExecutionBackendCapability> capabilities =
            voxel4d::execution_backend_capabilities();
        std::cout << "[11/12] Reported " << capabilities.size()
                  << " explicit execution capabilities; unavailable accelerators use no hidden "
                     "fallback\n";
        std::cout << "[12/12] Integrated deterministic milestone completed successfully. Generated "
                     "artifacts are available in ./data.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Voxel4D PoC failed: " << error.what() << '\n';
        return 1;
    }
}
