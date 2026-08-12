#include <exception>
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "doppler_simulator.h"
#include "octree.h"
#include "raytracer.h"
#include "synthetic_data_generator.h"
#include "temporal_voxel_map.h"
#include "voxelizer.h"

int main() {
    try {
        std::cout << "Voxel4D PoC\n"
                  << "Sparse voxel reconstruction, temporal snapshots, DDA traversal, and Doppler "
                     "field sampling\n\n";

        constexpr float kOctreeSizeMeters = 50.0F;
        constexpr int kOctreeDepth = 8;
        constexpr int kFramesToFuse = 3;
        constexpr TemporalVoxelMap::TimestampNanoseconds kFramePeriodNanoseconds = 100000000;
        TemporalVoxelMap temporal_map(static_cast<std::size_t>(kFramesToFuse));
        std::cout << "[1/6] Temporal map initialized: " << kFramesToFuse << " retained snapshots, "
                  << kOctreeSizeMeters << " m Octree extent, depth " << kOctreeDepth << '\n';

        SyntheticDataGenerator generator(640, 480);
        const std::string output_directory = "data";
        constexpr int kFrameCount = 10;
        generator.generate_moving_object_sequence(kFrameCount, output_directory);
        std::cout << "[2/6] Generated " << kFrameCount
                  << " synthetic RGB-D frames for two cameras\n";

        const std::vector<Camera> cameras = generator.generate_camera_setup(2);
        std::size_t total_samples_inserted = 0;
        std::shared_ptr<SparseVoxelOctree> latest_octree;
        for (int frame = 0; frame < kFramesToFuse; ++frame) {
            auto frame_octree = std::make_shared<SparseVoxelOctree>(
                glm::vec3(0.0F), kOctreeSizeMeters, kOctreeDepth);
            Voxelizer voxelizer(frame_octree);

            for (std::size_t camera_index = 0; camera_index < cameras.size(); ++camera_index) {
                const std::string filename = output_directory + "/frame_" + std::to_string(frame) +
                                             "_cam_" + std::to_string(camera_index) + ".csv";
                const std::vector<PixelData> pixels = generator.load_frame(filename);
                total_samples_inserted += voxelizer.voxelize_frame(cameras[camera_index], pixels);
            }

            const TemporalVoxelMap::TimestampNanoseconds timestamp_nanoseconds =
                static_cast<TemporalVoxelMap::TimestampNanoseconds>(frame) *
                kFramePeriodNanoseconds;
            if (!temporal_map.insert_snapshot(timestamp_nanoseconds, frame_octree)) {
                std::cerr << "[3/6] Temporal snapshot insertion failed\n";
                return 2;
            }
            latest_octree = std::move(frame_octree);
        }

        const auto latest_snapshot = temporal_map.get_latest_snapshot();
        if (!latest_snapshot || latest_snapshot != latest_octree) {
            std::cerr << "[3/6] Temporal map did not retain the latest spatial snapshot\n";
            return 3;
        }
        std::cout << "[3/6] Fused " << total_samples_inserted << " RGB-D samples into "
                  << temporal_map.get_snapshot_count() << " temporal snapshots; latest has "
                  << latest_octree->get_node_count() << " nodes and "
                  << latest_octree->get_leaf_count() << " leaves\n";

        VoxelRaytracer raytracer(latest_octree);
        Ray test_ray{};
        test_ray.origin = glm::vec3(-15.0F, 0.0F, 0.0F);
        test_ray.direction = glm::vec3(1.0F, 0.0F, 0.0F);
        const RayHitResult hit = raytracer.trace_ray(test_ray, 100.0F);
        if (!hit.hit) {
            std::cerr << "[4/6] Ray traversal failed to find the reconstructed object\n";
            return 4;
        }
        std::cout << "[4/6] DDA hit at " << hit.distance << " m: (" << hit.hit_point.x << ", "
                  << hit.hit_point.y << ", " << hit.hit_point.z << ")\n";

        DopplerSimulator doppler;
        std::vector<DopplerResult> samples;
        constexpr float kBaseFrequencyHz = 440.0F;
        doppler.sample_sound_doppler_field(latest_octree, glm::vec3(0.0F),
                                           glm::vec3(10.0F, 0.0F, 0.0F), kBaseFrequencyHz, samples);
        if (samples.empty()) {
            std::cerr << "[5/6] Doppler field sampling produced no receiver samples\n";
            return 5;
        }
        std::cout << "[5/6] Sampled " << samples.size()
                  << " receiver positions; first observed frequency: "
                  << (kBaseFrequencyHz + samples.front().frequency_shift_hz) << " Hz\n";

        std::cout << "[6/6] PoC completed successfully. Generated data is available in ./data.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Voxel4D PoC failed: " << error.what() << '\n';
        return 1;
    }
}
