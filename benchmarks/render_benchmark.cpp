#include <chrono>
#include <cstddef>
#include <iostream>
#include <memory>

#include "execution_runtime.h"
#include "free_view_renderer.h"
#include "octree.h"

namespace {

voxel4d::CalibratedCamera make_camera() {
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = "benchmark-camera";
    camera.intrinsics = voxel4d::CameraIntrinsics{96, 72, 80.0F, 80.0F, 48.0F, 36.0F};
    camera.world_from_camera.translation_meters = glm::vec3(0.0F, 0.0F, 12.0F);
    return camera;
}

long long measure_microseconds(const voxel4d::FreeViewRenderer& renderer,
                               const voxel4d::CalibratedCamera& camera,
                               const voxel4d::ExecutionRuntime& runtime,
                               const std::size_t iterations) {
    const auto begin = std::chrono::steady_clock::now();
    for (std::size_t iteration = 0U; iteration < iterations; ++iteration) {
        static_cast<void>(renderer.render(camera, 100.0F, glm::vec3(0.02F), runtime));
    }
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
}

}  // namespace

int main() {
    constexpr std::size_t kIterations = 3U;
    const auto octree = std::make_shared<SparseVoxelOctree>(glm::vec3(0.0F), 50.0F, 8);
    VoxelAttribute voxel{};
    voxel.density = 1.0F;
    voxel.color = glm::vec3(0.8F, 0.3F, 0.2F);
    if (!octree->insert(glm::vec3(0.0F), voxel)) {
        std::cerr << "benchmark setup insertion failed\n";
        return 1;
    }

    const voxel4d::FreeViewRenderer renderer(octree);
    const voxel4d::CalibratedCamera camera = make_camera();
    const voxel4d::ExecutionRuntime serial(voxel4d::ExecutionBackend::kCpuSerial);
    const voxel4d::ExecutionRuntime parallel(voxel4d::ExecutionBackend::kCpuParallel, 2U);
    const long long serial_microseconds =
        measure_microseconds(renderer, camera, serial, kIterations);
    const long long parallel_microseconds =
        measure_microseconds(renderer, camera, parallel, kIterations);
    const std::size_t pixels_per_render = static_cast<std::size_t>(camera.intrinsics.width_pixels) *
                                          static_cast<std::size_t>(camera.intrinsics.height_pixels);
    const double serial_megapixels_per_second =
        static_cast<double>(pixels_per_render * kIterations) /
        static_cast<double>(serial_microseconds);
    const double parallel_megapixels_per_second =
        static_cast<double>(pixels_per_render * kIterations) /
        static_cast<double>(parallel_microseconds);

    std::cout << "backend,iterations,total_microseconds,megapixels_per_second\n";
    std::cout << "cpu_serial," << kIterations << ',' << serial_microseconds << ','
              << serial_megapixels_per_second << '\n';
    std::cout << "cpu_parallel," << kIterations << ',' << parallel_microseconds << ','
              << parallel_megapixels_per_second << '\n';
    return 0;
}
