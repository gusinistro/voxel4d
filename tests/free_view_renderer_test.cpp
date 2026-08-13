#include "free_view_renderer.h"

#include <filesystem>
#include <memory>
#include <stdexcept>

#include "test_support.h"

namespace {

voxel4d::CalibratedCamera make_camera() {
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = "virtual-render-camera";
    camera.intrinsics = voxel4d::CameraIntrinsics{5, 5, 5.0F, 5.0F, 2.0F, 2.0F};
    camera.world_from_camera.translation_meters = glm::vec3(0.0F, 0.0F, 2.0F);
    return camera;
}

}  // namespace

int main() {
    voxel4d::test::TestContext test;

    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(voxel4d::FreeViewRenderer(nullptr)); },
        "Free-view renderer must reject a null Octree");

    const auto octree = std::make_shared<SparseVoxelOctree>(glm::vec3(0.0F), 10.0F, 3);
    VoxelAttribute voxel{};
    voxel.density = 1.0F;
    voxel.color = glm::vec3(0.8F, 0.2F, 0.1F);
    test.expect(octree->insert(glm::vec3(0.0F, 0.0F, -2.0F), voxel),
                "Renderable occupied voxel insertion must succeed");

    const voxel4d::FreeViewRenderer renderer(octree);
    const voxel4d::CalibratedCamera camera = make_camera();
    const glm::vec3 background(0.1F, 0.1F, 0.1F);
    const voxel4d::RenderedImage serial_image = renderer.render(camera, 20.0F, background);
    const voxel4d::RenderedImage parallel_image =
        renderer.render(camera, 20.0F, background,
                        voxel4d::ExecutionRuntime(voxel4d::ExecutionBackend::kCpuParallel, 2U));
    test.expect(serial_image.is_valid(), "Serial free-view render must produce a valid image");
    test.expect(parallel_image.is_valid(), "Parallel free-view render must produce a valid image");
    const glm::vec3 center_pixel = serial_image.pixel(2, 2);
    test.expect_near(center_pixel.r, voxel.color.r, 1.0e-6F,
                     "Central virtual camera ray must recover voxel red channel");
    test.expect_near(center_pixel.g, voxel.color.g, 1.0e-6F,
                     "Central virtual camera ray must recover voxel green channel");
    test.expect_near(parallel_image.pixel(2, 2).r, center_pixel.r, 1.0e-6F,
                     "Serial and parallel free-view render must agree for independent pixels");
    test.expect_near(serial_image.pixel(0, 0).r, background.r, 1.0e-6F,
                     "Missed rays must preserve the requested background color");

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "voxel4d_free_view_renderer_test.ppm";
    test.expect(voxel4d::FreeViewRenderer::write_ppm(serial_image, output_path.string()),
                "Renderer must serialize valid output as PPM");
    test.expect(std::filesystem::file_size(output_path) > 20U,
                "Serialized PPM must contain a header and pixel payload");
    std::filesystem::remove(output_path);

    test.expect_throws<std::invalid_argument>(
        [&] { static_cast<void>(renderer.render(camera, 0.0F)); },
        "Renderer must reject a non-positive maximum ray distance");
    test.expect_throws<std::out_of_range>([&] { static_cast<void>(serial_image.pixel(5, 0)); },
                                          "Rendered image must reject out-of-bounds pixel lookup");

    return test.failures() == 0 ? 0 : 1;
}
