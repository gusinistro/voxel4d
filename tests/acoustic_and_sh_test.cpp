#include <memory>
#include <stdexcept>

#include "acoustic_raytracer.h"
#include "doppler_simulator.h"
#include "spherical_harmonics.h"
#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;

    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(voxel4d::AcousticRaytracer(nullptr)); },
        "Acoustic ray tracer must reject a null Octree");

    const auto octree = std::make_shared<SparseVoxelOctree>(glm::vec3(0.0F), 20.0F, 4);
    voxel4d::AcousticRaytracer acoustic_raytracer(octree);
    const glm::vec3 source(-8.0F, 0.0F, 0.0F);
    const glm::vec3 receiver(8.0F, 0.0F, 0.0F);
    const voxel4d::AcousticTraceResult clear_path =
        acoustic_raytracer.trace_direct_path(source, receiver);
    test.expect(!clear_path.blocked, "Unoccupied direct acoustic path must remain unblocked");
    test.expect_near(clear_path.path_length_meters, 16.0F, 1.0e-6F,
                     "Acoustic trace must report source-receiver distance");
    test.expect_near(clear_path.travel_time_seconds, 16.0F / voxel4d::kSpeedOfSoundMetersPerSecond,
                     1.0e-6F, "Acoustic trace must use the configured speed of sound");
    test.expect_near(clear_path.transmission_gain, 1.0F, 1.0e-6F,
                     "Clear direct acoustic path must have unit transmission gain");

    VoxelAttribute blocker{};
    blocker.density = 1.0F;
    test.expect(octree->insert(glm::vec3(0.0F), blocker), "Blocker voxel insertion must succeed");
    const voxel4d::AcousticTraceResult blocked_path =
        acoustic_raytracer.trace_direct_path(source, receiver);
    test.expect(blocked_path.blocked, "Occupied voxel on direct path must block acoustic trace");
    test.expect(blocked_path.first_blocking_voxel != nullptr,
                "Blocked acoustic trace must identify its first voxel");
    test.expect_near(blocked_path.transmission_gain, 0.0F, 1.0e-6F,
                     "Blocked direct acoustic path must have zero transmission gain");
    test.expect_throws<std::invalid_argument>(
        [&] { static_cast<void>(acoustic_raytracer.trace_direct_path(source, source)); },
        "Coincident acoustic source and receiver must be rejected");

    voxel4d::SphericalHarmonicsL1 harmonics;
    test.expect_near(harmonics.evaluate(glm::vec3(1.0F, 0.0F, 0.0F)).x, 0.0F, 1.0e-6F,
                     "Empty spherical-harmonic field must evaluate to zero");
    harmonics.accumulate(glm::vec3(1.0F, 0.0F, 0.0F), glm::vec3(2.0F, 1.0F, 0.5F), 1.0F);
    const glm::vec3 forward_radiance = harmonics.evaluate_clamped(glm::vec3(1.0F, 0.0F, 0.0F));
    const glm::vec3 backward_radiance = harmonics.evaluate_clamped(glm::vec3(-1.0F, 0.0F, 0.0F));
    test.expect(forward_radiance.x > backward_radiance.x,
                "L1 field must preserve directional contrast in the sampled direction");
    test.expect(forward_radiance.y > 0.0F && forward_radiance.z > 0.0F,
                "L1 field must preserve all positive radiance channels");
    test.expect_throws<std::invalid_argument>(
        [&] { harmonics.accumulate(glm::vec3(0.0F), glm::vec3(1.0F), 1.0F); },
        "Spherical harmonics must reject a zero direction");
    test.expect_throws<std::invalid_argument>(
        [&] { harmonics.accumulate(glm::vec3(1.0F), glm::vec3(1.0F), -1.0F); },
        "Spherical harmonics must reject a negative solid angle");
    harmonics.clear();
    test.expect_near(harmonics.evaluate(glm::vec3(1.0F, 0.0F, 0.0F)).x, 0.0F, 1.0e-6F,
                     "Cleared spherical-harmonic field must evaluate to zero");

    return test.failures() == 0 ? 0 : 1;
}
