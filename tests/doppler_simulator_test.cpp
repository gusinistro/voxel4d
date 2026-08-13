#include "doppler_simulator.h"

#include <glm/glm.hpp>
#include <memory>
#include <stdexcept>
#include <vector>

#include "octree.h"
#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;
    DopplerSimulator simulator;

    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(simulator.calculate_doppler_sound(
                glm::vec3(0.0F), glm::vec3(0.0F), glm::vec3(1.0F), glm::vec3(0.0F), 0.0F));
        },
        "Acoustic Doppler must reject a non-positive source frequency");
    test.expect_throws<std::invalid_argument>(
        [&] {
            static_cast<void>(simulator.calculate_doppler_light(glm::vec3(0.0F), glm::vec3(0.0F),
                                                                glm::vec3(1.0F), 0.0F));
        },
        "Optical Doppler must reject a non-positive source wavelength");

    const DopplerResult stationary_sound = simulator.calculate_doppler_sound(
        glm::vec3(0.0F), glm::vec3(0.0F), glm::vec3(10.0F, 0.0F, 0.0F), glm::vec3(0.0F), 440.0F);
    test.expect_near(stationary_sound.frequency_ratio, 1.0F, 1.0e-6F,
                     "Stationary acoustic source and observer must have unity ratio");
    test.expect_near(stationary_sound.frequency_shift_hz, 0.0F, 1.0e-6F,
                     "Stationary acoustic source and observer must have zero shift");

    const DopplerResult approaching_sound =
        simulator.calculate_doppler_sound(glm::vec3(0.0F), glm::vec3(10.0F, 0.0F, 0.0F),
                                          glm::vec3(10.0F, 0.0F, 0.0F), glm::vec3(0.0F), 440.0F);
    const float expected_approaching_ratio =
        voxel4d::kSpeedOfSoundMetersPerSecond / (voxel4d::kSpeedOfSoundMetersPerSecond - 10.0F);
    test.expect_near(approaching_sound.frequency_ratio, expected_approaching_ratio, 1.0e-5F,
                     "Approaching acoustic source must use the classical Doppler ratio");
    test.expect(approaching_sound.frequency_shift_hz > 0.0F,
                "Approaching acoustic source must produce a positive frequency shift");

    const DopplerResult stationary_light = simulator.calculate_doppler_light(
        glm::vec3(0.0F), glm::vec3(0.0F), glm::vec3(1.0F, 0.0F, 0.0F), 550.0e-9F);
    test.expect_near(stationary_light.frequency_ratio, 1.0F, 1.0e-6F,
                     "Stationary optical source must have unity ratio");
    test.expect_near(stationary_light.wavelength_shift_meters, 0.0F, 1.0e-10F,
                     "Stationary optical source must have zero wavelength shift");

    const DopplerResult receding_light = simulator.calculate_doppler_light(
        glm::vec3(0.0F), glm::vec3(1000.0F, 0.0F, 0.0F), glm::vec3(10.0F, 0.0F, 0.0F), 550.0e-9F);
    test.expect(receding_light.frequency_ratio < 1.0F,
                "Receding optical source must produce a redshifted frequency ratio");
    test.expect(receding_light.wavelength_shift_meters > 0.0F,
                "Receding optical source must increase observed wavelength");

    test.expect_near(
        simulator.calculate_voxel_doppler(nullptr, glm::vec3(0.0F), 440.0F).frequency_ratio, 1.0F,
        1.0e-6F, "Null voxel Doppler request must return the neutral result");

    const auto octree = std::make_shared<SparseVoxelOctree>(glm::vec3(0.0F), 20.0F, 1);
    std::vector<DopplerResult> field{stationary_sound};
    simulator.sample_sound_doppler_field(octree, glm::vec3(0.0F), glm::vec3(5.0F, 0.0F, 0.0F),
                                         440.0F, field);
    test.expect(field.size() == 8U, "Doppler field sampling must emit eight receiver samples");
    test.expect(field.front().frequency_ratio > 1.0F,
                "First receiver on the source velocity axis must observe an approaching shift");

    simulator.sample_sound_doppler_field(nullptr, glm::vec3(0.0F), glm::vec3(0.0F), 440.0F, field);
    test.expect(field.empty(), "Invalid field inputs must clear and leave no samples");

    return test.failures() == 0 ? 0 : 1;
}
