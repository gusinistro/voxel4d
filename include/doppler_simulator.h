#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

#include "octree.h"

namespace voxel4d {
constexpr float kSpeedOfLightMetersPerSecond = 299792458.0F;
constexpr float kSpeedOfSoundMetersPerSecond = 343.0F;
}  // namespace voxel4d

/** @brief Doppler output for either acoustic or optical sampling. */
struct DopplerResult {
    float frequency_shift_hz{0.0F};
    float frequency_ratio{1.0F};
    float wavelength_shift_meters{0.0F};
};

/**
 * @brief Classical acoustic and special-relativistic optical Doppler helpers.
 *
 * Coordinate positions are expressed in meters and velocities in metres per
 * second. The acoustic calculation assumes a stationary medium.
 */
class DopplerSimulator {
   public:
    [[nodiscard]] DopplerResult calculate_doppler_sound(const glm::vec3& source_position,
                                                        const glm::vec3& source_velocity,
                                                        const glm::vec3& observer_position,
                                                        const glm::vec3& observer_velocity,
                                                        float source_frequency_hz) const;

    [[nodiscard]] DopplerResult calculate_doppler_light(const glm::vec3& source_position,
                                                        const glm::vec3& source_velocity,
                                                        const glm::vec3& observer_position,
                                                        float source_wavelength_meters) const;

    [[nodiscard]] DopplerResult calculate_voxel_doppler(const std::shared_ptr<OctreeNode>& voxel,
                                                        const glm::vec3& observer_position,
                                                        float base_frequency_hz) const;

    /**
     * @brief Samples the acoustic Doppler field around a source.
     *
     * This PoC method samples listener positions; it does not model occlusion,
     * reflections, diffraction, or a full acoustic transport solution.
     */
    void sample_sound_doppler_field(const std::shared_ptr<SparseVoxelOctree>& octree,
                                    const glm::vec3& sound_source, const glm::vec3& sound_velocity,
                                    float base_frequency_hz,
                                    std::vector<DopplerResult>& results) const;

   private:
    [[nodiscard]] static float calculate_radial_velocity(const glm::vec3& moving_position,
                                                         const glm::vec3& moving_velocity,
                                                         const glm::vec3& reference_position);
};
