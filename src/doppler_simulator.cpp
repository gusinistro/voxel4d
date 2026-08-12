#include "doppler_simulator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr float kMinimumDistanceMeters = 1.0e-4F;
constexpr float kMaximumBeta = 0.999999F;
constexpr float kPi = 3.14159265358979323846F;
}  // namespace

float DopplerSimulator::calculate_radial_velocity(const glm::vec3& moving_position,
                                                  const glm::vec3& moving_velocity,
                                                  const glm::vec3& reference_position) {
    const glm::vec3 direction = reference_position - moving_position;
    if (glm::length(direction) < kMinimumDistanceMeters) {
        return 0.0F;
    }
    return glm::dot(moving_velocity, glm::normalize(direction));
}

DopplerResult DopplerSimulator::calculate_doppler_sound(const glm::vec3& source_position,
                                                        const glm::vec3& source_velocity,
                                                        const glm::vec3& observer_position,
                                                        const glm::vec3& observer_velocity,
                                                        float source_frequency_hz) const {
    if (source_frequency_hz <= 0.0F) {
        throw std::invalid_argument("source_frequency_hz must be greater than zero");
    }

    // Positive velocities are measured from source to observer.
    const float source_radial_velocity =
        calculate_radial_velocity(source_position, source_velocity, observer_position);
    const float observer_radial_velocity =
        -calculate_radial_velocity(observer_position, observer_velocity, source_position);

    const float denominator = voxel4d::kSpeedOfSoundMetersPerSecond - source_radial_velocity;
    if (std::abs(denominator) < kMinimumDistanceMeters) {
        throw std::domain_error("source radial velocity is too close to the speed of sound");
    }

    DopplerResult result{};
    result.frequency_ratio =
        (voxel4d::kSpeedOfSoundMetersPerSecond - observer_radial_velocity) / denominator;
    result.frequency_shift_hz = source_frequency_hz * (result.frequency_ratio - 1.0F);
    return result;
}

DopplerResult DopplerSimulator::calculate_doppler_light(const glm::vec3& source_position,
                                                        const glm::vec3& source_velocity,
                                                        const glm::vec3& observer_position,
                                                        float source_wavelength_meters) const {
    if (source_wavelength_meters <= 0.0F) {
        throw std::invalid_argument("source_wavelength_meters must be greater than zero");
    }

    const float radial_velocity =
        calculate_radial_velocity(source_position, source_velocity, observer_position);
    const float beta = std::clamp(radial_velocity / voxel4d::kSpeedOfLightMetersPerSecond,
                                  -kMaximumBeta, kMaximumBeta);

    DopplerResult result{};
    // Longitudinal special-relativistic Doppler ratio: f_observed / f_emitted.
    result.frequency_ratio = std::sqrt((1.0F - beta) / (1.0F + beta));
    const float observed_wavelength = source_wavelength_meters / result.frequency_ratio;
    result.wavelength_shift_meters = observed_wavelength - source_wavelength_meters;
    return result;
}

DopplerResult DopplerSimulator::calculate_voxel_doppler(const std::shared_ptr<OctreeNode>& voxel,
                                                        const glm::vec3& observer_position,
                                                        float base_frequency_hz) const {
    if (!voxel) {
        return {};
    }

    return calculate_doppler_sound(voxel->center, voxel->attribute.velocity, observer_position,
                                   glm::vec3(0.0F), base_frequency_hz);
}

void DopplerSimulator::sample_sound_doppler_field(const std::shared_ptr<SparseVoxelOctree>& octree,
                                                  const glm::vec3& sound_source,
                                                  const glm::vec3& sound_velocity,
                                                  float base_frequency_hz,
                                                  std::vector<DopplerResult>& results) const {
    results.clear();
    if (!octree || !octree->get_root() || base_frequency_hz <= 0.0F) {
        return;
    }

    const auto root = octree->get_root();
    const float sample_radius = std::min(10.0F, root->size * 0.25F);
    constexpr int kSampleCount = 8;
    results.reserve(kSampleCount);

    for (int index = 0; index < kSampleCount; ++index) {
        const float angle =
            (2.0F * kPi * static_cast<float>(index)) / static_cast<float>(kSampleCount);
        const glm::vec3 observer_position =
            sound_source +
            glm::vec3(sample_radius * std::cos(angle), sample_radius * std::sin(angle), 0.0F);

        results.push_back(calculate_doppler_sound(sound_source, sound_velocity, observer_position,
                                                  glm::vec3(0.0F), base_frequency_hz));
    }
}
