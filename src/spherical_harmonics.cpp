#include "spherical_harmonics.h"

#include <cmath>
#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <stdexcept>

namespace {

constexpr float kMinimumDirectionLength = 1.0e-6F;
constexpr float kY00 = 0.2820947918F;
constexpr float kY1 = 0.4886025119F;

bool is_finite(const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

}  // namespace

namespace voxel4d {

std::array<float, 4> SphericalHarmonicsL1::basis(const glm::vec3& direction) {
    const float direction_length = glm::length(direction);
    if (direction_length < kMinimumDirectionLength || !is_finite(direction)) {
        throw std::invalid_argument("spherical-harmonic direction must be finite and non-zero");
    }

    const glm::vec3 unit_direction = direction / direction_length;
    return {{kY00, kY1 * unit_direction.x, kY1 * unit_direction.y, kY1 * unit_direction.z}};
}

void SphericalHarmonicsL1::accumulate(const glm::vec3& direction, const glm::vec3& radiance_linear,
                                      const float solid_angle_steradians) {
    if (!is_finite(radiance_linear) || !std::isfinite(solid_angle_steradians) ||
        solid_angle_steradians < 0.0F) {
        throw std::invalid_argument(
            "radiance and solid angle must be finite with non-negative solid angle");
    }

    const std::array<float, 4> values = basis(direction);
    for (std::size_t index = 0; index < coefficients_.size(); ++index) {
        coefficients_[index] += radiance_linear * values[index] * solid_angle_steradians;
    }
}

glm::vec3 SphericalHarmonicsL1::evaluate(const glm::vec3& direction) const {
    const std::array<float, 4> values = basis(direction);
    glm::vec3 radiance(0.0F);
    for (std::size_t index = 0; index < coefficients_.size(); ++index) {
        radiance += coefficients_[index] * values[index];
    }
    return radiance;
}

glm::vec3 SphericalHarmonicsL1::evaluate_clamped(const glm::vec3& direction) const {
    return glm::max(evaluate(direction), glm::vec3(0.0F));
}

void SphericalHarmonicsL1::clear() {
    coefficients_.fill(glm::vec3(0.0F));
}

const std::array<glm::vec3, 4>& SphericalHarmonicsL1::coefficients() const {
    return coefficients_;
}

}  // namespace voxel4d
