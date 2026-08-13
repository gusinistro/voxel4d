#pragma once

#include <array>
#include <glm/glm.hpp>

namespace voxel4d {

/**
 * @brief Real spherical-harmonic coefficients through first order (four basis functions).
 *
 * Coefficients describe a low-frequency directional radiance field. The caller
 * supplies each sample's solid angle in steradians; this compact baseline does
 * not implement higher-order bands, environment-map loading, or GPU shading.
 */
class SphericalHarmonicsL1 {
   public:
    /** @brief Adds a radiance sample along a non-zero direction. */
    void accumulate(const glm::vec3& direction, const glm::vec3& radiance_linear,
                    float solid_angle_steradians);

    /** @return Unclamped radiance reconstructed from the L1 field. */
    [[nodiscard]] glm::vec3 evaluate(const glm::vec3& direction) const;

    /** @return Non-negative radiance reconstructed from the L1 field. */
    [[nodiscard]] glm::vec3 evaluate_clamped(const glm::vec3& direction) const;

    void clear();

    /** @return Coefficients ordered as Y00, Y1x, Y1y, Y1z. */
    [[nodiscard]] const std::array<glm::vec3, 4>& coefficients() const;

   private:
    [[nodiscard]] static std::array<float, 4> basis(const glm::vec3& direction);

    std::array<glm::vec3, 4> coefficients_{};
};

}  // namespace voxel4d
