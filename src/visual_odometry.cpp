#include "visual_odometry.h"

#include <array>
#include <cmath>
#include <stdexcept>

namespace {

constexpr float kMinimumCenteredVariance = 1.0e-10F;
constexpr int kPowerIterationCount = 64;

bool is_finite(const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

using Vector4 = std::array<float, 4>;
using Matrix4 = std::array<std::array<float, 4>, 4>;

Vector4 multiply(const Matrix4& matrix, const Vector4& vector) {
    Vector4 result{};
    for (std::size_t row = 0; row < result.size(); ++row) {
        for (std::size_t column = 0; column < vector.size(); ++column) {
            result[row] += matrix[row][column] * vector[column];
        }
    }
    return result;
}

float squared_norm(const Vector4& vector) {
    float result = 0.0F;
    for (const float value : vector) {
        result += value * value;
    }
    return result;
}

Vector4 normalize(const Vector4& vector) {
    const float norm = std::sqrt(squared_norm(vector));
    if (norm <= 0.0F) {
        return Vector4{};
    }

    Vector4 result{};
    for (std::size_t index = 0; index < vector.size(); ++index) {
        result[index] = vector[index] / norm;
    }
    return result;
}

}  // namespace

namespace voxel4d {

VisualOdometryResult VisualOdometryEstimator::estimate_rigid_motion(
    const std::vector<PointCorrespondence3D>& correspondences) const {
    VisualOdometryResult result{};
    result.correspondence_count = correspondences.size();
    if (correspondences.size() < 3U) {
        return result;
    }

    glm::vec3 previous_centroid(0.0F);
    glm::vec3 current_centroid(0.0F);
    for (const PointCorrespondence3D& correspondence : correspondences) {
        if (!is_finite(correspondence.previous_point_meters) ||
            !is_finite(correspondence.current_point_meters)) {
            throw std::invalid_argument("point correspondences must be finite");
        }
        previous_centroid += correspondence.previous_point_meters;
        current_centroid += correspondence.current_point_meters;
    }

    const float inverse_count = 1.0F / static_cast<float>(correspondences.size());
    previous_centroid *= inverse_count;
    current_centroid *= inverse_count;

    float previous_variance = 0.0F;
    float current_variance = 0.0F;
    float sxx = 0.0F;
    float sxy = 0.0F;
    float sxz = 0.0F;
    float syx = 0.0F;
    float syy = 0.0F;
    float syz = 0.0F;
    float szx = 0.0F;
    float szy = 0.0F;
    float szz = 0.0F;

    for (const PointCorrespondence3D& correspondence : correspondences) {
        const glm::vec3 previous = correspondence.previous_point_meters - previous_centroid;
        const glm::vec3 current = correspondence.current_point_meters - current_centroid;
        previous_variance += glm::dot(previous, previous);
        current_variance += glm::dot(current, current);
        sxx += previous.x * current.x;
        sxy += previous.x * current.y;
        sxz += previous.x * current.z;
        syx += previous.y * current.x;
        syy += previous.y * current.y;
        syz += previous.y * current.z;
        szx += previous.z * current.x;
        szy += previous.z * current.y;
        szz += previous.z * current.z;
    }

    if (previous_variance < kMinimumCenteredVariance ||
        current_variance < kMinimumCenteredVariance) {
        return result;
    }

    const Matrix4 quaternion_matrix{{
        {{sxx + syy + szz, syz - szy, szx - sxz, sxy - syx}},
        {{syz - szy, sxx - syy - szz, sxy + syx, szx + sxz}},
        {{szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy}},
        {{sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz}},
    }};

    Vector4 eigenvector{{1.0F, 0.0F, 0.0F, 0.0F}};
    for (int iteration = 0; iteration < kPowerIterationCount; ++iteration) {
        eigenvector = normalize(multiply(quaternion_matrix, eigenvector));
        if (squared_norm(eigenvector) == 0.0F) {
            return result;
        }
    }

    SensorPose current_from_previous{};
    current_from_previous.world_from_sensor =
        glm::quat(eigenvector[0], eigenvector[1], eigenvector[2], eigenvector[3]);
    current_from_previous.translation_meters =
        current_centroid - current_from_previous.world_from_sensor * previous_centroid;
    current_from_previous = current_from_previous.normalized();

    float squared_error = 0.0F;
    for (const PointCorrespondence3D& correspondence : correspondences) {
        const glm::vec3 residual =
            current_from_previous.transform_point_to_world(correspondence.previous_point_meters) -
            correspondence.current_point_meters;
        squared_error += glm::dot(residual, residual);
    }

    result.success = true;
    result.current_from_previous = current_from_previous;
    result.root_mean_square_error_meters =
        std::sqrt(squared_error / static_cast<float>(correspondences.size()));
    return result;
}

}  // namespace voxel4d
