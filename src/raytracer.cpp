#include "raytracer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
constexpr float kRayEpsilon = 1.0e-5F;

float axis_entry_time(float origin, float direction, float min_bound, float max_bound,
                      float& axis_exit) {
    if (std::abs(direction) < kRayEpsilon) {
        if (origin < min_bound || origin > max_bound) {
            axis_exit = -std::numeric_limits<float>::infinity();
            return std::numeric_limits<float>::infinity();
        }
        axis_exit = std::numeric_limits<float>::infinity();
        return -std::numeric_limits<float>::infinity();
    }

    const float first = (min_bound - origin) / direction;
    const float second = (max_bound - origin) / direction;
    axis_exit = std::max(first, second);
    return std::min(first, second);
}

int initial_cell_index(float value, float min_bound, float cell_size, int resolution) {
    const int index = static_cast<int>(std::floor((value - min_bound) / cell_size));
    return std::clamp(index, 0, resolution - 1);
}
}  // namespace

VoxelRaytracer::VoxelRaytracer(std::shared_ptr<SparseVoxelOctree> octree)
    : octree_(std::move(octree)) {
    if (!octree_) {
        throw std::invalid_argument("octree must not be null");
    }
}

bool VoxelRaytracer::ray_box_intersect(const Ray& ray, const glm::vec3& box_min,
                                       const glm::vec3& box_max, float& t_near,
                                       float& t_far) const {
    float x_far = 0.0F;
    float y_far = 0.0F;
    float z_far = 0.0F;
    const float x_near =
        axis_entry_time(ray.origin.x, ray.direction.x, box_min.x, box_max.x, x_far);
    const float y_near =
        axis_entry_time(ray.origin.y, ray.direction.y, box_min.y, box_max.y, y_far);
    const float z_near =
        axis_entry_time(ray.origin.z, ray.direction.z, box_min.z, box_max.z, z_far);

    t_near = std::max({x_near, y_near, z_near});
    t_far = std::min({x_far, y_far, z_far});
    return t_near <= t_far && t_far >= 0.0F;
}

std::vector<std::shared_ptr<OctreeNode>> VoxelRaytracer::dda_traverse(const Ray& ray,
                                                                      float max_distance) const {
    std::vector<std::shared_ptr<OctreeNode>> visited;
    if (max_distance <= 0.0F ||
        glm::dot(ray.direction, ray.direction) < kRayEpsilon * kRayEpsilon) {
        return visited;
    }

    const auto root = octree_->get_root();
    if (!root) {
        return visited;
    }

    float t_near = 0.0F;
    float t_far = 0.0F;
    const glm::vec3 root_min = root->center - glm::vec3(root->size * 0.5F);
    const glm::vec3 root_max = root->center + glm::vec3(root->size * 0.5F);
    if (!ray_box_intersect(ray, root_min, root_max, t_near, t_far)) {
        return visited;
    }

    const glm::vec3 direction = glm::normalize(ray.direction);
    const int resolution = 1 << octree_->get_max_depth();
    const float cell_size = root->size / static_cast<float>(resolution);
    const float start_t = std::max(0.0F, t_near);
    const float end_t = std::min(t_far, max_distance);
    if (start_t > end_t) {
        return visited;
    }

    glm::vec3 position = ray.origin + direction * (start_t + kRayEpsilon);
    glm::ivec3 cell(initial_cell_index(position.x, root_min.x, cell_size, resolution),
                    initial_cell_index(position.y, root_min.y, cell_size, resolution),
                    initial_cell_index(position.z, root_min.z, cell_size, resolution));

    const glm::ivec3 step(direction.x >= 0.0F ? 1 : -1, direction.y >= 0.0F ? 1 : -1,
                          direction.z >= 0.0F ? 1 : -1);

    const auto next_crossing = [&](float origin, float coordinate, float dir, int cell_index,
                                   int axis_step) {
        if (std::abs(dir) < kRayEpsilon) {
            return std::numeric_limits<float>::infinity();
        }
        const float next_boundary =
            coordinate +
            static_cast<float>(axis_step > 0 ? cell_index + 1 : cell_index) * cell_size;
        return start_t + (next_boundary - origin) / dir;
    };

    glm::vec3 t_max(next_crossing(position.x, root_min.x, direction.x, cell.x, step.x),
                    next_crossing(position.y, root_min.y, direction.y, cell.y, step.y),
                    next_crossing(position.z, root_min.z, direction.z, cell.z, step.z));
    const glm::vec3 t_delta(
        std::abs(direction.x) < kRayEpsilon ? std::numeric_limits<float>::infinity()
                                            : cell_size / std::abs(direction.x),
        std::abs(direction.y) < kRayEpsilon ? std::numeric_limits<float>::infinity()
                                            : cell_size / std::abs(direction.y),
        std::abs(direction.z) < kRayEpsilon ? std::numeric_limits<float>::infinity()
                                            : cell_size / std::abs(direction.z));

    float current_t = start_t;
    std::shared_ptr<OctreeNode> last_voxel;
    while (current_t <= end_t && cell.x >= 0 && cell.x < resolution && cell.y >= 0 &&
           cell.y < resolution && cell.z >= 0 && cell.z < resolution) {
        const glm::vec3 sample_position = ray.origin + direction * (current_t + kRayEpsilon);
        const auto voxel = octree_->search(sample_position);
        if (voxel && voxel->attribute.density > 0.0F && voxel != last_voxel) {
            visited.push_back(voxel);
            last_voxel = voxel;
        }

        if (t_max.x <= t_max.y && t_max.x <= t_max.z) {
            current_t = t_max.x;
            t_max.x += t_delta.x;
            cell.x += step.x;
        } else if (t_max.y <= t_max.z) {
            current_t = t_max.y;
            t_max.y += t_delta.y;
            cell.y += step.y;
        } else {
            current_t = t_max.z;
            t_max.z += t_delta.z;
            cell.z += step.z;
        }
    }

    return visited;
}

RayHitResult VoxelRaytracer::trace_ray(const Ray& ray, float max_distance) const {
    RayHitResult result{};
    if (max_distance <= 0.0F ||
        glm::dot(ray.direction, ray.direction) < kRayEpsilon * kRayEpsilon) {
        return result;
    }

    const auto root = octree_->get_root();
    if (!root) {
        return result;
    }

    const glm::vec3 root_min = root->center - glm::vec3(root->size * 0.5F);
    const glm::vec3 root_max = root->center + glm::vec3(root->size * 0.5F);
    float t_near = 0.0F;
    float t_far = 0.0F;
    if (!ray_box_intersect(ray, root_min, root_max, t_near, t_far)) {
        return result;
    }

    const glm::vec3 direction = glm::normalize(ray.direction);
    const int resolution = 1 << octree_->get_max_depth();
    const float cell_size = root->size / static_cast<float>(resolution);
    const float start_t = std::max(0.0F, t_near);
    const float end_t = std::min(t_far, max_distance);
    if (start_t > end_t) {
        return result;
    }

    glm::vec3 position = ray.origin + direction * (start_t + kRayEpsilon);
    glm::ivec3 cell(initial_cell_index(position.x, root_min.x, cell_size, resolution),
                    initial_cell_index(position.y, root_min.y, cell_size, resolution),
                    initial_cell_index(position.z, root_min.z, cell_size, resolution));
    const glm::ivec3 step(direction.x >= 0.0F ? 1 : -1, direction.y >= 0.0F ? 1 : -1,
                          direction.z >= 0.0F ? 1 : -1);

    const auto next_crossing = [&](float origin, float coordinate, float dir, int cell_index,
                                   int axis_step) {
        if (std::abs(dir) < kRayEpsilon) {
            return std::numeric_limits<float>::infinity();
        }
        const float next_boundary =
            coordinate +
            static_cast<float>(axis_step > 0 ? cell_index + 1 : cell_index) * cell_size;
        return start_t + (next_boundary - origin) / dir;
    };

    glm::vec3 t_max(next_crossing(position.x, root_min.x, direction.x, cell.x, step.x),
                    next_crossing(position.y, root_min.y, direction.y, cell.y, step.y),
                    next_crossing(position.z, root_min.z, direction.z, cell.z, step.z));
    const glm::vec3 t_delta(
        std::abs(direction.x) < kRayEpsilon ? std::numeric_limits<float>::infinity()
                                            : cell_size / std::abs(direction.x),
        std::abs(direction.y) < kRayEpsilon ? std::numeric_limits<float>::infinity()
                                            : cell_size / std::abs(direction.y),
        std::abs(direction.z) < kRayEpsilon ? std::numeric_limits<float>::infinity()
                                            : cell_size / std::abs(direction.z));

    float current_t = start_t;
    while (current_t <= end_t && cell.x >= 0 && cell.x < resolution && cell.y >= 0 &&
           cell.y < resolution && cell.z >= 0 && cell.z < resolution) {
        const glm::vec3 sample_position = ray.origin + direction * (current_t + kRayEpsilon);
        const auto voxel = octree_->search(sample_position);
        if (voxel && voxel->attribute.density > 0.0F) {
            result.hit = true;
            result.distance = current_t;
            result.hit_point = sample_position;
            result.voxel = voxel;
            return result;
        }

        if (t_max.x <= t_max.y && t_max.x <= t_max.z) {
            current_t = t_max.x;
            t_max.x += t_delta.x;
            cell.x += step.x;
        } else if (t_max.y <= t_max.z) {
            current_t = t_max.y;
            t_max.y += t_delta.y;
            cell.y += step.y;
        } else {
            current_t = t_max.z;
            t_max.z += t_delta.z;
            cell.z += step.z;
        }
    }

    return result;
}

std::vector<RayHitResult> VoxelRaytracer::trace_rays(const std::vector<Ray>& rays,
                                                     float max_distance) const {
    std::vector<RayHitResult> results;
    results.reserve(rays.size());
    for (const Ray& ray : rays) {
        results.push_back(trace_ray(ray, max_distance));
    }
    return results;
}

glm::vec3 VoxelRaytracer::calculate_lighting(const glm::vec3& position,
                                             const glm::vec3& normal) const {
    (void)position;
    const glm::vec3 light_direction = glm::normalize(glm::vec3(1.0F, 1.0F, 1.0F));
    const float diffuse = std::max(0.0F, glm::dot(glm::normalize(normal), light_direction));
    return glm::vec3(0.2F) + glm::vec3(0.8F * diffuse);
}
