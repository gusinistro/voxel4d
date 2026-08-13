#include "multi_sensor_fuser.h"

#include <algorithm>
#include <stdexcept>

namespace {

float blend(const float previous_value, const float incoming_value, const float previous_weight,
            const float incoming_weight) {
    const float total_weight = previous_weight + incoming_weight;
    if (total_weight <= 0.0F) {
        return incoming_value;
    }
    return (previous_value * previous_weight + incoming_value * incoming_weight) / total_weight;
}

glm::vec3 blend(const glm::vec3& previous_value, const glm::vec3& incoming_value,
                const float previous_weight, const float incoming_weight) {
    const float total_weight = previous_weight + incoming_weight;
    if (total_weight <= 0.0F) {
        return incoming_value;
    }
    return (previous_value * previous_weight + incoming_value * incoming_weight) / total_weight;
}

}  // namespace

namespace voxel4d {

MultiSensorFuser::MultiSensorFuser(std::shared_ptr<SparseVoxelOctree> octree)
    : octree_(std::move(octree)) {
    if (!octree_) {
        throw std::invalid_argument("octree must not be null");
    }
}

std::uint32_t MultiSensorFuser::modality_mask(const SensorModality modality) {
    return 1U << static_cast<unsigned int>(modality);
}

std::size_t MultiSensorFuser::fuse(const SensorObservation& observation) const {
    if (!observation.is_valid()) {
        throw std::invalid_argument("sensor observation does not satisfy its contract");
    }
    if (observation.modality == SensorModality::kImu) {
        return 0U;
    }

    std::size_t fused_sample_count = 0U;
    for (const SpatialSample& sample : observation.spatial_samples) {
        const glm::vec3 world_position =
            observation.world_from_sensor.transform_point_to_world(sample.position_sensor_meters);
        if (!octree_->contains(world_position)) {
            continue;
        }

        const auto existing_node = octree_->search(world_position);
        VoxelAttribute attribute{};
        if (existing_node && existing_node->attribute.density > 0.0F) {
            attribute = existing_node->attribute;
        }

        const float incoming_weight = sample.confidence * observation.observation_confidence;
        const float previous_weight = attribute.confidence;
        const bool has_previous_modality =
            (attribute.source_modality_mask & modality_mask(observation.modality)) != 0U;
        attribute.density = std::max(attribute.density, incoming_weight);
        attribute.confidence =
            std::clamp(previous_weight + incoming_weight * (1.0F - previous_weight), 0.0F, 1.0F);
        attribute.source_modality_mask |= modality_mask(observation.modality);
        attribute.last_observed_timestamp_nanoseconds = std::max(
            attribute.last_observed_timestamp_nanoseconds, observation.timestamp_nanoseconds);
        if (sample.semantic_label != 0) {
            attribute.semantic_label = sample.semantic_label;
        }

        switch (observation.modality) {
            case SensorModality::kRgbd:
                attribute.color =
                    blend(attribute.color, sample.color_linear, previous_weight, incoming_weight);
                attribute.intensity =
                    blend(attribute.intensity, sample.intensity, previous_weight, incoming_weight);
                attribute.temperature = blend(attribute.temperature, sample.temperature_kelvin,
                                              previous_weight, incoming_weight);
                break;
            case SensorModality::kLidar:
                attribute.intensity =
                    blend(attribute.intensity, sample.intensity, previous_weight, incoming_weight);
                break;
            case SensorModality::kRadar: {
                attribute.intensity =
                    blend(attribute.intensity, sample.intensity, previous_weight, incoming_weight);
                const glm::vec3 incoming_velocity =
                    observation.world_from_sensor.transform_vector_to_world(
                        sample.velocity_sensor_meters_per_second);
                attribute.velocity = has_previous_modality
                                         ? blend(attribute.velocity, incoming_velocity,
                                                 previous_weight, incoming_weight)
                                         : incoming_velocity;
                break;
            }
            case SensorModality::kThermal:
                attribute.temperature = blend(attribute.temperature, sample.temperature_kelvin,
                                              previous_weight, incoming_weight);
                break;
            case SensorModality::kImu:
                break;
        }

        if (!octree_->insert(world_position, attribute)) {
            continue;
        }
        ++fused_sample_count;
    }
    return fused_sample_count;
}

}  // namespace voxel4d
