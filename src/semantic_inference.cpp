#include "semantic_inference.h"

#include <cmath>

namespace voxel4d {

bool SemanticPrediction::is_valid(const std::size_t input_sample_count) const {
    return sample_index < input_sample_count && semantic_label > 0 && std::isfinite(confidence) &&
           confidence >= 0.0F && confidence <= 1.0F;
}

std::string UnavailableSemanticInferenceProvider::name() const {
    return "unavailable";
}

bool UnavailableSemanticInferenceProvider::is_available() const {
    return false;
}

SemanticInferenceResult UnavailableSemanticInferenceProvider::infer(
    const std::vector<SpatialSample>& /*samples*/) const {
    return SemanticInferenceResult{InferenceStatus::kUnavailable, name(), {}};
}

std::string RecordedLabelInferenceProvider::name() const {
    return "recorded-labels";
}

bool RecordedLabelInferenceProvider::is_available() const {
    return true;
}

SemanticInferenceResult RecordedLabelInferenceProvider::infer(
    const std::vector<SpatialSample>& samples) const {
    SemanticInferenceResult result{};
    result.status = InferenceStatus::kSuccess;
    result.provider_name = name();
    result.predictions.reserve(samples.size());
    for (std::size_t index = 0U; index < samples.size(); ++index) {
        const SpatialSample& sample = samples.at(index);
        if (sample.semantic_label > 0 && std::isfinite(sample.confidence) &&
            sample.confidence >= 0.0F && sample.confidence <= 1.0F) {
            result.predictions.push_back(
                SemanticPrediction{index, sample.semantic_label, sample.confidence});
        }
    }
    return result;
}

}  // namespace voxel4d
