#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "sensor_observation.h"

namespace voxel4d {

enum class InferenceStatus {
    kSuccess,
    kUnavailable,
};

/** @brief A semantic prediction aligned to an input spatial sample index. */
struct SemanticPrediction {
    std::size_t sample_index{0U};
    int semantic_label{0};
    float confidence{0.0F};

    [[nodiscard]] bool is_valid(std::size_t input_sample_count) const;
};

/** @brief Result of an optional inference call without hidden fallback behavior. */
struct SemanticInferenceResult {
    InferenceStatus status{InferenceStatus::kUnavailable};
    std::string provider_name{};
    std::vector<SemanticPrediction> predictions{};
};

/**
 * @brief Provider contract for optional semantic inference or post-processing.
 *
 * Implementations may use CPU, GPU, NPU, remote services, or learned models,
 * but the geometry pipeline must remain valid when no provider is available.
 */
class SemanticInferenceProvider {
   public:
    virtual ~SemanticInferenceProvider() = default;

    [[nodiscard]] virtual std::string name() const = 0;
    [[nodiscard]] virtual bool is_available() const = 0;
    [[nodiscard]] virtual SemanticInferenceResult infer(
        const std::vector<SpatialSample>& samples) const = 0;
};

/** @brief Explicit no-model provider that reports unavailable inference. */
class UnavailableSemanticInferenceProvider final : public SemanticInferenceProvider {
   public:
    [[nodiscard]] std::string name() const override;
    [[nodiscard]] bool is_available() const override;
    [[nodiscard]] SemanticInferenceResult infer(
        const std::vector<SpatialSample>& samples) const override;
};

/**
 * @brief Deterministic provider that exposes labels already supplied by an input stream.
 *
 * This is not a learned model. It validates downstream integration paths while
 * preserving externally annotated labels for replay and evaluation.
 */
class RecordedLabelInferenceProvider final : public SemanticInferenceProvider {
   public:
    [[nodiscard]] std::string name() const override;
    [[nodiscard]] bool is_available() const override;
    [[nodiscard]] SemanticInferenceResult infer(
        const std::vector<SpatialSample>& samples) const override;
};

}  // namespace voxel4d
