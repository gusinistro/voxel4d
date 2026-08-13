#include "semantic_inference.h"

#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;

    voxel4d::SpatialSample annotated{};
    annotated.position_sensor_meters = glm::vec3(1.0F, 0.0F, 0.0F);
    annotated.color_linear = glm::vec3(0.5F);
    annotated.confidence = 0.8F;
    annotated.semantic_label = 4;
    voxel4d::SpatialSample unannotated = annotated;
    unannotated.semantic_label = 0;

    const voxel4d::UnavailableSemanticInferenceProvider unavailable;
    test.expect(!unavailable.is_available(),
                "Unavailable provider must explicitly report lack of inference capability");
    const voxel4d::SemanticInferenceResult unavailable_result =
        unavailable.infer({annotated, unannotated});
    test.expect(unavailable_result.status == voxel4d::InferenceStatus::kUnavailable &&
                    unavailable_result.predictions.empty(),
                "Unavailable provider must not silently synthesize predictions");

    const voxel4d::RecordedLabelInferenceProvider recorded;
    test.expect(recorded.is_available(),
                "Recorded-label provider must be available without a model");
    const voxel4d::SemanticInferenceResult recorded_result =
        recorded.infer({annotated, unannotated});
    test.expect(recorded_result.status == voxel4d::InferenceStatus::kSuccess &&
                    recorded_result.predictions.size() == 1U,
                "Recorded-label provider must return only externally annotated samples");
    if (!recorded_result.predictions.empty()) {
        const voxel4d::SemanticPrediction& prediction = recorded_result.predictions.front();
        test.expect(prediction.sample_index == 0U && prediction.semantic_label == 4,
                    "Recorded-label provider must preserve sample index and label");
        test.expect(prediction.is_valid(2U),
                    "Valid prediction must satisfy the input-aligned contract");
        test.expect(!prediction.is_valid(0U),
                    "Prediction must reject an input count that excludes its sample index");
    }

    return test.failures() == 0 ? 0 : 1;
}
