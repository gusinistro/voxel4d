#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "multiview_calibration.h"
#include "time_types.h"

namespace voxel4d {

/** @brief A path-based reference to one recorded color/depth camera frame. */
struct RecordedCameraFrame {
    std::string camera_id{};
    TimestampNanoseconds timestamp_nanoseconds{0};
    std::string color_path{};
    std::string depth_path{};

    [[nodiscard]] bool is_valid() const;
};

/** @brief A validated association between a recorded frame and a calibrated camera. */
struct CalibratedRecordedFrame {
    const CalibratedCamera* camera{nullptr};
    RecordedCameraFrame frame{};

    [[nodiscard]] bool is_valid() const;
};

/**
 * @brief Strict CSV adapter for replayable multiview frame manifests.
 *
 * The manifest format is `camera_id,timestamp_nanoseconds,color_path,depth_path`.
 * It deliberately stores file references only; decoding and sensor-specific data
 * quality checks remain separate adapters.
 */
class RecordedMultiviewDataset {
   public:
    [[nodiscard]] static std::vector<RecordedCameraFrame> read_manifest_csv(
        const std::string& file_path);
    [[nodiscard]] static bool write_manifest_csv(const std::string& file_path,
                                                 const std::vector<RecordedCameraFrame>& frames);

    /**
     * @brief Bind frames to provided calibrations and reject unknown cameras or unordered input.
     * @throws std::invalid_argument if the calibration set or frame sequence is invalid.
     */
    [[nodiscard]] static std::vector<CalibratedRecordedFrame> bind_calibrations(
        const std::vector<RecordedCameraFrame>& frames,
        const std::vector<CalibratedCamera>& cameras);
};

}  // namespace voxel4d
