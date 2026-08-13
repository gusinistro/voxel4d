#pragma once

#include <string>
#include <vector>

#include "sensor_observation.h"

namespace voxel4d {

/**
 * @brief Strict portable CSV adapter for replaying previously recorded observations.
 *
 * The adapter serializes one spatial sample per row and reconstructs adjacent
 * rows sharing the same observation metadata into one SensorObservation. IMU
 * observations use one row. It provides a reproducible interchange boundary,
 * not a live hardware driver or a general CSV dialect parser.
 */
class RecordedObservationCsv {
   public:
    /** @return false when the output path cannot be opened or an observation is invalid. */
    [[nodiscard]] static bool write(const std::string& output_path,
                                    const std::vector<SensorObservation>& observations);

    /** @throws std::runtime_error when the file or row format is invalid. */
    [[nodiscard]] static std::vector<SensorObservation> read(const std::string& input_path);
};

}  // namespace voxel4d
