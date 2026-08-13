#include "recorded_multiview_dataset.h"

#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace voxel4d {
namespace {

constexpr const char* kManifestHeader = "camera_id,timestamp_nanoseconds,color_path,depth_path";

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

TimestampNanoseconds parse_timestamp(const std::string& field, const std::size_t line_number) {
    std::size_t consumed = 0U;
    long long parsed = 0;
    try {
        parsed = std::stoll(field, &consumed, 10);
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid timestamp at manifest line " +
                                    std::to_string(line_number));
    }
    if (consumed != field.size() || parsed < 0 ||
        parsed > std::numeric_limits<TimestampNanoseconds>::max()) {
        throw std::invalid_argument("Invalid timestamp at manifest line " +
                                    std::to_string(line_number));
    }
    return static_cast<TimestampNanoseconds>(parsed);
}

}  // namespace

bool RecordedCameraFrame::is_valid() const {
    return !camera_id.empty() && timestamp_nanoseconds >= 0 && !color_path.empty() &&
           !depth_path.empty();
}

bool CalibratedRecordedFrame::is_valid() const {
    return camera != nullptr && camera->is_valid() && frame.is_valid() &&
           frame.camera_id == camera->camera_id;
}

std::vector<RecordedCameraFrame> RecordedMultiviewDataset::read_manifest_csv(
    const std::string& file_path) {
    if (file_path.empty()) {
        throw std::invalid_argument("Manifest file path must not be empty");
    }
    std::ifstream input(file_path);
    if (!input.is_open()) {
        throw std::runtime_error("Unable to open multiview manifest: " + file_path);
    }

    std::string line;
    if (!std::getline(input, line) || line != kManifestHeader) {
        throw std::invalid_argument("Multiview manifest has an invalid header");
    }

    std::vector<RecordedCameraFrame> frames;
    std::size_t line_number = 1U;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty()) {
            throw std::invalid_argument("Empty line in multiview manifest at line " +
                                        std::to_string(line_number));
        }
        const std::vector<std::string> fields = split_csv_line(line);
        if (fields.size() != 4U) {
            throw std::invalid_argument("Expected four fields at manifest line " +
                                        std::to_string(line_number));
        }
        RecordedCameraFrame frame{fields.at(0), parse_timestamp(fields.at(1), line_number),
                                  fields.at(2), fields.at(3)};
        if (!frame.is_valid()) {
            throw std::invalid_argument("Invalid frame at manifest line " +
                                        std::to_string(line_number));
        }
        frames.push_back(std::move(frame));
    }
    if (frames.empty()) {
        throw std::invalid_argument("Multiview manifest must contain at least one frame");
    }
    return frames;
}

bool RecordedMultiviewDataset::write_manifest_csv(const std::string& file_path,
                                                  const std::vector<RecordedCameraFrame>& frames) {
    if (file_path.empty() || frames.empty()) {
        return false;
    }
    for (const RecordedCameraFrame& frame : frames) {
        if (!frame.is_valid() || frame.camera_id.find(',') != std::string::npos ||
            frame.color_path.find(',') != std::string::npos ||
            frame.depth_path.find(',') != std::string::npos) {
            return false;
        }
    }

    std::ofstream output(file_path);
    if (!output.is_open()) {
        return false;
    }
    output << kManifestHeader << '\n';
    for (const RecordedCameraFrame& frame : frames) {
        output << frame.camera_id << ',' << frame.timestamp_nanoseconds << ',' << frame.color_path
               << ',' << frame.depth_path << '\n';
    }
    return output.good();
}

std::vector<CalibratedRecordedFrame> RecordedMultiviewDataset::bind_calibrations(
    const std::vector<RecordedCameraFrame>& frames, const std::vector<CalibratedCamera>& cameras) {
    if (frames.empty() || cameras.empty()) {
        throw std::invalid_argument("Frames and calibrated cameras must not be empty");
    }

    std::unordered_map<std::string, const CalibratedCamera*> camera_by_id;
    camera_by_id.reserve(cameras.size());
    for (const CalibratedCamera& camera : cameras) {
        if (!camera.is_valid() || !camera_by_id.emplace(camera.camera_id, &camera).second) {
            throw std::invalid_argument(
                "Calibrated cameras must be valid and have unique identifiers");
        }
    }

    std::unordered_map<std::string, TimestampNanoseconds> last_timestamp_by_camera;
    std::vector<CalibratedRecordedFrame> bound_frames;
    bound_frames.reserve(frames.size());
    TimestampNanoseconds previous_timestamp = -1;
    for (const RecordedCameraFrame& frame : frames) {
        if (!frame.is_valid() || frame.timestamp_nanoseconds < previous_timestamp) {
            throw std::invalid_argument("Manifest frames must be valid and globally time ordered");
        }
        previous_timestamp = frame.timestamp_nanoseconds;
        const auto camera_iterator = camera_by_id.find(frame.camera_id);
        if (camera_iterator == camera_by_id.end()) {
            throw std::invalid_argument("Manifest frame references an unknown camera: " +
                                        frame.camera_id);
        }
        const auto previous_for_camera = last_timestamp_by_camera.find(frame.camera_id);
        if (previous_for_camera != last_timestamp_by_camera.end() &&
            frame.timestamp_nanoseconds <= previous_for_camera->second) {
            throw std::invalid_argument("Timestamps must be strictly increasing for each camera");
        }
        last_timestamp_by_camera[frame.camera_id] = frame.timestamp_nanoseconds;
        bound_frames.push_back(CalibratedRecordedFrame{camera_iterator->second, frame});
    }
    return bound_frames;
}

}  // namespace voxel4d
