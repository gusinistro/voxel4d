#include "tum_rgbd_dataset.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifndef VOXEL4D_HAVE_PNG
#define VOXEL4D_HAVE_PNG 0
#endif

#if VOXEL4D_HAVE_PNG
#include <png.h>
#endif

namespace voxel4d {
namespace {

struct TimestampedPath {
    double timestamp_seconds{0.0};
    std::string relative_path{};
};

std::vector<std::string> tokenize_data_line(const std::string& line) {
    if (line.empty() || line.front() == '#') {
        return {};
    }
    std::stringstream stream(line);
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

double parse_finite_double(const std::string& text, const std::string& field_name) {
    std::size_t consumed = 0U;
    double parsed = 0.0;
    try {
        parsed = std::stod(text, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid " + field_name + " value: " + text);
    }
    if (consumed != text.size() || !std::isfinite(parsed)) {
        throw std::invalid_argument("Invalid " + field_name + " value: " + text);
    }
    return parsed;
}

std::vector<TimestampedPath> read_timestamped_paths(const std::filesystem::path& file_path) {
    std::ifstream input(file_path);
    if (!input.is_open()) {
        throw std::runtime_error("Unable to open TUM association file: " + file_path.string());
    }
    std::vector<TimestampedPath> records;
    std::string line;
    double previous_timestamp = -std::numeric_limits<double>::infinity();
    std::size_t line_number = 0U;
    while (std::getline(input, line)) {
        ++line_number;
        const std::vector<std::string> tokens = tokenize_data_line(line);
        if (tokens.empty()) {
            continue;
        }
        if (tokens.size() != 2U) {
            throw std::invalid_argument("Expected timestamp and path at " + file_path.string() +
                                        " line " + std::to_string(line_number));
        }
        const double timestamp = parse_finite_double(tokens.at(0), "timestamp");
        if (timestamp <= previous_timestamp || tokens.at(1).empty()) {
            throw std::invalid_argument("TUM association timestamps must be strictly increasing");
        }
        previous_timestamp = timestamp;
        records.push_back(TimestampedPath{timestamp, tokens.at(1)});
    }
    if (records.empty()) {
        throw std::invalid_argument("TUM association file must contain at least one record");
    }
    return records;
}

TimestampNanoseconds seconds_to_nanoseconds(const double timestamp_seconds) {
    if (!std::isfinite(timestamp_seconds) || timestamp_seconds < 0.0 ||
        timestamp_seconds >
            static_cast<double>(std::numeric_limits<TimestampNanoseconds>::max()) * 1.0e-9) {
        throw std::invalid_argument("TUM timestamp cannot be represented in nanoseconds");
    }
    return static_cast<TimestampNanoseconds>(std::llround(timestamp_seconds * 1.0e9));
}

#if VOXEL4D_HAVE_PNG
PnmImage read_png(const std::string& file_path, const bool require_color) {
    if (file_path.empty()) {
        throw std::invalid_argument("PNG file path must not be empty");
    }
    png_image image{};
    image.version = PNG_IMAGE_VERSION;
    if (png_image_begin_read_from_file(&image, file_path.c_str()) == 0) {
        throw std::runtime_error("Unable to read PNG header: " + file_path);
    }

    const bool source_is_color = (image.format & PNG_FORMAT_FLAG_COLOR) != 0;
    if (require_color && !source_is_color) {
        png_image_free(&image);
        throw std::invalid_argument("Expected a color PNG image: " + file_path);
    }
    if (!require_color && source_is_color) {
        png_image_free(&image);
        throw std::invalid_argument("Expected a grayscale PNG depth image: " + file_path);
    }

    const int width = static_cast<int>(image.width);
    const int height = static_cast<int>(image.height);
    if (width <= 0 || height <= 0) {
        png_image_free(&image);
        throw std::invalid_argument("PNG dimensions are invalid: " + file_path);
    }

    if (require_color) {
        image.format = PNG_FORMAT_RGB;
        std::vector<png_byte> raw(PNG_IMAGE_SIZE(image));
        if (png_image_finish_read(&image, nullptr, raw.data(), 0, nullptr) == 0) {
            const std::string message = image.message;
            png_image_free(&image);
            throw std::runtime_error("Unable to decode PNG image: " + message);
        }
        PnmImage decoded{width, height, 3, 255U, {}};
        decoded.samples.reserve(raw.size());
        for (const png_byte sample : raw) {
            decoded.samples.push_back(static_cast<std::uint16_t>(sample));
        }
        return decoded;
    }

    image.format = PNG_FORMAT_LINEAR_Y;
    const std::size_t sample_count =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<png_uint_16> raw(sample_count);
    if (png_image_finish_read(&image, nullptr, raw.data(), 0, nullptr) == 0) {
        const std::string message = image.message;
        png_image_free(&image);
        throw std::runtime_error("Unable to decode PNG depth image: " + message);
    }
    PnmImage decoded{width, height, 1, std::numeric_limits<std::uint16_t>::max(), {}};
    decoded.samples.reserve(raw.size());
    for (const png_uint_16 sample : raw) {
        decoded.samples.push_back(static_cast<std::uint16_t>(sample));
    }
    return decoded;
}
#endif

}  // namespace

bool TumGroundTruthPose::is_valid() const {
    return std::isfinite(timestamp_seconds) && timestamp_seconds >= 0.0 &&
           world_from_camera.is_valid();
}

bool TumRgbdAssociation::is_valid() const {
    return std::isfinite(color_timestamp_seconds) && std::isfinite(depth_timestamp_seconds) &&
           color_timestamp_seconds >= 0.0 && depth_timestamp_seconds >= 0.0 &&
           !color_path.empty() && !depth_path.empty();
}

bool PngImageCodec::is_available() {
#if VOXEL4D_HAVE_PNG
    return true;
#else
    return false;
#endif
}

PnmImage PngImageCodec::read_rgb8(const std::string& file_path) {
#if VOXEL4D_HAVE_PNG
    return read_png(file_path, true);
#else
    static_cast<void>(file_path);
    throw std::runtime_error(
        "PNG support is unavailable; rebuild with a system libpng development package");
#endif
}

PnmImage PngImageCodec::read_gray16(const std::string& file_path) {
#if VOXEL4D_HAVE_PNG
    return read_png(file_path, false);
#else
    static_cast<void>(file_path);
    throw std::runtime_error(
        "PNG support is unavailable; rebuild with a system libpng development package");
#endif
}

std::vector<TumRgbdAssociation> TumRgbdDataset::read_associations(
    const std::string& sequence_directory, const double maximum_timestamp_difference_seconds) {
    if (sequence_directory.empty() || !std::isfinite(maximum_timestamp_difference_seconds) ||
        maximum_timestamp_difference_seconds < 0.0) {
        throw std::invalid_argument("TUM sequence directory and timestamp tolerance must be valid");
    }
    const std::filesystem::path root(sequence_directory);
    const std::vector<TimestampedPath> colors = read_timestamped_paths(root / "rgb.txt");
    const std::vector<TimestampedPath> depths = read_timestamped_paths(root / "depth.txt");

    std::vector<TumRgbdAssociation> associations;
    std::size_t depth_index = 0U;
    for (const TimestampedPath& color : colors) {
        while (depth_index + 1U < depths.size() &&
               std::fabs(depths.at(depth_index + 1U).timestamp_seconds - color.timestamp_seconds) <=
                   std::fabs(depths.at(depth_index).timestamp_seconds - color.timestamp_seconds)) {
            ++depth_index;
        }
        const TimestampedPath& depth = depths.at(depth_index);
        if (std::fabs(depth.timestamp_seconds - color.timestamp_seconds) <=
            maximum_timestamp_difference_seconds) {
            associations.push_back(TumRgbdAssociation{
                color.timestamp_seconds, depth.timestamp_seconds,
                (root / color.relative_path).string(), (root / depth.relative_path).string()});
        }
    }
    if (associations.empty()) {
        throw std::invalid_argument("No TUM RGB/depth pairs satisfy the timestamp tolerance");
    }
    return associations;
}

std::vector<TumGroundTruthPose> TumRgbdDataset::read_ground_truth(
    const std::string& sequence_directory) {
    if (sequence_directory.empty()) {
        throw std::invalid_argument("TUM sequence directory must not be empty");
    }
    const std::filesystem::path file_path =
        std::filesystem::path(sequence_directory) / "groundtruth.txt";
    std::ifstream input(file_path);
    if (!input.is_open()) {
        throw std::runtime_error("Unable to open TUM ground-truth file: " + file_path.string());
    }

    std::vector<TumGroundTruthPose> poses;
    std::string line;
    double previous_timestamp = -std::numeric_limits<double>::infinity();
    std::size_t line_number = 0U;
    while (std::getline(input, line)) {
        ++line_number;
        const std::vector<std::string> tokens = tokenize_data_line(line);
        if (tokens.empty()) {
            continue;
        }
        if (tokens.size() != 8U) {
            throw std::invalid_argument("Expected eight ground-truth values at line " +
                                        std::to_string(line_number));
        }
        const double timestamp = parse_finite_double(tokens.at(0), "ground-truth timestamp");
        if (timestamp <= previous_timestamp) {
            throw std::invalid_argument("Ground-truth timestamps must be strictly increasing");
        }
        previous_timestamp = timestamp;
        SensorPose pose{};
        pose.translation_meters =
            glm::vec3(static_cast<float>(parse_finite_double(tokens.at(1), "tx")),
                      static_cast<float>(parse_finite_double(tokens.at(2), "ty")),
                      static_cast<float>(parse_finite_double(tokens.at(3), "tz")));
        pose.world_from_sensor =
            glm::quat(static_cast<float>(parse_finite_double(tokens.at(7), "qw")),
                      static_cast<float>(parse_finite_double(tokens.at(4), "qx")),
                      static_cast<float>(parse_finite_double(tokens.at(5), "qy")),
                      static_cast<float>(parse_finite_double(tokens.at(6), "qz")));
        const TumGroundTruthPose record{timestamp, pose.normalized()};
        if (!record.is_valid()) {
            throw std::invalid_argument("Invalid TUM ground-truth record");
        }
        poses.push_back(record);
    }
    if (poses.empty()) {
        throw std::invalid_argument("TUM ground-truth file must contain at least one pose");
    }
    return poses;
}

DecodedRgbdFrame TumRgbdDataset::load_frame(const TumRgbdAssociation& association,
                                            const CalibratedCamera& camera) {
    if (!association.is_valid() || !camera.is_valid()) {
        throw std::invalid_argument("TUM association and calibration must be valid");
    }
    const RecordedCameraFrame recorded{camera.camera_id,
                                       seconds_to_nanoseconds(association.color_timestamp_seconds),
                                       association.color_path, association.depth_path};
    const CalibratedRecordedFrame calibrated{&camera, recorded};
    DecodedRgbdFrame decoded{calibrated, PngImageCodec::read_rgb8(association.color_path),
                             PngImageCodec::read_gray16(association.depth_path),
                             kDepthScaleMetersPerUnit};
    decoded.depth_convention = DepthConvention::kOpticalAxis;
    if (!decoded.is_valid()) {
        throw std::invalid_argument(
            "TUM RGB-D image dimensions do not match the supplied calibration");
    }
    return decoded;
}

}  // namespace voxel4d
