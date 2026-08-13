#include "pnm_image.h"

#include <cctype>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

namespace voxel4d {
namespace {

std::string read_token(std::istream& input) {
    std::string token;
    char character = '\0';
    while (input.get(character)) {
        if (character == '#') {
            input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        if (!std::isspace(static_cast<unsigned char>(character))) {
            token.push_back(character);
            break;
        }
    }
    while (input.get(character)) {
        if (std::isspace(static_cast<unsigned char>(character))) {
            break;
        }
        if (character == '#') {
            input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            break;
        }
        token.push_back(character);
    }
    return token;
}

int parse_positive_int(const std::string& token, const char* field_name) {
    std::size_t consumed = 0U;
    int value = 0;
    try {
        value = std::stoi(token, &consumed, 10);
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string("Invalid PNM ") + field_name);
    }
    if (consumed != token.size() || value <= 0) {
        throw std::invalid_argument(std::string("Invalid PNM ") + field_name);
    }
    return value;
}

std::size_t checked_sample_count(const int width, const int height, const int channels) {
    const std::size_t width_size = static_cast<std::size_t>(width);
    const std::size_t height_size = static_cast<std::size_t>(height);
    const std::size_t channel_size = static_cast<std::size_t>(channels);
    if (width_size > std::numeric_limits<std::size_t>::max() / height_size ||
        width_size * height_size > std::numeric_limits<std::size_t>::max() / channel_size) {
        throw std::invalid_argument("PNM dimensions overflow sample count");
    }
    return width_size * height_size * channel_size;
}

}  // namespace

bool PnmImage::is_valid() const {
    if (width_pixels <= 0 || height_pixels <= 0 || (channel_count != 1 && channel_count != 3) ||
        maximum_value == 0U) {
        return false;
    }
    try {
        return samples.size() == checked_sample_count(width_pixels, height_pixels, channel_count);
    } catch (const std::exception&) {
        return false;
    }
}

std::uint16_t PnmImage::sample_at(const int x_pixels, const int y_pixels, const int channel) const {
    if (!is_valid() || x_pixels < 0 || x_pixels >= width_pixels || y_pixels < 0 ||
        y_pixels >= height_pixels || channel < 0 || channel >= channel_count) {
        throw std::out_of_range("PNM sample coordinate is out of range");
    }
    const std::size_t index =
        (static_cast<std::size_t>(y_pixels) * static_cast<std::size_t>(width_pixels) +
         static_cast<std::size_t>(x_pixels)) *
            static_cast<std::size_t>(channel_count) +
        static_cast<std::size_t>(channel);
    return samples.at(index);
}

bool DecodedRgbdFrame::is_valid() const {
    return recorded_frame.is_valid() && color.is_valid() && depth.is_valid() &&
           color.channel_count == 3 && depth.channel_count == 1 &&
           color.width_pixels == depth.width_pixels && color.height_pixels == depth.height_pixels &&
           color.width_pixels == recorded_frame.camera->intrinsics.width_pixels &&
           color.height_pixels == recorded_frame.camera->intrinsics.height_pixels &&
           std::isfinite(depth_scale_meters_per_unit) && depth_scale_meters_per_unit > 0.0F;
}

float DecodedRgbdFrame::depth_meters_at(const int x_pixels, const int y_pixels) const {
    if (!is_valid()) {
        throw std::logic_error("Cannot sample an invalid decoded RGB-D frame");
    }
    return static_cast<float>(depth.sample_at(x_pixels, y_pixels, 0)) * depth_scale_meters_per_unit;
}

PnmImage PnmImageCodec::read(const std::string& file_path) {
    if (file_path.empty()) {
        throw std::invalid_argument("PNM file path must not be empty");
    }
    std::ifstream input(file_path, std::ios::binary);
    if (!input.is_open()) {
        throw std::runtime_error("Unable to open PNM image: " + file_path);
    }
    const std::string magic = read_token(input);
    const int channels = magic == "P5" ? 1 : (magic == "P6" ? 3 : 0);
    if (channels == 0) {
        throw std::invalid_argument("Only binary P5 and P6 PNM images are supported");
    }
    const int width = parse_positive_int(read_token(input), "width");
    const int height = parse_positive_int(read_token(input), "height");
    const int maximum_value = parse_positive_int(read_token(input), "maximum value");
    if (maximum_value > static_cast<int>(std::numeric_limits<std::uint16_t>::max())) {
        throw std::invalid_argument("PNM maximum value exceeds 16-bit storage");
    }

    const std::size_t sample_count = checked_sample_count(width, height, channels);
    PnmImage image{width, height, channels, static_cast<std::uint16_t>(maximum_value), {}};
    image.samples.resize(sample_count);
    const bool uses_two_bytes = maximum_value > 255;
    for (std::size_t index = 0U; index < sample_count; ++index) {
        const int high_byte = input.get();
        if (high_byte == std::char_traits<char>::eof()) {
            throw std::runtime_error("PNM pixel data ended unexpectedly");
        }
        if (uses_two_bytes) {
            const int low_byte = input.get();
            if (low_byte == std::char_traits<char>::eof()) {
                throw std::runtime_error("PNM pixel data ended unexpectedly");
            }
            image.samples.at(index) = static_cast<std::uint16_t>((high_byte << 8) | low_byte);
        } else {
            image.samples.at(index) = static_cast<std::uint16_t>(high_byte);
        }
        if (image.samples.at(index) > image.maximum_value) {
            throw std::invalid_argument("PNM sample exceeds declared maximum value");
        }
    }
    return image;
}

bool PnmImageCodec::write(const PnmImage& image, const std::string& file_path) {
    if (!image.is_valid() || file_path.empty()) {
        return false;
    }
    std::ofstream output(file_path, std::ios::binary);
    if (!output.is_open()) {
        return false;
    }
    output << (image.channel_count == 1 ? "P5" : "P6") << '\n'
           << image.width_pixels << ' ' << image.height_pixels << '\n'
           << image.maximum_value << '\n';
    const bool uses_two_bytes = image.maximum_value > 255U;
    for (const std::uint16_t sample : image.samples) {
        if (sample > image.maximum_value) {
            return false;
        }
        if (uses_two_bytes) {
            output.put(static_cast<char>((sample >> 8U) & 0xFFU));
        }
        output.put(static_cast<char>(sample & 0xFFU));
    }
    return output.good();
}

DecodedRgbdFrame PnmImageCodec::load_calibrated_rgbd(const CalibratedRecordedFrame& recorded_frame,
                                                     const float depth_scale_meters_per_unit) {
    if (!recorded_frame.is_valid() || !std::isfinite(depth_scale_meters_per_unit) ||
        depth_scale_meters_per_unit <= 0.0F) {
        throw std::invalid_argument("Recorded RGB-D frame and depth scale must be valid");
    }
    DecodedRgbdFrame decoded{recorded_frame, read(recorded_frame.frame.color_path),
                             read(recorded_frame.frame.depth_path), depth_scale_meters_per_unit};
    if (!decoded.is_valid()) {
        throw std::invalid_argument(
            "Decoded RGB-D dimensions or channels do not match calibration");
    }
    return decoded;
}

}  // namespace voxel4d
