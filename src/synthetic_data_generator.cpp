#include "synthetic_data_generator.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <glm/gtc/constants.hpp>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace {
constexpr int kPixelStride = 2;
constexpr float kCameraRadiusMeters = 8.0F;
constexpr float kObjectRadiusMeters = 2.0F;
}  // namespace

SyntheticDataGenerator::SyntheticDataGenerator(int width, int height)
    : width_(width), height_(height) {
    if (width_ <= 0 || height_ <= 0) {
        throw std::invalid_argument("image dimensions must be positive");
    }
}

std::vector<Camera> SyntheticDataGenerator::generate_camera_setup(int num_cameras) const {
    if (num_cameras <= 0) {
        throw std::invalid_argument("num_cameras must be positive");
    }

    std::vector<Camera> cameras;
    cameras.reserve(static_cast<std::size_t>(num_cameras));
    for (int index = 0; index < num_cameras; ++index) {
        const float angle =
            (2.0F * glm::pi<float>() * static_cast<float>(index)) / static_cast<float>(num_cameras);

        Camera camera{};
        camera.position = glm::vec3(kCameraRadiusMeters * std::cos(angle), 1.0F,
                                    kCameraRadiusMeters * std::sin(angle));
        camera.direction = glm::normalize(-camera.position);
        camera.up = glm::vec3(0.0F, 1.0F, 0.0F);
        camera.fov_degrees = 60.0F;
        camera.width = width_;
        camera.height = height_;
        camera.near_plane_meters = 0.1F;
        camera.far_plane_meters = 100.0F;
        cameras.push_back(camera);
    }
    return cameras;
}

float SyntheticDataGenerator::ray_sphere_intersection(const glm::vec3& ray_origin,
                                                      const glm::vec3& ray_direction,
                                                      const glm::vec3& sphere_center,
                                                      float radius) {
    const glm::vec3 offset = ray_origin - sphere_center;
    const float b = 2.0F * glm::dot(offset, ray_direction);
    const float c = glm::dot(offset, offset) - radius * radius;
    const float discriminant = b * b - 4.0F * c;
    if (discriminant < 0.0F) {
        return -1.0F;
    }

    const float root = std::sqrt(discriminant);
    const float near_hit = (-b - root) * 0.5F;
    const float far_hit = (-b + root) * 0.5F;
    if (near_hit > 0.0F) {
        return near_hit;
    }
    return far_hit > 0.0F ? far_hit : -1.0F;
}

std::vector<PixelData> SyntheticDataGenerator::generate_camera_frame(
    const Camera& camera, const glm::vec3& object_position, const glm::vec3& object_velocity,
    float object_radius, const glm::vec3& object_color) const {
    if (object_radius <= 0.0F) {
        throw std::invalid_argument("object_radius must be positive");
    }
    static_cast<void>(object_velocity);  // Reserved for future per-pixel motion-vector output.

    std::vector<PixelData> pixels;
    const glm::vec3 right = glm::normalize(glm::cross(camera.direction, camera.up));
    const glm::vec3 up = glm::normalize(glm::cross(right, camera.direction));
    const float tangent = std::tan(glm::radians(camera.fov_degrees * 0.5F));
    const float aspect = static_cast<float>(camera.width) / static_cast<float>(camera.height);

    for (int y = 0; y < camera.height; y += kPixelStride) {
        for (int x = 0; x < camera.width; x += kPixelStride) {
            const float ndc_x =
                (2.0F * static_cast<float>(x)) / static_cast<float>(camera.width) - 1.0F;
            const float ndc_y =
                1.0F - (2.0F * static_cast<float>(y)) / static_cast<float>(camera.height);
            const glm::vec3 ray_direction = glm::normalize(
                camera.direction + right * ndc_x * aspect * tangent + up * ndc_y * tangent);
            const float depth = ray_sphere_intersection(camera.position, ray_direction,
                                                        object_position, object_radius);

            if (depth >= camera.near_plane_meters && depth <= camera.far_plane_meters) {
                pixels.push_back(PixelData{
                    x,
                    y,
                    depth,
                    object_color,
                });
            }
        }
    }
    return pixels;
}

bool SyntheticDataGenerator::save_frame(const std::vector<PixelData>& pixels,
                                        const std::string& filename) const {
    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    if (!file) {
        return false;
    }

    file << "x,y,depth_m,r,g,b\n";
    for (const PixelData& pixel : pixels) {
        file << pixel.x << ',' << pixel.y << ',' << pixel.depth_meters << ',' << pixel.color.r
             << ',' << pixel.color.g << ',' << pixel.color.b << '\n';
    }
    return static_cast<bool>(file);
}

std::vector<PixelData> SyntheticDataGenerator::load_frame(const std::string& filename) const {
    std::ifstream file(filename);
    if (!file) {
        throw std::runtime_error("unable to open frame file: " + filename);
    }

    std::string line;
    if (!std::getline(file, line)) {
        throw std::runtime_error("missing CSV header in frame file: " + filename);
    }

    std::vector<PixelData> pixels;
    std::size_t line_number = 1;
    while (std::getline(file, line)) {
        ++line_number;
        if (line.empty()) {
            continue;
        }

        std::stringstream stream(line);
        std::string token;
        PixelData pixel{};
        try {
            std::getline(stream, token, ',');
            pixel.x = std::stoi(token);
            std::getline(stream, token, ',');
            pixel.y = std::stoi(token);
            std::getline(stream, token, ',');
            pixel.depth_meters = std::stof(token);
            std::getline(stream, token, ',');
            pixel.color.r = std::stof(token);
            std::getline(stream, token, ',');
            pixel.color.g = std::stof(token);
            std::getline(stream, token, ',');
            pixel.color.b = std::stof(token);
            if (stream.fail() || pixel.depth_meters <= 0.0F) {
                throw std::runtime_error("invalid field");
            }
        } catch (const std::exception&) {
            throw std::runtime_error("invalid CSV record at " + filename + ':' +
                                     std::to_string(line_number));
        }
        pixels.push_back(pixel);
    }
    return pixels;
}

void SyntheticDataGenerator::generate_moving_object_sequence(
    int num_frames, const std::string& output_directory) const {
    if (num_frames <= 0) {
        throw std::invalid_argument("num_frames must be positive");
    }

    fs::create_directories(output_directory);
    const std::vector<Camera> cameras = generate_camera_setup(2);
    const glm::vec3 object_color(0.8F, 0.2F, 0.2F);
    const glm::vec3 object_velocity(4.0F, 0.0F, 0.0F);

    for (int frame = 0; frame < num_frames; ++frame) {
        const float interpolation = static_cast<float>(frame) / static_cast<float>(num_frames - 1);
        const glm::vec3 object_position(-2.0F + 4.0F * interpolation, 0.0F, 0.0F);

        for (std::size_t camera_index = 0; camera_index < cameras.size(); ++camera_index) {
            const std::vector<PixelData> pixels =
                generate_camera_frame(cameras[camera_index], object_position, object_velocity,
                                      kObjectRadiusMeters, object_color);
            const std::string filename = output_directory + "/frame_" + std::to_string(frame) +
                                         "_cam_" + std::to_string(camera_index) + ".csv";
            if (!save_frame(pixels, filename)) {
                throw std::runtime_error("unable to save frame file: " + filename);
            }
        }
    }
}
