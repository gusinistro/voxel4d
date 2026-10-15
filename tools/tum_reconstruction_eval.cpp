#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "image_visual_odometry.h"
#include "rgbd_temporal_integrator.h"
#include "tum_rgbd_dataset.h"

namespace {
struct Configuration {
    std::string dataset_directory{};
    std::string output_csv_path{"tum_reconstruction_eval.csv"};
    std::size_t maximum_frame_count{10U};
    std::size_t octree_depth{4U};
    float root_size_meters{8.0F};
    int maximum_features{300};
    int search_radius_pixels{5};
    float maximum_patch_error{0.05F};
    int displacement_inlier_tolerance_pixels{1};
    float maximum_geometric_residual_meters{0.05F};
};

[[nodiscard]] int positive_int(const std::string& value, const std::string& option) {
    const int parsed = std::stoi(value);
    if (parsed <= 0) {
        throw std::invalid_argument(option + " must be positive");
    }
    return parsed;
}

[[nodiscard]] float positive_float(const std::string& value, const std::string& option) {
    const float parsed = std::stof(value);
    if (!std::isfinite(parsed) || parsed <= 0.0F) {
        throw std::invalid_argument(option + " must be finite and positive");
    }
    return parsed;
}

[[nodiscard]] Configuration parse_arguments(const int argument_count, char* argument_values[]) {
    Configuration config{};
    for (int index = 1; index < argument_count; ++index) {
        const std::string option(argument_values[index]);
        if (option == "--help") {
            std::cout
                << "Usage: voxel4d_tum_reconstruction_eval --dataset DIRECTORY [options]\n"
                << "Options:\n"
                << "  --output FILE                 CSV output path (default: "
                   "tum_reconstruction_eval.csv)\n"
                << "  --max-frames N               Frames to integrate (default: 10)\n"
                << "  --octree-depth N             SVO depth (default: 4)\n"
                << "  --root-size M                SVO root cube size in meters (default: 8)\n"
                << "  --max-features N             RGB-D odometry features (default: 300)\n"
                << "  --search-radius N            Patch search radius in pixels (default: 5)\n"
                << "  --max-patch-error X          Patch error limit (default: 0.05)\n"
                << "  --displacement-tolerance N   Median-flow tolerance in pixels (default: 1)\n"
                << "  --max-geometric-residual M   3D outlier threshold in meters (default: "
                   "0.05)\n";
            std::exit(0);
        }
        if (index + 1 >= argument_count) {
            throw std::invalid_argument("Missing value for " + option);
        }
        const std::string value(argument_values[++index]);
        if (option == "--dataset") {
            config.dataset_directory = value;
        } else if (option == "--output") {
            config.output_csv_path = value;
        } else if (option == "--max-frames") {
            config.maximum_frame_count = static_cast<std::size_t>(positive_int(value, option));
        } else if (option == "--octree-depth") {
            config.octree_depth = static_cast<std::size_t>(positive_int(value, option));
        } else if (option == "--root-size") {
            config.root_size_meters = positive_float(value, option);
        } else if (option == "--max-features") {
            config.maximum_features = positive_int(value, option);
        } else if (option == "--search-radius") {
            config.search_radius_pixels = positive_int(value, option);
        } else if (option == "--max-patch-error") {
            config.maximum_patch_error = positive_float(value, option);
        } else if (option == "--displacement-tolerance") {
            config.displacement_inlier_tolerance_pixels = positive_int(value, option);
        } else if (option == "--max-geometric-residual") {
            config.maximum_geometric_residual_meters = positive_float(value, option);
        } else {
            throw std::invalid_argument("Unknown option: " + option);
        }
    }
    if (config.dataset_directory.empty()) {
        throw std::invalid_argument("--dataset is required");
    }
    return config;
}

[[nodiscard]] voxel4d::CalibratedCamera tum_fr1_camera() {
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = "tum_fr1_rgb";
    camera.intrinsics = voxel4d::CameraIntrinsics{640, 480, 517.3F, 516.5F, 318.6F, 255.3F};
    return camera;
}

void write_header(std::ofstream& output) {
    output << "frame_index,timestamp_seconds,status,valid_depth_samples,inserted_samples,"
              "candidate_feature_count,matched_feature_count,inlier_feature_count,"
              "pose_tx_m,pose_ty_m,pose_tz_m,snapshot_count\n";
}

void write_row(std::ofstream& output, const std::size_t frame_index,
               const voxel4d::TumRgbdAssociation& association, const std::string& status,
               const voxel4d::RgbdIntegrationResult& integration,
               const voxel4d::ImageVisualOdometryResult& odometry, const voxel4d::SensorPose& pose,
               const std::size_t snapshot_count) {
    output << frame_index << ',' << std::fixed << std::setprecision(9)
           << association.color_timestamp_seconds << ',' << status << ','
           << integration.valid_depth_samples << ',' << integration.inserted_samples << ','
           << odometry.candidate_feature_count << ',' << odometry.matched_feature_count << ','
           << odometry.inlier_feature_count << ',' << pose.translation_meters.x << ','
           << pose.translation_meters.y << ',' << pose.translation_meters.z << ',' << snapshot_count
           << '\n';
}
}  // namespace

int main(const int argument_count, char* argument_values[]) {
    try {
        const Configuration config = parse_arguments(argument_count, argument_values);
        if (!std::filesystem::is_directory(config.dataset_directory)) {
            throw std::invalid_argument("Dataset directory does not exist: " +
                                        config.dataset_directory);
        }
        const auto associations =
            voxel4d::TumRgbdDataset::read_associations(config.dataset_directory, 0.02);
        if (associations.empty()) {
            throw std::runtime_error("Dataset contains no RGB-D associations");
        }
        const auto camera = tum_fr1_camera();
        const voxel4d::ImageVisualOdometry odometry(
            config.maximum_features, config.search_radius_pixels, config.maximum_patch_error,
            config.displacement_inlier_tolerance_pixels, config.maximum_geometric_residual_meters);
        const voxel4d::RgbdTemporalIntegrator integrator(config.octree_depth,
                                                         config.root_size_meters);
        TemporalVoxelMap temporal_map(config.maximum_frame_count);
        std::ofstream output(config.output_csv_path);
        if (!output.is_open()) {
            throw std::runtime_error("Unable to open CSV output: " + config.output_csv_path);
        }
        write_header(output);

        const std::size_t frame_count = std::min(config.maximum_frame_count, associations.size());
        voxel4d::DecodedRgbdFrame previous_frame =
            voxel4d::TumRgbdDataset::load_frame(associations.front(), camera);
        voxel4d::SensorPose world_from_camera{};
        const auto first_integration = integrator.integrate(
            previous_frame, world_from_camera,
            previous_frame.recorded_frame.frame.timestamp_nanoseconds, temporal_map);
        write_row(output, 0U, associations.front(), "integrated", first_integration,
                  voxel4d::ImageVisualOdometryResult{}, world_from_camera,
                  temporal_map.get_snapshot_count());
        std::size_t integrated_count = 1U;
        std::size_t failed_count = 0U;
        for (std::size_t index = 1U; index < frame_count; ++index) {
            const auto current_frame =
                voxel4d::TumRgbdDataset::load_frame(associations.at(index), camera);
            const auto estimate = odometry.estimate_rgbd_motion(previous_frame, current_frame);
            if (!estimate.success) {
                ++failed_count;
                write_row(output, index, associations.at(index), "odometry_failed",
                          voxel4d::RgbdIntegrationResult{}, estimate, world_from_camera,
                          temporal_map.get_snapshot_count());
                continue;
            }
            // VO estimates current_from_previous. Convert it to the next camera-to-world pose.
            world_from_camera =
                world_from_camera.compose(estimate.rigid_motion.current_from_previous.inverse());
            const auto integration = integrator.integrate(
                current_frame, world_from_camera,
                current_frame.recorded_frame.frame.timestamp_nanoseconds, temporal_map);
            write_row(output, index, associations.at(index), "integrated", integration, estimate,
                      world_from_camera, temporal_map.get_snapshot_count());
            previous_frame = current_frame;
            ++integrated_count;
        }
        std::cout << "frames_requested=" << frame_count << '\n'
                  << "frames_integrated=" << integrated_count << '\n'
                  << "odometry_failures=" << failed_count << '\n'
                  << "snapshots_retained=" << temporal_map.get_snapshot_count() << '\n'
                  << "csv_output=" << config.output_csv_path << '\n';
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "TUM temporal reconstruction failed: " << exception.what() << '\n';
        return 1;
    }
}
