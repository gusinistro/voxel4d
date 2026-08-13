#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <glm/common.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "image_visual_odometry.h"
#include "tum_rgbd_dataset.h"

namespace {

struct EvaluationConfig {
    std::string dataset_directory{};
    std::string output_csv_path{"tum_odometry_eval.csv"};
    std::size_t maximum_pair_count{50U};
    double association_tolerance_seconds{0.02};
    double ground_truth_tolerance_seconds{0.02};
    int maximum_features{300};
    int search_radius_pixels{5};
    float maximum_patch_error{0.05F};
    int displacement_inlier_tolerance_pixels{1};
};

struct GroundTruthMatch {
    const voxel4d::TumGroundTruthPose* pose{nullptr};
    double timestamp_delta_seconds{std::numeric_limits<double>::infinity()};
};

struct PairMetrics {
    double previous_timestamp_seconds{0.0};
    double current_timestamp_seconds{0.0};
    double previous_ground_truth_delta_seconds{0.0};
    double current_ground_truth_delta_seconds{0.0};
    std::string status{};
    std::size_t candidate_feature_count{0U};
    std::size_t matched_feature_count{0U};
    std::size_t inlier_feature_count{0U};
    double estimated_rmse_meters{0.0};
    double translation_error_meters{std::numeric_limits<double>::quiet_NaN()};
    double rotation_error_degrees{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] int parse_positive_int(const std::string& text, const std::string& option_name) {
    const int value = std::stoi(text);
    if (value <= 0) {
        throw std::invalid_argument(option_name + " must be positive");
    }
    return value;
}

[[nodiscard]] double parse_positive_double(const std::string& text,
                                           const std::string& option_name) {
    const double value = std::stod(text);
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(option_name + " must be finite and positive");
    }
    return value;
}

[[nodiscard]] EvaluationConfig parse_arguments(const int argument_count, char* argument_values[]) {
    EvaluationConfig config{};
    for (int index = 1; index < argument_count; ++index) {
        const std::string option(argument_values[index]);
        if (option == "--help") {
            std::cout
                << "Usage: voxel4d_tum_odometry_eval --dataset DIRECTORY [options]\n"
                << "Options:\n"
                << "  --output FILE                 CSV output path (default: "
                   "tum_odometry_eval.csv)\n"
                << "  --max-pairs N                Consecutive pairs to evaluate (default: 50)\n"
                << "  --association-tolerance S    RGB/depth association tolerance in seconds "
                   "(default: 0.02)\n"
                << "  --ground-truth-tolerance S   Ground-truth timestamp tolerance in seconds "
                   "(default: 0.02)\n"
                << "  --max-features N             Gradient features per frame (default: 300)\n"
                << "  --search-radius N            Patch search radius in pixels (default: 5)\n"
                << "  --max-patch-error X          Mean squared luminance error limit (default: "
                   "0.05)\n"
                << "  --displacement-tolerance N   Median-flow inlier tolerance in pixels "
                   "(default: 1)\n";
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
        } else if (option == "--max-pairs") {
            config.maximum_pair_count =
                static_cast<std::size_t>(parse_positive_int(value, "--max-pairs"));
        } else if (option == "--association-tolerance") {
            config.association_tolerance_seconds =
                parse_positive_double(value, "--association-tolerance");
        } else if (option == "--ground-truth-tolerance") {
            config.ground_truth_tolerance_seconds =
                parse_positive_double(value, "--ground-truth-tolerance");
        } else if (option == "--max-features") {
            config.maximum_features = parse_positive_int(value, "--max-features");
        } else if (option == "--search-radius") {
            config.search_radius_pixels = parse_positive_int(value, "--search-radius");
        } else if (option == "--max-patch-error") {
            config.maximum_patch_error =
                static_cast<float>(parse_positive_double(value, "--max-patch-error"));
        } else if (option == "--displacement-tolerance") {
            config.displacement_inlier_tolerance_pixels =
                parse_positive_int(value, "--displacement-tolerance");
        } else {
            throw std::invalid_argument("Unknown option: " + option);
        }
    }
    if (config.dataset_directory.empty()) {
        throw std::invalid_argument("--dataset is required");
    }
    return config;
}

[[nodiscard]] GroundTruthMatch find_nearest_ground_truth(
    const std::vector<voxel4d::TumGroundTruthPose>& ground_truth, const double timestamp_seconds) {
    const auto after =
        std::lower_bound(ground_truth.begin(), ground_truth.end(), timestamp_seconds,
                         [](const voxel4d::TumGroundTruthPose& pose, const double timestamp) {
                             return pose.timestamp_seconds < timestamp;
                         });

    GroundTruthMatch result{};
    const auto evaluate_candidate =
        [&result, timestamp_seconds](const voxel4d::TumGroundTruthPose& candidate) {
            const double delta = std::fabs(candidate.timestamp_seconds - timestamp_seconds);
            if (delta < result.timestamp_delta_seconds) {
                result.pose = &candidate;
                result.timestamp_delta_seconds = delta;
            }
        };
    if (after != ground_truth.end()) {
        evaluate_candidate(*after);
    }
    if (after != ground_truth.begin()) {
        evaluate_candidate(*std::prev(after));
    }
    return result;
}

[[nodiscard]] double rotation_error_degrees(const voxel4d::SensorPose& estimated,
                                            const voxel4d::SensorPose& expected) {
    const glm::quat estimated_rotation = estimated.normalized().world_from_sensor;
    const glm::quat expected_rotation = expected.normalized().world_from_sensor;
    const float aligned_dot =
        glm::clamp(std::fabs(glm::dot(estimated_rotation, expected_rotation)), 0.0F, 1.0F);
    return static_cast<double>(2.0F * std::acos(aligned_dot) * 180.0F / glm::pi<float>());
}

[[nodiscard]] double median(std::vector<double> values) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    if (values.size() % 2U != 0U) {
        return values[middle];
    }
    return (values[middle - 1U] + values[middle]) * 0.5;
}

[[nodiscard]] double mean(const std::vector<double>& values) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double sum = 0.0;
    for (const double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

void write_csv_header(std::ofstream& output) {
    output << "previous_color_timestamp_seconds,current_color_timestamp_seconds,"
           << "previous_ground_truth_delta_seconds,current_ground_truth_delta_seconds,status,"
           << "candidate_feature_count,matched_feature_count,inlier_feature_count,"
           << "estimated_rmse_meters,translation_error_meters,rotation_error_degrees\n";
}

void write_csv_row(std::ofstream& output, const PairMetrics& metrics) {
    output << std::fixed << std::setprecision(9) << metrics.previous_timestamp_seconds << ','
           << metrics.current_timestamp_seconds << ','
           << metrics.previous_ground_truth_delta_seconds << ','
           << metrics.current_ground_truth_delta_seconds << ',' << metrics.status << ','
           << metrics.candidate_feature_count << ',' << metrics.matched_feature_count << ','
           << metrics.inlier_feature_count << ',' << metrics.estimated_rmse_meters << ','
           << metrics.translation_error_meters << ',' << metrics.rotation_error_degrees << '\n';
}

[[nodiscard]] voxel4d::CalibratedCamera tum_fr1_camera() {
    voxel4d::CalibratedCamera camera{};
    camera.camera_id = "tum_fr1_rgb";
    camera.intrinsics = voxel4d::CameraIntrinsics{640, 480, 517.3F, 516.5F, 318.6F, 255.3F};
    return camera;
}

}  // namespace

int main(int argument_count, char* argument_values[]) {
    try {
        const EvaluationConfig config = parse_arguments(argument_count, argument_values);
        if (!std::filesystem::is_directory(config.dataset_directory)) {
            throw std::invalid_argument("Dataset directory does not exist: " +
                                        config.dataset_directory);
        }

        const std::vector<voxel4d::TumRgbdAssociation> associations =
            voxel4d::TumRgbdDataset::read_associations(config.dataset_directory,
                                                       config.association_tolerance_seconds);
        const std::vector<voxel4d::TumGroundTruthPose> ground_truth =
            voxel4d::TumRgbdDataset::read_ground_truth(config.dataset_directory);
        if (associations.size() < 2U || ground_truth.empty()) {
            throw std::runtime_error(
                "Dataset does not contain enough associated frames and ground truth");
        }

        std::ofstream output(config.output_csv_path);
        if (!output.is_open()) {
            throw std::runtime_error("Unable to open CSV output: " + config.output_csv_path);
        }
        write_csv_header(output);

        const voxel4d::CalibratedCamera camera = tum_fr1_camera();
        const voxel4d::ImageVisualOdometry estimator(
            config.maximum_features, config.search_radius_pixels, config.maximum_patch_error,
            config.displacement_inlier_tolerance_pixels);

        const std::size_t available_pair_count = associations.size() - 1U;
        const std::size_t evaluated_pair_count =
            std::min(config.maximum_pair_count, available_pair_count);
        std::vector<double> translation_errors;
        std::vector<double> rotation_errors;
        translation_errors.reserve(evaluated_pair_count);
        rotation_errors.reserve(evaluated_pair_count);
        std::size_t successful_pair_count = 0U;
        std::size_t reference_pair_count = 0U;

        for (std::size_t index = 0U; index < evaluated_pair_count; ++index) {
            const voxel4d::TumRgbdAssociation& previous_association = associations[index];
            const voxel4d::TumRgbdAssociation& current_association = associations[index + 1U];
            const GroundTruthMatch previous_ground_truth = find_nearest_ground_truth(
                ground_truth, previous_association.color_timestamp_seconds);
            const GroundTruthMatch current_ground_truth = find_nearest_ground_truth(
                ground_truth, current_association.color_timestamp_seconds);

            PairMetrics metrics{};
            metrics.previous_timestamp_seconds = previous_association.color_timestamp_seconds;
            metrics.current_timestamp_seconds = current_association.color_timestamp_seconds;
            metrics.previous_ground_truth_delta_seconds =
                previous_ground_truth.timestamp_delta_seconds;
            metrics.current_ground_truth_delta_seconds =
                current_ground_truth.timestamp_delta_seconds;

            if (previous_ground_truth.pose == nullptr || current_ground_truth.pose == nullptr ||
                previous_ground_truth.timestamp_delta_seconds >
                    config.ground_truth_tolerance_seconds ||
                current_ground_truth.timestamp_delta_seconds >
                    config.ground_truth_tolerance_seconds) {
                metrics.status = "missing_reference";
                write_csv_row(output, metrics);
                continue;
            }
            ++reference_pair_count;

            try {
                const voxel4d::DecodedRgbdFrame previous_frame =
                    voxel4d::TumRgbdDataset::load_frame(previous_association, camera);
                const voxel4d::DecodedRgbdFrame current_frame =
                    voxel4d::TumRgbdDataset::load_frame(current_association, camera);
                const voxel4d::ImageVisualOdometryResult estimate =
                    estimator.estimate_rgbd_motion(previous_frame, current_frame);
                metrics.candidate_feature_count = estimate.candidate_feature_count;
                metrics.matched_feature_count = estimate.matched_feature_count;
                metrics.inlier_feature_count = estimate.inlier_feature_count;
                metrics.estimated_rmse_meters =
                    static_cast<double>(estimate.rigid_motion.root_mean_square_error_meters);
                if (!estimate.success) {
                    metrics.status = "odometry_failed";
                    write_csv_row(output, metrics);
                    continue;
                }

                const voxel4d::SensorPose expected_current_from_previous =
                    current_ground_truth.pose->world_from_camera.inverse().compose(
                        previous_ground_truth.pose->world_from_camera);
                const glm::vec3 translation_delta =
                    estimate.rigid_motion.current_from_previous.translation_meters -
                    expected_current_from_previous.translation_meters;
                metrics.translation_error_meters =
                    static_cast<double>(glm::length(translation_delta));
                metrics.rotation_error_degrees = rotation_error_degrees(
                    estimate.rigid_motion.current_from_previous, expected_current_from_previous);
                metrics.status = "success";
                ++successful_pair_count;
                translation_errors.push_back(metrics.translation_error_meters);
                rotation_errors.push_back(metrics.rotation_error_degrees);
                write_csv_row(output, metrics);
            } catch (const std::exception& exception) {
                metrics.status = "input_error";
                std::cerr << "Pair " << index << " input error: " << exception.what() << '\n';
                write_csv_row(output, metrics);
            }
        }

        const double success_rate_percent =
            reference_pair_count == 0U ? 0.0
                                       : 100.0 * static_cast<double>(successful_pair_count) /
                                             static_cast<double>(reference_pair_count);
        std::cout << std::fixed << std::setprecision(6) << "dataset=" << config.dataset_directory
                  << '\n'
                  << "pairs_requested=" << config.maximum_pair_count << '\n'
                  << "pairs_evaluated=" << evaluated_pair_count << '\n'
                  << "pairs_with_reference=" << reference_pair_count << '\n'
                  << "pairs_successful=" << successful_pair_count << '\n'
                  << "success_rate_percent=" << success_rate_percent << '\n'
                  << "translation_error_mean_meters=" << mean(translation_errors) << '\n'
                  << "translation_error_median_meters=" << median(translation_errors) << '\n'
                  << "rotation_error_mean_degrees=" << mean(rotation_errors) << '\n'
                  << "rotation_error_median_degrees=" << median(rotation_errors) << '\n'
                  << "csv_output=" << config.output_csv_path << '\n';
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "TUM RGB-D odometry evaluation failed: " << exception.what() << '\n';
        return 1;
    }
}
