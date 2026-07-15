#include "alignment_reconstruct/reconstruction.h"

#include "horizontal/horizontal_alignment.h"
#include "horizontal/horizontal_reconstructor.h"
#include "vertical/profile_classifier.h"
#include "vertical/vertical_reconstructor.h"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace alignment_reconstruct {
namespace {

ReconstructionResult error_result(
    StatusCode status,
    DiagnosticCode code,
    std::size_t input_index,
    std::string message) {
    ReconstructionResult result;
    result.status = status;
    result.diagnostics.push_back({
        DiagnosticSeverity::error, code, input_index, std::move(message)
    });
    return result;
}

bool finite_options(const ReconstructionOptions& options) {
    if (!std::isfinite(options.start_station_m) ||
        !std::isfinite(options.curvature_window_m) ||
        !std::isfinite(options.curvature_threshold_inverse_m) ||
        !std::isfinite(options.horizontal_optimization_tolerance) ||
        !std::isfinite(options.minimum_grade_length_m) ||
        !std::isfinite(options.maximum_up_grade) ||
        !std::isfinite(options.maximum_down_grade) ||
        !std::isfinite(options.maximum_grade_difference_per_mille) ||
        !std::isfinite(options.minimum_vertical_curve_radius_m) ||
        !std::isfinite(options.maximum_vertical_curve_radius_m) ||
        !std::isfinite(options.vertical_curve_radius_rounding_m) ||
        !std::isfinite(options.grade_rounding_per_mille) ||
        !std::isfinite(options.station_rounding_m) ||
        !std::isfinite(options.station_monotonicity_tolerance_m)) {
        return false;
    }
    for (const double threshold : options.vertical_second_difference_threshold_candidates) {
        if (!std::isfinite(threshold)) {
            return false;
        }
    }
    return true;
}

} // namespace

ReconstructionResult reconstruct(
    const std::vector<Point3D>& ordered_points,
    const ReconstructionOptions& options) {
    if (ordered_points.size() < 2) {
        return error_result(StatusCode::invalid_input, DiagnosticCode::too_few_points,
                            no_input_index, "At least two ordered XYZ points are required");
    }
    if (!finite_options(options) || options.curvature_window_m <= 0.0 ||
        options.curvature_threshold_inverse_m < 0.0 ||
        options.minimum_curve_sample_count < 1 ||
        options.maximum_spiral_classification_iterations < 1 ||
        options.horizontal_optimization_tolerance <= 0.0 ||
        options.horizontal_optimization_max_iterations < 1 ||
        options.minimum_grade_length_m < 0.0 ||
        options.minimum_vertical_curve_radius_m <= 0.0 ||
        options.maximum_vertical_curve_radius_m < options.minimum_vertical_curve_radius_m ||
        options.vertical_curve_radius_rounding_m < 0.0 ||
        options.grade_rounding_per_mille < 0.0 ||
        options.station_rounding_m < 0.0 ||
        options.station_monotonicity_tolerance_m < 0.0) {
        return error_result(StatusCode::invalid_input, DiagnosticCode::non_finite_value,
                            no_input_index, "Reconstruction options are invalid");
    }
    std::vector<Point2D> horizontal_samples;
    horizontal_samples.reserve(ordered_points.size());
    for (std::size_t index = 0; index < ordered_points.size(); ++index) {
        const auto& point = ordered_points[index];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            return error_result(StatusCode::invalid_input, DiagnosticCode::non_finite_value,
                                index, "Reconstruction point contains a non-finite coordinate");
        }
        if (index > 0 && point.x == ordered_points[index - 1].x &&
            point.y == ordered_points[index - 1].y) {
            return error_result(StatusCode::invalid_input,
                                DiagnosticCode::coincident_consecutive_points,
                                index, "Consecutive reconstruction points have identical XY");
        }
        horizontal_samples.push_back({point.x, point.y});
    }

    ReconstructionResult result;
    try {
        const detail::HorizontalReconstructor horizontal_reconstructor(options);
        result.horizontal = horizontal_reconstructor.reconstruct(horizontal_samples);
        const detail::HorizontalAlignment horizontal(result.horizontal, options.start_station_m);
        result.projected_stations_m.reserve(ordered_points.size());
        std::vector<detail::ProfileSample> vertical_samples;
        vertical_samples.reserve(ordered_points.size());
        for (std::size_t index = 0; index < ordered_points.size(); ++index) {
            const auto projection = horizontal.project(horizontal_samples[index]);
            if (!projection.valid) {
                return error_result(StatusCode::numerical_failure,
                                    DiagnosticCode::projection_failed,
                                    index, "Projection onto reconstructed horizontal alignment failed");
            }
            if (!result.projected_stations_m.empty() &&
                projection.station_m <= result.projected_stations_m.back()) {
                const double reverse = result.projected_stations_m.back() - projection.station_m;
                const std::string message = reverse > options.station_monotonicity_tolerance_m
                    ? "Reconstructed projection station reverses input order"
                    : "Reconstructed projection stations are not strictly increasing";
                return error_result(StatusCode::invalid_input,
                                    DiagnosticCode::non_monotonic_station,
                                    index, message);
            }
            result.projected_stations_m.push_back(projection.station_m);
            vertical_samples.push_back({projection.station_m, ordered_points[index].z});
        }
        const detail::VerticalReconstructor vertical_reconstructor(options);
        result.vertical = vertical_reconstructor.reconstruct(vertical_samples);
        return result;
    } catch (const std::invalid_argument& error) {
        return error_result(StatusCode::infeasible_geometry,
                            DiagnosticCode::degenerate_fit,
                            no_input_index, error.what());
    } catch (const std::exception& error) {
        return error_result(StatusCode::numerical_failure,
                            DiagnosticCode::degenerate_fit,
                            no_input_index, error.what());
    }
}

} // namespace alignment_reconstruct
