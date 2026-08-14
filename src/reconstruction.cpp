#include "alignment_reconstruct/reconstruction.h"

#include "horizontal/horizontal_alignment.h"
#include "horizontal/horizontal_reconstructor.h"
#include "vertical/profile_classifier.h"
#include "vertical/vertical_reconstructor.h"

#include <algorithm>
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
        !std::isfinite(options.minimum_vertical_curve_grade_difference_per_mille) ||
        !std::isfinite(options.minimum_vertical_curve_radius_m) ||
        !std::isfinite(options.maximum_vertical_curve_radius_m) ||
        !std::isfinite(options.vertical_curve_radius_rounding_m) ||
        !std::isfinite(options.grade_rounding_per_mille) ||
        !std::isfinite(options.station_rounding_m) ||
        !std::isfinite(options.station_monotonicity_tolerance_m) ||
        !std::isfinite(options.horizontal_curve_radius_rounding_m) ||
        !std::isfinite(options.horizontal_spiral_length_rounding_m)) {
        return false;
    }
    for (const double threshold : options.vertical_second_difference_threshold_candidates) {
        if (!std::isfinite(threshold)) {
            return false;
        }
    }
    return true;
}

bool valid_horizontal_sample_property(HorizontalSampleProperty property) {
    switch (property) {
    case HorizontalSampleProperty::straight:
    case HorizontalSampleProperty::curve:
        return true;
    }
    return false;
}

ReconstructionResult validate_curvature_graph(
    const std::vector<Point3D>& ordered_points,
    const ReconstructionOptions& options,
    const std::vector<HorizontalCurvatureSample>& curvature_graph) {
    if (curvature_graph.empty()) {
        return {};
    }
    if (curvature_graph.size() != ordered_points.size()) {
        return error_result(
            StatusCode::invalid_input,
            DiagnosticCode::curvature_graph_size_mismatch,
            no_input_index,
            "Curvature graph size must match the ordered point count");
    }
    for (std::size_t index = 0; index < curvature_graph.size(); ++index) {
        if (!std::isfinite(curvature_graph[index].curvature_inverse_m)) {
            return error_result(
                StatusCode::invalid_input,
                DiagnosticCode::non_finite_value,
                index,
                "Curvature graph contains a non-finite curvature");
        }
        if (!valid_horizontal_sample_property(curvature_graph[index].property)) {
            return error_result(
                StatusCode::invalid_input,
                DiagnosticCode::invalid_horizontal_sample_property,
                index,
                "Curvature graph contains an invalid horizontal sample property");
        }
    }

    const std::size_t minimum_curve_count =
        std::max<std::size_t>(3, options.minimum_curve_sample_count);
    std::size_t run_first = 0;
    while (run_first < curvature_graph.size()) {
        const HorizontalSampleProperty property =
            curvature_graph[run_first].property;
        std::size_t run_last = run_first + 1;
        while (run_last < curvature_graph.size() &&
               curvature_graph[run_last].property == property) {
            ++run_last;
        }
        const std::size_t run_count = run_last - run_first;
        const std::size_t minimum_count =
            property == HorizontalSampleProperty::straight
                ? 2
                : minimum_curve_count;
        if (run_count < minimum_count) {
            return error_result(
                StatusCode::invalid_input,
                DiagnosticCode::invalid_curvature_graph_shape,
                run_first,
                "Curvature graph contains a line/curve run with too few samples");
        }
        run_first = run_last;
    }
    if (curvature_graph.front().property != HorizontalSampleProperty::straight ||
        curvature_graph.back().property != HorizontalSampleProperty::straight) {
        const std::size_t index =
            curvature_graph.front().property != HorizontalSampleProperty::straight
                ? 0
                : curvature_graph.size() - 1;
        return error_result(
            StatusCode::invalid_input,
            DiagnosticCode::invalid_curvature_graph_shape,
            index,
            "Curvature graph must start and end with straight samples");
    }
    if (curvature_graph.size() > 4 &&
        curvature_graph[curvature_graph.size() - 2].property !=
            HorizontalSampleProperty::straight) {
        return error_result(
            StatusCode::invalid_input,
            DiagnosticCode::invalid_curvature_graph_shape,
            curvature_graph.size() - 2,
            "Curvature graph must preserve the straight route-end classification");
    }
    return {};
}

} // namespace

ReconstructionResult reconstruct(
    const std::vector<Point3D>& ordered_points,
    const ReconstructionOptions& options,
    const std::vector<HorizontalCurvatureSample>& curvature_graph) {
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
        options.minimum_vertical_curve_grade_difference_per_mille < 0.0 ||
        options.minimum_vertical_curve_radius_m <= 0.0 ||
        options.maximum_vertical_curve_radius_m < options.minimum_vertical_curve_radius_m ||
        options.maximum_vertical_curve_radius_m > 30000.0 ||
        options.vertical_curve_radius_rounding_m < 0.0 ||
        options.grade_rounding_per_mille < 0.0 ||
        options.station_rounding_m < 0.0 ||
        options.station_monotonicity_tolerance_m < 0.0 ||
        options.horizontal_curve_radius_rounding_m < 0.0 ||
        options.horizontal_spiral_length_rounding_m < 0.0) {
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
    const ReconstructionResult graph_validation =
        validate_curvature_graph(ordered_points, options, curvature_graph);
    if (!graph_validation.ok()) {
        return graph_validation;
    }

    ReconstructionResult result;
    try {
        const detail::HorizontalReconstructor horizontal_reconstructor(options);
        auto horizontal_reconstruction =
            horizontal_reconstructor.reconstruct_with_curvature_graph(
                horizontal_samples, curvature_graph);
        result.horizontal = std::move(horizontal_reconstruction.controls);
        result.curvature_graph =
            std::move(horizontal_reconstruction.curvature_graph);
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
        std::vector<detail::HorizontalSpiralInterval> horizontal_spirals;
        for (const auto& element : horizontal.elements()) {
            if ((element.kind() == HorizontalElementKind::entry_spiral ||
                 element.kind() == HorizontalElementKind::exit_spiral) &&
                element.length_m() > 0.001) {
                horizontal_spirals.push_back({
                    element.start_station_m(), element.end_station_m()
                });
            }
        }
        const detail::VerticalReconstructor vertical_reconstructor(
            options, std::move(horizontal_spirals));
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
