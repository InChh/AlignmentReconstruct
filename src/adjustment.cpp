#include "alignment_reconstruct/adjustment.h"

#include "alignment/adjustment_calculator.h"
#include "horizontal/horizontal_alignment.h"
#include "vertical/vertical_alignment.h"

#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>

namespace alignment_reconstruct {
namespace {

AdjustmentCalculationResult error_result(
    StatusCode status,
    DiagnosticCode code,
    std::size_t input_index,
    std::string message) {
    AdjustmentCalculationResult result;
    result.status = status;
    result.diagnostics.push_back({
        DiagnosticSeverity::error, code, input_index, std::move(message)
    });
    return result;
}

AdjustmentCalculationResult validate_definition(const AlignmentDefinition& alignment) {
    if (!std::isfinite(alignment.start_station_m)) {
        return error_result(StatusCode::invalid_input, DiagnosticCode::non_finite_value,
                            no_input_index, "Alignment start station is non-finite");
    }
    if (alignment.horizontal.size() < 2) {
        return error_result(StatusCode::invalid_input,
                            DiagnosticCode::too_few_horizontal_control_points,
                            no_input_index, "At least two horizontal control points are required");
    }
    for (std::size_t index = 0; index < alignment.horizontal.size(); ++index) {
        const auto& point = alignment.horizontal[index];
        if (!std::isfinite(point.position.x) || !std::isfinite(point.position.y) ||
            !std::isfinite(point.radius_m) ||
            !std::isfinite(point.entry_spiral_length_m) ||
            !std::isfinite(point.exit_spiral_length_m)) {
            return error_result(StatusCode::invalid_input, DiagnosticCode::non_finite_value,
                                index, "Horizontal control point contains non-finite data");
        }
        const bool endpoint = index == 0 || index + 1 == alignment.horizontal.size();
        if (index > 0 && point.position.x == alignment.horizontal[index - 1].position.x &&
            point.position.y == alignment.horizontal[index - 1].position.y) {
            return error_result(StatusCode::invalid_input,
                                DiagnosticCode::coincident_consecutive_points,
                                index, "Consecutive horizontal control points coincide");
        }
        if (endpoint && point.radius_m != 0.0) {
            return error_result(StatusCode::invalid_input,
                                DiagnosticCode::invalid_horizontal_radius,
                                index, "Horizontal start and end radii must be zero");
        }
        if (!endpoint && point.radius_m <= 0.0) {
            return error_result(StatusCode::invalid_input,
                                DiagnosticCode::invalid_horizontal_radius,
                                index, "Interior horizontal radius must be positive");
        }
        if (point.entry_spiral_length_m < 0.0 || point.exit_spiral_length_m < 0.0) {
            return error_result(StatusCode::invalid_input, DiagnosticCode::invalid_spiral_length,
                                index, "Horizontal spiral lengths cannot be negative");
        }
        if (endpoint &&
            (point.entry_spiral_length_m != 0.0 || point.exit_spiral_length_m != 0.0)) {
            return error_result(StatusCode::invalid_input, DiagnosticCode::invalid_spiral_length,
                                index, "Horizontal start and end spiral lengths must be zero");
        }
    }
    if (alignment.vertical.size() < 2) {
        return error_result(StatusCode::invalid_input,
                            DiagnosticCode::too_few_vertical_control_points,
                            no_input_index, "At least two vertical control points are required");
    }
    for (std::size_t index = 0; index < alignment.vertical.size(); ++index) {
        const auto& point = alignment.vertical[index];
        if (!std::isfinite(point.station_m) || !std::isfinite(point.elevation_m) ||
            !std::isfinite(point.vertical_curve_radius_m)) {
            return error_result(StatusCode::invalid_input, DiagnosticCode::non_finite_value,
                                index, "Vertical control point contains non-finite data");
        }
        if (index > 0 && point.station_m <= alignment.vertical[index - 1].station_m) {
            return error_result(StatusCode::invalid_input, DiagnosticCode::non_monotonic_station,
                                index, "Vertical stations must be strictly increasing");
        }
        if (point.vertical_curve_radius_m < 0.0 ||
            (point.vertical_curve_radius_m > 0.0 && point.vertical_curve_radius_m < 100.0)) {
            return error_result(StatusCode::invalid_input, DiagnosticCode::invalid_vertical_radius,
                                index, "Vertical radius must be zero or a physical radius >= 100 m");
        }
        if ((index == 0 || index + 1 == alignment.vertical.size()) &&
            point.vertical_curve_radius_m != 0.0) {
            return error_result(StatusCode::invalid_input, DiagnosticCode::invalid_vertical_radius,
                                index, "Vertical start and end radii must be zero");
        }
    }
    double previous_curve_end = -std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index + 1 < alignment.vertical.size(); ++index) {
        const double radius = alignment.vertical[index].vertical_curve_radius_m;
        if (radius == 0.0) {
            continue;
        }
        const double incoming_grade =
            (alignment.vertical[index].elevation_m - alignment.vertical[index - 1].elevation_m) /
            (alignment.vertical[index].station_m - alignment.vertical[index - 1].station_m);
        const double outgoing_grade =
            (alignment.vertical[index + 1].elevation_m - alignment.vertical[index].elevation_m) /
            (alignment.vertical[index + 1].station_m - alignment.vertical[index].station_m);
        const double incoming_angle = std::atan(incoming_grade);
        const double outgoing_angle = std::atan(outgoing_grade);
        const double tangent = radius *
            std::tan(0.5 * std::abs(outgoing_angle - incoming_angle));
        const double curve_start = alignment.vertical[index].station_m -
            tangent * std::cos(incoming_angle);
        const double curve_end = alignment.vertical[index].station_m +
            tangent * std::cos(outgoing_angle);
        if (curve_start < alignment.vertical[index - 1].station_m ||
            curve_end > alignment.vertical[index + 1].station_m ||
            curve_start < previous_curve_end) {
            return error_result(StatusCode::infeasible_geometry,
                                DiagnosticCode::overlapping_vertical_curves,
                                index, "Vertical curves overlap or exceed adjacent grade segments");
        }
        previous_curve_end = curve_end;
    }
    return {};
}

} // namespace

AdjustmentCalculationResult calculate_adjustments(
    const std::vector<Point3D>& points,
    const AlignmentDefinition& alignment) {
    auto validation = validate_definition(alignment);
    if (!validation.ok()) {
        return validation;
    }
    try {
        const detail::HorizontalAlignment horizontal(
            alignment.horizontal, alignment.start_station_m);
        const detail::VerticalAlignment vertical(alignment.vertical);
        constexpr double range_tolerance_m = 1.0e-6;
        if (vertical.start_station_m() > horizontal.start_station_m() + range_tolerance_m ||
            vertical.end_station_m() < horizontal.end_station_m() - range_tolerance_m) {
            return error_result(
                StatusCode::invalid_input,
                DiagnosticCode::vertical_range_mismatch,
                no_input_index,
                "Vertical alignment must cover the complete horizontal station range");
        }
        const detail::AlignmentAdjustmentCalculator calculator(horizontal, vertical);
        return calculator.calculate(points);
    } catch (const std::invalid_argument& error) {
        return error_result(StatusCode::infeasible_geometry,
                            DiagnosticCode::infeasible_horizontal_curve,
                            no_input_index, error.what());
    } catch (const std::exception& error) {
        return error_result(StatusCode::numerical_failure, DiagnosticCode::projection_failed,
                            no_input_index, error.what());
    }
}

} // namespace alignment_reconstruct
