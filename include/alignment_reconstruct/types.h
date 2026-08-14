#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace alignment_reconstruct {

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

enum class HorizontalControlPointKind {
    start,
    intersection,
    end
};

enum class HorizontalElementKind {
    straight,
    entry_spiral,
    circular,
    exit_spiral
};

enum class HorizontalSampleProperty {
    straight = 0,
    curve = 1
};

struct HorizontalCurvatureSample {
    double curvature_inverse_m = 0.0;
    HorizontalSampleProperty property = HorizontalSampleProperty::straight;
};

struct HorizontalControlPoint {
    HorizontalControlPointKind kind = HorizontalControlPointKind::intersection;
    Point2D position{};
    double radius_m = 0.0;
    double entry_spiral_length_m = 0.0;
    double exit_spiral_length_m = 0.0;
};

struct VerticalControlPoint {
    double station_m = 0.0;
    double elevation_m = 0.0;
    double vertical_curve_radius_m = 0.0;
};

struct AlignmentDefinition {
    double start_station_m = 0.0;
    std::vector<HorizontalControlPoint> horizontal;
    std::vector<VerticalControlPoint> vertical;
};

enum class StatusCode {
    success,
    invalid_input,
    infeasible_geometry,
    numerical_failure
};

enum class DiagnosticSeverity {
    warning,
    error
};

enum class DiagnosticCode {
    none,
    too_few_points,
    non_finite_value,
    coincident_consecutive_points,
    non_monotonic_station,
    too_few_horizontal_control_points,
    invalid_horizontal_radius,
    invalid_spiral_length,
    infeasible_horizontal_curve,
    too_few_vertical_control_points,
    vertical_range_mismatch,
    invalid_vertical_radius,
    overlapping_vertical_curves,
    degenerate_fit,
    projection_failed,
    invalid_sample_interval,
    insufficient_spatial_support,
    curvature_graph_size_mismatch,
    invalid_horizontal_sample_property,
    invalid_curvature_graph_shape
};

inline constexpr std::size_t no_input_index = std::numeric_limits<std::size_t>::max();

struct Diagnostic {
    DiagnosticSeverity severity = DiagnosticSeverity::error;
    DiagnosticCode code = DiagnosticCode::none;
    std::size_t input_index = no_input_index;
    std::string message;
};

struct ReconstructionOptions {
    double start_station_m = 0.0;
    double curvature_window_m = 15.0;
    double curvature_threshold_inverse_m = 0.001;
    std::size_t minimum_curve_sample_count = 3;
    std::size_t maximum_spiral_classification_iterations = 300;
    double horizontal_optimization_tolerance = 1.0e-4;
    std::size_t horizontal_optimization_max_iterations = 10;
    std::vector<double> vertical_second_difference_threshold_candidates{
        0.001
    };
    double minimum_grade_length_m = 350.0;
    double maximum_up_grade = 25.0;
    double maximum_down_grade = 25.0;
    double maximum_grade_difference_per_mille = 7.0;
    double minimum_vertical_curve_grade_difference_per_mille = 3.0;
    double minimum_vertical_curve_radius_m = 10000.0;
    double maximum_vertical_curve_radius_m = 30000.0;
    double vertical_curve_radius_rounding_m = 100.0;
    double grade_rounding_per_mille = 0.01;
    double station_rounding_m = 10.0;
    double station_monotonicity_tolerance_m = 1.0e-6;
    double horizontal_curve_radius_rounding_m = 0.0;
    double horizontal_spiral_length_rounding_m = 0.0;
    bool equalize_horizontal_spiral_lengths = false;
};

struct ReconstructionResult {
    StatusCode status = StatusCode::success;
    std::vector<HorizontalControlPoint> horizontal;
    std::vector<VerticalControlPoint> vertical;
    std::vector<double> projected_stations_m;
    std::vector<HorizontalCurvatureSample> curvature_graph;
    std::vector<Diagnostic> diagnostics;

    bool ok() const noexcept {
        return status == StatusCode::success;
    }
};

struct PointCloudFitResult {
    StatusCode status = StatusCode::success;
    std::vector<Point3D> centerline_points;
    std::vector<Diagnostic> diagnostics;

    bool ok() const noexcept {
        return status == StatusCode::success;
    }
};

struct PointAdjustment {
    std::size_t point_index = 0;
    double station_m = 0.0;
    Point2D projected_point{};
    double design_elevation_m = 0.0;
    double horizontal_offset_m = 0.0;
    double lift_drop_m = 0.0;
    HorizontalElementKind horizontal_element = HorizontalElementKind::straight;

    double horizontal_offset_mm() const noexcept {
        return horizontal_offset_m * 1000.0;
    }

    double lift_drop_mm() const noexcept {
        return lift_drop_m * 1000.0;
    }
};

struct AdjustmentCalculationResult {
    StatusCode status = StatusCode::success;
    std::vector<PointAdjustment> points;
    std::vector<Diagnostic> diagnostics;

    bool ok() const noexcept {
        return status == StatusCode::success;
    }
};

} // namespace alignment_reconstruct
