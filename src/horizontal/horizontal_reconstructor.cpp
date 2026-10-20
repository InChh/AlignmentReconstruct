#include "horizontal/horizontal_reconstructor.h"

#include "geometry/legacy_line_fit.h"
#include "geometry/vector_math.h"
#include "horizontal/alignment_optimizer.h"
#include "horizontal/curvature_classifier.h"
#include "horizontal/horizontal_alignment.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace alignment_reconstruct::detail {
namespace {

struct FittedStraightRun {
    HorizontalStraightSegment segment;
    LegacyLineFitResult fit;
};

std::vector<Point2D> copy_range(
    const std::vector<Point2D>& points,
    std::size_t first,
    std::size_t last) {
    return std::vector<Point2D>(
        points.begin() + static_cast<std::ptrdiff_t>(first),
        points.begin() + static_cast<std::ptrdiff_t>(last + 1));
}

double round_horizontal_parameter(double value_m, double rounding_m) {
    if (!(rounding_m > 0.0)) {
        return value_m;
    }
    const double whole = std::trunc(value_m / rounding_m);
    const double remainder = value_m - whole * rounding_m;
    // The active legacy Digit helper leaves an exact half step on the lower multiple.
    const double increment = remainder > rounding_m * 0.5 ? 1.0 : 0.0;
    return (whole + increment) * rounding_m;
}

HorizontalReconstruction reconstruct_analyzed_horizontal_samples(
    const std::vector<Point2D>& ordered_points,
    const HorizontalSampleAnalysis& analysis,
    const ReconstructionOptions& options) {
    std::vector<FittedStraightRun> straight_runs;
    for (const auto& segment : analysis.straight_segments) {
        const auto points = copy_range(
            ordered_points, segment.first_index, segment.last_index);
        const auto fit = fit_legacy_reconstruction_line(points);
        if (!fit.valid) {
            throw std::invalid_argument("Fitted horizontal tangent is degenerate");
        }
        straight_runs.push_back({segment, fit});
    }
    if (straight_runs.empty()) {
        throw std::invalid_argument("Reconstruction requires straight samples at both route ends");
    }
    const std::size_t curve_count = straight_runs.size() - 1;

    std::vector<Point2D> intersections;
    intersections.reserve(curve_count);
    for (std::size_t index = 0; index < curve_count; ++index) {
        Point2D intersection{};
        if (!intersect_legacy_lines(
                straight_runs[index].fit,
                straight_runs[index + 1].fit,
                intersection)) {
            throw std::invalid_argument("Adjacent fitted tangents are parallel");
        }
        intersections.push_back(intersection);
    }

    std::vector<HorizontalControlPoint> controls;
    controls.reserve(curve_count + 2);
    controls.push_back({
        HorizontalControlPointKind::start,
        project_to_legacy_line(ordered_points.front(), straight_runs.front().fit),
        0.0, 0.0, 0.0
    });
    for (std::size_t index = 0; index < curve_count; ++index) {
        const auto& incoming_segment = straight_runs[index].segment;
        const auto& outgoing_segment = straight_runs[index + 1].segment;
        const auto parameters = apply_horizontal_output_options(
            fit_horizontal_curve(
                ordered_points,
                incoming_segment.circular_first_index,
                incoming_segment.circular_last_index,
                incoming_segment.first_index,
                outgoing_segment.last_index,
                straight_runs[index].fit.line,
                straight_runs[index + 1].fit.line,
                straight_runs[index].fit.start,
                straight_runs[index].fit.end,
                straight_runs[index + 1].fit.start,
                straight_runs[index + 1].fit.end,
                intersections[index],
                options),
            options);
        controls.push_back({
            HorizontalControlPointKind::intersection,
            intersections[index],
            parameters.radius_m,
            parameters.entry_spiral_length_m,
            parameters.exit_spiral_length_m
        });
    }
    controls.push_back({
        HorizontalControlPointKind::end,
        project_to_legacy_line(ordered_points.back(), straight_runs.back().fit),
        0.0, 0.0, 0.0
    });

    const HorizontalAlignment closure_check(controls, options.start_station_m);
    (void)closure_check;
    return {std::move(controls), analysis.curvature_graph};
}

} // namespace

HorizontalCurveParameters apply_horizontal_output_options(
    HorizontalCurveParameters parameters,
    const ReconstructionOptions& options) {
    if (options.equalize_horizontal_spiral_lengths) {
        const double equal_length_m =
            (parameters.entry_spiral_length_m + parameters.exit_spiral_length_m) * 0.5;
        parameters.entry_spiral_length_m = equal_length_m;
        parameters.exit_spiral_length_m = equal_length_m;
    }
    parameters.radius_m = round_horizontal_parameter(
        parameters.radius_m, options.horizontal_curve_radius_rounding_m);
    parameters.entry_spiral_length_m = round_horizontal_parameter(
        parameters.entry_spiral_length_m,
        options.horizontal_spiral_length_rounding_m);
    parameters.exit_spiral_length_m = round_horizontal_parameter(
        parameters.exit_spiral_length_m,
        options.horizontal_spiral_length_rounding_m);
    return parameters;
}

HorizontalReconstructor::HorizontalReconstructor(ReconstructionOptions options)
    : options_(std::move(options)) {}

std::vector<HorizontalControlPoint> HorizontalReconstructor::reconstruct(
    const std::vector<Point2D>& ordered_points) const {
    return reconstruct_with_curvature_graph(ordered_points).controls;
}

HorizontalReconstruction HorizontalReconstructor::reconstruct_with_curvature_graph(
    const std::vector<Point2D>& ordered_points,
    const std::vector<HorizontalCurvatureSample>& curvature_graph) const {
    const auto analysis = classify_horizontal_samples(
        ordered_points, options_, curvature_graph);
    const bool missing_tangent_support = curvature_graph.empty() &&
        std::any_of(analysis.straight_segments.begin(), analysis.straight_segments.end(),
            [](const HorizontalStraightSegment& segment) {
                return segment.first_index == segment.last_index;
            });
    if (!analysis.curves_erased_by_dynamic && !missing_tangent_support) {
        return reconstruct_analyzed_horizontal_samples(ordered_points, analysis, options_);
    }

    // Noisy curvature can erase all curves or split one with a singleton
    // tangent that cannot be fitted. Retry bounded wider support in the same
    // classifier, and adopt only a complete fitted scheme with valid geometry.
    // Explicit caller-supplied properties never enter this recovery.
    for (const double window_scale : {2.0, 3.0}) {
        ReconstructionOptions refined_options = options_;
        refined_options.curvature_window_m = window_scale * options_.curvature_window_m;
        if (!has_horizontal_curvature_support(
                analysis.cumulative_distance_m, refined_options.curvature_window_m)) {
            continue;
        }
        try {
            const auto refined = classify_horizontal_samples(
                ordered_points, refined_options);
            if (std::any_of(refined.curved.begin(), refined.curved.end(),
                            [](bool curved) { return curved; })) {
                return reconstruct_analyzed_horizontal_samples(
                    ordered_points, refined, options_);
            }
        } catch (const std::invalid_argument&) {
            // Wider support is an internal candidate. Its classification,
            // fitting or closure failure does not invalidate the original.
            continue;
        }
    }
    return reconstruct_analyzed_horizontal_samples(ordered_points, analysis, options_);
}

} // namespace alignment_reconstruct::detail
