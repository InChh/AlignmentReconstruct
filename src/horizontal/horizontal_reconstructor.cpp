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

} // namespace

HorizontalReconstructor::HorizontalReconstructor(ReconstructionOptions options)
    : options_(std::move(options)) {}

std::vector<HorizontalControlPoint> HorizontalReconstructor::reconstruct(
    const std::vector<Point2D>& ordered_points) const {
    const auto analysis = classify_horizontal_samples(ordered_points, options_);
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
        const auto parameters = fit_horizontal_curve(
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
            options_);
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

    const HorizontalAlignment closure_check(controls, options_.start_station_m);
    (void)closure_check;
    return controls;
}

} // namespace alignment_reconstruct::detail
