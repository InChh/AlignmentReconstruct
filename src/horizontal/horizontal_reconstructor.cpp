#include "horizontal/horizontal_reconstructor.h"

#include "geometry/line_fit.h"
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
    ClassificationRun run;
    LineFitResult line;
};

Point2D project_to_line(const Point2D& point, const LineFitResult& line) {
    return add(line.centroid, scale(
        line.unit_direction, dot(subtract(point, line.centroid), line.unit_direction)));
}

Point2D intersect_lines(const LineFitResult& first, const LineFitResult& second) {
    const double denominator = cross(first.unit_direction, second.unit_direction);
    if (std::abs(denominator) <= 1.0e-10) {
        throw std::invalid_argument("Adjacent fitted tangents are parallel");
    }
    const double parameter = cross(
        subtract(second.centroid, first.centroid), second.unit_direction) / denominator;
    return add(first.centroid, scale(first.unit_direction, parameter));
}

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
    std::vector<ClassificationRun> curve_runs;
    for (const auto& run : analysis.runs) {
        if (run.curved) {
            curve_runs.push_back(run);
        } else {
            const auto points = copy_range(ordered_points, run.first_index, run.last_index);
            const auto fit = fit_orthogonal_line(points);
            if (!fit.valid) {
                throw std::invalid_argument("Fitted horizontal tangent is degenerate");
            }
            straight_runs.push_back({run, fit});
        }
    }
    if (straight_runs.empty() || analysis.runs.front().curved || analysis.runs.back().curved) {
        throw std::invalid_argument("Reconstruction requires straight samples at both route ends");
    }
    if (straight_runs.size() != curve_runs.size() + 1) {
        throw std::invalid_argument("Horizontal straight/curve segmentation is inconsistent");
    }

    std::vector<Point2D> intersections;
    intersections.reserve(curve_runs.size());
    for (std::size_t index = 0; index < curve_runs.size(); ++index) {
        intersections.push_back(intersect_lines(
            straight_runs[index].line, straight_runs[index + 1].line));
    }

    std::vector<HorizontalControlPoint> controls;
    controls.reserve(curve_runs.size() + 2);
    controls.push_back({
        HorizontalControlPointKind::start,
        project_to_line(ordered_points.front(), straight_runs.front().line),
        0.0, 0.0, 0.0
    });
    for (std::size_t index = 0; index < curve_runs.size(); ++index) {
        const auto& incoming_run = straight_runs[index].run;
        const auto& outgoing_run = straight_runs[index + 1].run;
        const std::size_t objective_first = incoming_run.last_index > 5
            ? std::max(incoming_run.first_index, incoming_run.last_index - 5)
            : incoming_run.first_index;
        const std::size_t objective_last = std::min(
            outgoing_run.last_index, outgoing_run.first_index + 5);
        const auto parameters = fit_horizontal_curve(
            ordered_points,
            analysis.curvature_inverse_m,
            curve_runs[index].first_index,
            curve_runs[index].last_index,
            objective_first,
            objective_last,
            straight_runs[index].line,
            straight_runs[index + 1].line,
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
        project_to_line(ordered_points.back(), straight_runs.back().line),
        0.0, 0.0, 0.0
    });

    const HorizontalAlignment closure_check(controls, options_.start_station_m);
    (void)closure_check;
    return controls;
}

} // namespace alignment_reconstruct::detail
