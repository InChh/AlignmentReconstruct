#include "horizontal/curvature_classifier.h"

#include "geometry/vector_math.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace alignment_reconstruct::detail {
namespace {

std::vector<ClassificationRun> form_runs(const std::vector<bool>& curved) {
    std::vector<ClassificationRun> runs;
    if (curved.empty()) {
        return runs;
    }
    std::size_t first = 0;
    for (std::size_t index = 1; index <= curved.size(); ++index) {
        if (index == curved.size() || curved[index] != curved[first]) {
            runs.push_back({curved[first], first, index - 1});
            first = index;
        }
    }
    return runs;
}

double run_mean_curvature(
    const ClassificationRun& run,
    const std::vector<double>& curvature) {
    double sum = 0.0;
    for (std::size_t index = run.first_index; index <= run.last_index; ++index) {
        sum += curvature[index];
    }
    return sum / static_cast<double>(run.last_index - run.first_index + 1);
}

double three_point_tangent(
    const Point2D& before,
    const Point2D& point,
    const Point2D& after) {
    const double incoming = heading(before, point);
    const double outgoing = heading(point, after);
    const double deflection = normalize_signed_angle(outgoing - incoming);
    if (std::abs(deflection) < 0.01) {
        return normalize_heading(incoming + 0.5 * deflection);
    }

    const double ax = point.x - before.x;
    const double ay = point.y - before.y;
    const double bx = after.x - before.x;
    const double by = after.y - before.y;
    const double determinant = 2.0 * (ax * by - ay * bx);
    if (std::abs(determinant) <= 1.0e-12) {
        return normalize_heading(incoming + 0.5 * deflection);
    }
    const double a_squared = ax * ax + ay * ay;
    const double b_squared = bx * bx + by * by;
    const Point2D center{
        before.x + (by * a_squared - ay * b_squared) / determinant,
        before.y + (ax * b_squared - bx * a_squared) / determinant
    };
    const double radial = heading(center, point);
    return normalize_heading(radial + (deflection > 0.0 ? 0.5 * pi : -0.5 * pi));
}

} // namespace

HorizontalSampleAnalysis classify_horizontal_samples(
    const std::vector<Point2D>& ordered_points,
    const ReconstructionOptions& options) {
    if (ordered_points.size() < 2) {
        throw std::invalid_argument("At least two ordered horizontal samples are required");
    }
    HorizontalSampleAnalysis analysis;
    const std::size_t count = ordered_points.size();
    analysis.cumulative_distance_m.assign(count, 0.0);
    analysis.tangent_heading_rad.assign(count, 0.0);
    analysis.curvature_inverse_m.assign(count, 0.0);
    analysis.curved.assign(count, false);
    for (std::size_t index = 0; index < count; ++index) {
        if (!std::isfinite(ordered_points[index].x) || !std::isfinite(ordered_points[index].y)) {
            throw std::invalid_argument("Horizontal sample contains non-finite data");
        }
        if (index > 0) {
            const double segment = distance(ordered_points[index - 1], ordered_points[index]);
            if (segment <= 1.0e-9) {
                throw std::invalid_argument("Consecutive horizontal samples coincide");
            }
            analysis.cumulative_distance_m[index] =
                analysis.cumulative_distance_m[index - 1] + segment;
        }
    }
    if (count < 4 ||
        analysis.cumulative_distance_m.back() <= 2.0 * options.curvature_window_m) {
        const double route_heading = heading(ordered_points.front(), ordered_points.back());
        std::fill(analysis.tangent_heading_rad.begin(),
                  analysis.tangent_heading_rad.end(), route_heading);
        analysis.runs = {{false, 0, count - 1}};
        return analysis;
    }

    std::size_t first_supported = 0;
    while (first_supported < count &&
           analysis.cumulative_distance_m[first_supported] <= options.curvature_window_m) {
        ++first_supported;
    }
    std::size_t last_supported = count - 1;
    while (last_supported > 0 &&
           analysis.cumulative_distance_m.back() -
               analysis.cumulative_distance_m[last_supported] <= options.curvature_window_m) {
        --last_supported;
    }
    if (first_supported >= last_supported) {
        throw std::invalid_argument("Horizontal samples do not span the curvature window");
    }

    for (std::size_t index = first_supported; index <= last_supported; ++index) {
        std::size_t before = index - 1;
        while (before > 0 && analysis.cumulative_distance_m[index] -
                analysis.cumulative_distance_m[before] <= options.curvature_window_m) {
            --before;
        }
        std::size_t after = index + 1;
        while (after + 1 < count && analysis.cumulative_distance_m[after] -
                analysis.cumulative_distance_m[index] <= options.curvature_window_m) {
            ++after;
        }
        analysis.tangent_heading_rad[index] = three_point_tangent(
            ordered_points[before], ordered_points[index], ordered_points[after]);
    }
    std::fill(analysis.tangent_heading_rad.begin(),
              analysis.tangent_heading_rad.begin() + static_cast<std::ptrdiff_t>(first_supported),
              analysis.tangent_heading_rad[first_supported]);
    std::fill(analysis.tangent_heading_rad.begin() +
                  static_cast<std::ptrdiff_t>(last_supported + 1),
              analysis.tangent_heading_rad.end(),
              analysis.tangent_heading_rad[last_supported]);

    for (std::size_t index = first_supported + 1; index < last_supported; ++index) {
        std::size_t before = index - 1;
        std::size_t after = index + 1;
        if (analysis.cumulative_distance_m[after] -
                analysis.cumulative_distance_m[before] <= options.curvature_window_m) {
            const double half_window = 0.5 * options.curvature_window_m;
            while (before > 0 && analysis.cumulative_distance_m[index] -
                    analysis.cumulative_distance_m[before] <= half_window) {
                --before;
            }
            while (after + 1 < count && analysis.cumulative_distance_m[after] -
                    analysis.cumulative_distance_m[index] <= half_window) {
                ++after;
            }
        }
        const double heading_change = normalize_signed_angle(
            analysis.tangent_heading_rad[after] - analysis.tangent_heading_rad[before]);
        analysis.curvature_inverse_m[index] = heading_change /
            (analysis.cumulative_distance_m[after] -
             analysis.cumulative_distance_m[before]);
    }
    std::fill(analysis.curvature_inverse_m.begin(),
              analysis.curvature_inverse_m.begin() +
                  static_cast<std::ptrdiff_t>(first_supported + 1),
              analysis.curvature_inverse_m[first_supported + 1]);
    std::fill(analysis.curvature_inverse_m.begin() +
                  static_cast<std::ptrdiff_t>(last_supported),
              analysis.curvature_inverse_m.end(),
              analysis.curvature_inverse_m[last_supported - 1]);

    for (std::size_t index = 0; index < count; ++index) {
        analysis.curved[index] =
            std::abs(analysis.curvature_inverse_m[index]) >=
            options.curvature_threshold_inverse_m;
    }

    bool changed = true;
    std::size_t classification_iteration = 0;
    while (changed) {
        if (++classification_iteration > options.maximum_spiral_classification_iterations) {
            throw std::invalid_argument("Horizontal classification did not stabilize");
        }
        changed = false;
        auto runs = form_runs(analysis.curved);
        for (const auto& run : runs) {
            const std::size_t sample_count = run.last_index - run.first_index + 1;
            if (run.curved && sample_count < options.minimum_curve_sample_count) {
                std::fill(analysis.curved.begin() + static_cast<std::ptrdiff_t>(run.first_index),
                          analysis.curved.begin() + static_cast<std::ptrdiff_t>(run.last_index + 1),
                          false);
                changed = true;
            }
        }
        if (changed) {
            continue;
        }
        runs = form_runs(analysis.curved);
        const std::size_t maximum_island =
            std::max<std::size_t>(1, options.minimum_curve_sample_count / 2);
        for (std::size_t run_index = 1; run_index + 1 < runs.size(); ++run_index) {
            const auto& run = runs[run_index];
            const std::size_t sample_count = run.last_index - run.first_index + 1;
            if (!run.curved && sample_count <= maximum_island &&
                runs[run_index - 1].curved && runs[run_index + 1].curved &&
                run_mean_curvature(runs[run_index - 1], analysis.curvature_inverse_m) *
                    run_mean_curvature(runs[run_index + 1], analysis.curvature_inverse_m) > 0.0) {
                std::fill(analysis.curved.begin() + static_cast<std::ptrdiff_t>(run.first_index),
                          analysis.curved.begin() + static_cast<std::ptrdiff_t>(run.last_index + 1),
                          true);
                changed = true;
            }
        }
    }
    analysis.runs = form_runs(analysis.curved);
    return analysis;
}

} // namespace alignment_reconstruct::detail
