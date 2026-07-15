#include "horizontal/alignment_optimizer.h"

#include "geometry/vector_math.h"
#include "horizontal/horizontal_alignment.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace alignment_reconstruct::detail {
namespace {

struct CircleFit {
    Point2D center{};
    double radius_m = 0.0;
    bool valid = false;
};

CircleFit fit_circle(const std::vector<Point2D>& points, const std::vector<std::size_t>& indices) {
    CircleFit result;
    if (indices.size() < 3) {
        return result;
    }
    for (const std::size_t index : indices) {
        result.center = add(result.center, points[index]);
    }
    result.center = scale(result.center, 1.0 / static_cast<double>(indices.size()));

    double xx = 0.0;
    double yy = 0.0;
    double xy = 0.0;
    double x_radius_squared = 0.0;
    double y_radius_squared = 0.0;
    double radius_squared_sum = 0.0;
    for (const std::size_t index : indices) {
        const Point2D delta = subtract(points[index], result.center);
        const double radius_squared = dot(delta, delta);
        xx += delta.x * delta.x;
        yy += delta.y * delta.y;
        xy += delta.x * delta.y;
        x_radius_squared += delta.x * radius_squared;
        y_radius_squared += delta.y * radius_squared;
        radius_squared_sum += radius_squared;
    }
    const double determinant = xx * yy - xy * xy;
    if (std::abs(determinant) <= 1.0e-18) {
        return result;
    }
    const Point2D local_center{
        (yy * x_radius_squared - xy * y_radius_squared) / (2.0 * determinant),
        (xx * y_radius_squared - xy * x_radius_squared) / (2.0 * determinant)
    };
    result.center = add(result.center, local_center);
    result.radius_m = std::sqrt(
        dot(local_center, local_center) +
        radius_squared_sum / static_cast<double>(indices.size()));
    result.valid = std::isfinite(result.radius_m) && result.radius_m > 0.0;
    return result;
}

double distance_to_line(const Point2D& point, const LineFitResult& line) {
    return std::abs(cross(line.unit_direction, subtract(point, line.centroid)));
}

std::vector<std::size_t> select_circular_samples(
    const std::vector<double>& curvature,
    std::size_t first,
    std::size_t last) {
    std::vector<std::pair<double, std::size_t>> ranked;
    for (std::size_t index = first; index <= last; ++index) {
        ranked.emplace_back(std::abs(curvature[index]), index);
    }
    std::sort(ranked.begin(), ranked.end(), [](const auto& left, const auto& right) {
        return left.first > right.first;
    });
    const double reference = ranked[ranked.size() / 4].first;
    const double threshold = 0.65 * reference;
    std::vector<std::size_t> selected;
    for (const auto& item : ranked) {
        if (item.first > threshold) {
            selected.push_back(item.second);
        }
    }
    if (selected.size() < 3) {
        selected.clear();
        for (std::size_t index = first; index <= last; ++index) {
            selected.push_back(index);
        }
    }
    return selected;
}

using Parameters = std::array<double, 3>;

Parameters sanitize(Parameters value) {
    value[0] = std::max(0.0, std::min(1000.0, value[0]));
    value[1] = std::max(1.0, std::min(1.0e7, value[1]));
    value[2] = std::max(0.0, std::min(1000.0, value[2]));
    return value;
}

class CurveObjective {
public:
    CurveObjective(
        const std::vector<Point2D>& points,
        std::size_t first,
        std::size_t last,
        const LineFitResult& incoming,
        const LineFitResult& outgoing,
        Point2D intersection)
        : points_(points), first_(first), last_(last), incoming_(incoming),
          outgoing_(outgoing), intersection_(intersection) {
        extent_m_ = 1000.0;
        for (std::size_t index = first_; index <= last_; ++index) {
            extent_m_ = std::max(extent_m_, distance(points_[index], intersection_) + 100.0);
        }
    }

    double operator()(const Parameters& raw) const {
        const Parameters value = sanitize(raw);
        const double incoming_heading = std::atan2(
            incoming_.unit_direction.y, incoming_.unit_direction.x);
        const double outgoing_heading = std::atan2(
            outgoing_.unit_direction.y, outgoing_.unit_direction.x);
        const double deflection = std::abs(normalize_signed_angle(
            outgoing_heading - incoming_heading));
        if (value[0] + value[2] >= 1.98 * value[1] * deflection) {
            return 1.0e30;
        }
        const Point2D start = subtract(
            intersection_, scale(incoming_.unit_direction, extent_m_));
        const Point2D end = add(
            intersection_, scale(outgoing_.unit_direction, extent_m_));
        const std::vector<HorizontalControlPoint> controls{
            {HorizontalControlPointKind::start, start, 0.0, 0.0, 0.0},
            {HorizontalControlPointKind::intersection, intersection_, value[1], value[0], value[2]},
            {HorizontalControlPointKind::end, end, 0.0, 0.0, 0.0}
        };
        try {
            const HorizontalAlignment alignment(controls, 0.0);
            double sum = 0.0;
            for (std::size_t index = first_; index <= last_; ++index) {
                const auto projection = alignment.project(points_[index]);
                if (!projection.valid) {
                    return 1.0e30;
                }
                sum += projection.distance_m * projection.distance_m;
            }
            return sum / static_cast<double>(last_ - first_ + 1);
        } catch (const std::invalid_argument&) {
            return 1.0e30;
        }
    }

private:
    const std::vector<Point2D>& points_;
    std::size_t first_;
    std::size_t last_;
    const LineFitResult& incoming_;
    const LineFitResult& outgoing_;
    Point2D intersection_;
    double extent_m_ = 0.0;
};

struct Vertex {
    Parameters parameters{};
    double objective = std::numeric_limits<double>::infinity();
};

Parameters nelder_mead(
    const CurveObjective& objective,
    Parameters initial,
    const ReconstructionOptions& options,
    double& final_objective) {
    initial = sanitize(initial);
    std::array<Vertex, 4> simplex;
    simplex[0].parameters = initial;
    simplex[1].parameters = sanitize({
        initial[0] + std::max(5.0, 0.1 * initial[0]), initial[1], initial[2]});
    simplex[2].parameters = sanitize({
        initial[0], initial[1] + std::max(5.0, 0.02 * initial[1]), initial[2]});
    simplex[3].parameters = sanitize({
        initial[0], initial[1], initial[2] + std::max(5.0, 0.1 * initial[2])});
    for (auto& vertex : simplex) {
        vertex.objective = objective(vertex.parameters);
    }

    const std::size_t maximum_iterations =
        std::max<std::size_t>(20, options.horizontal_optimization_max_iterations);
    for (std::size_t iteration = 0; iteration < maximum_iterations; ++iteration) {
        std::sort(simplex.begin(), simplex.end(), [](const Vertex& left, const Vertex& right) {
            return left.objective < right.objective;
        });
        double maximum_normalized_spread = 0.0;
        for (std::size_t vertex = 1; vertex < simplex.size(); ++vertex) {
            for (std::size_t dimension = 0; dimension < 3; ++dimension) {
                maximum_normalized_spread = std::max(
                    maximum_normalized_spread,
                    std::abs(simplex[vertex].parameters[dimension] -
                             simplex[0].parameters[dimension]) /
                        std::max(1.0, std::abs(simplex[0].parameters[dimension])));
            }
        }
        if (maximum_normalized_spread <= options.horizontal_optimization_tolerance) {
            break;
        }

        Parameters centroid{};
        for (std::size_t vertex = 0; vertex < 3; ++vertex) {
            for (std::size_t dimension = 0; dimension < 3; ++dimension) {
                centroid[dimension] += simplex[vertex].parameters[dimension] / 3.0;
            }
        }
        auto combine = [&](double factor) {
            Parameters result{};
            for (std::size_t dimension = 0; dimension < 3; ++dimension) {
                result[dimension] = centroid[dimension] + factor *
                    (centroid[dimension] - simplex[3].parameters[dimension]);
            }
            return sanitize(result);
        };

        Vertex reflected{combine(1.0), 0.0};
        reflected.objective = objective(reflected.parameters);
        if (reflected.objective < simplex[0].objective) {
            Vertex expanded{combine(2.0), 0.0};
            expanded.objective = objective(expanded.parameters);
            simplex[3] = expanded.objective < reflected.objective ? expanded : reflected;
        } else if (reflected.objective < simplex[2].objective) {
            simplex[3] = reflected;
        } else {
            const double contraction_factor =
                reflected.objective < simplex[3].objective ? 0.5 : -0.5;
            Vertex contracted{combine(contraction_factor), 0.0};
            contracted.objective = objective(contracted.parameters);
            if (contracted.objective < simplex[3].objective) {
                simplex[3] = contracted;
            } else {
                for (std::size_t vertex = 1; vertex < simplex.size(); ++vertex) {
                    for (std::size_t dimension = 0; dimension < 3; ++dimension) {
                        simplex[vertex].parameters[dimension] =
                            0.5 * (simplex[0].parameters[dimension] +
                                   simplex[vertex].parameters[dimension]);
                    }
                    simplex[vertex].parameters = sanitize(simplex[vertex].parameters);
                    simplex[vertex].objective = objective(simplex[vertex].parameters);
                }
            }
        }
    }
    std::sort(simplex.begin(), simplex.end(), [](const Vertex& left, const Vertex& right) {
        return left.objective < right.objective;
    });
    final_objective = simplex.front().objective;
    return simplex.front().parameters;
}

} // namespace

HorizontalCurveParameters fit_horizontal_curve(
    const std::vector<Point2D>& points,
    const std::vector<double>& curvature_inverse_m,
    std::size_t curve_first_index,
    std::size_t curve_last_index,
    std::size_t objective_first_index,
    std::size_t objective_last_index,
    const LineFitResult& incoming_line,
    const LineFitResult& outgoing_line,
    const Point2D& intersection,
    const ReconstructionOptions& options) {
    const auto circular_indices = select_circular_samples(
        curvature_inverse_m, curve_first_index, curve_last_index);
    CircleFit circle = fit_circle(points, circular_indices);
    if (!circle.valid) {
        throw std::invalid_argument("Circular sample fit is degenerate");
    }
    const double incoming_distance = distance_to_line(circle.center, incoming_line);
    const double outgoing_distance = distance_to_line(circle.center, outgoing_line);
    if (incoming_distance - circle.radius_m < 1.0e-6) {
        circle.radius_m = std::min(circle.radius_m, incoming_distance);
    }
    if (outgoing_distance - circle.radius_m < 1.0e-6) {
        circle.radius_m = std::min(circle.radius_m, outgoing_distance);
    }
    const double entry_shift = std::max(0.0, incoming_distance - circle.radius_m);
    const double exit_shift = std::max(0.0, outgoing_distance - circle.radius_m);
    Parameters initial{
        std::sqrt(24.0 * circle.radius_m * entry_shift),
        circle.radius_m,
        std::sqrt(24.0 * circle.radius_m * exit_shift)
    };
    if (initial[0] > 1000.0) {
        initial[0] = 0.0;
    }
    if (initial[2] > 1000.0) {
        initial[2] = 0.0;
    }
    const CurveObjective objective(
        points, objective_first_index, objective_last_index,
        incoming_line, outgoing_line, intersection);
    double final_objective = 0.0;
    const Parameters optimized = nelder_mead(objective, initial, options, final_objective);
    return {optimized[1], optimized[0], optimized[2], final_objective};
}

} // namespace alignment_reconstruct::detail
