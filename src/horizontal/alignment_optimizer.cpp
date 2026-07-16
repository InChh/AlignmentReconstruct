#include "horizontal/alignment_optimizer.h"

#include "geometry/circle_fit.h"
#include "geometry/vector_math.h"
#include "horizontal/legacy_horizontal_alignment.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace alignment_reconstruct::detail {
namespace {

using Parameters = std::array<double, 3>;
using Directions = std::array<Parameters, 3>;

double legacy_distance(const Point2D& first, const Point2D& second) {
    const double delta_x = first.x - second.x;
    const double delta_y = first.y - second.y;
    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

double distance_to_line(
    const Point2D& point,
    const Point2D& line_start,
    const Point2D& line_end) {
    constexpr double precision = 1.0e-6;
    Point2D foot{};
    if (std::abs(line_start.y - line_end.y) < precision) {
        foot = {point.x, 0.5 * (line_start.y + line_end.y)};
    } else if (std::abs(line_start.x - line_end.x) < precision) {
        foot = {0.5 * (line_start.x + line_end.x), point.y};
    } else {
        const double slope = (line_end.y - line_start.y) /
            (line_end.x - line_start.x);
        const double intercept = line_start.y - line_start.x * slope;
        foot.x = (point.y + point.x / slope - intercept) /
            (slope + 1.0 / slope);
        foot.y = slope * (point.y + point.x / slope - intercept) /
            (slope + 1.0 / slope) + intercept;
    }
    return legacy_distance(foot, point);
}

class CurveObjective {
public:
    CurveObjective(
        const std::vector<Point2D>& points,
        std::size_t first,
        std::size_t last,
        Point2D incoming_start,
        Point2D incoming_end,
        Point2D outgoing_start,
        Point2D outgoing_end,
        Point2D intersection)
        : points_(points), first_(first), last_(last),
          incoming_start_(incoming_start), incoming_end_(incoming_end),
          outgoing_start_(outgoing_start), outgoing_end_(outgoing_end),
          intersection_(intersection) {}

    double operator()(const Parameters& value) const {
        if (value[0] < 0.0 || value[1] < 0.0 || value[2] < 0.0) {
            return 1.0e6;
        }

        const Point2D incoming_direction = subtract(incoming_end_, incoming_start_);
        const Point2D outgoing_direction = subtract(outgoing_end_, outgoing_start_);
        const Point2D start = subtract(
            incoming_start_, scale(incoming_direction, 10000.0));
        const Point2D end = add(
            outgoing_end_, scale(outgoing_direction, 10000.0));
        try {
            const LegacyHorizontalAlignment alignment(
                start, intersection_, end, value[1], value[0], value[2]);
            double squared_offset_sum = 0.0;
            for (std::size_t index = first_; index <= last_; ++index) {
                const LegacyHorizontalProjection projection =
                    alignment.project(points_[index]);
                if (!projection.valid) {
                    return 1.0e6;
                }
                squared_offset_sum += projection.distance_m * projection.distance_m;
            }
            return squared_offset_sum;
        } catch (const std::invalid_argument&) {
            return 1.0e6;
        }
    }

private:
    const std::vector<Point2D>& points_;
    std::size_t first_;
    std::size_t last_;
    Point2D incoming_start_{};
    Point2D incoming_end_{};
    Point2D outgoing_start_{};
    Point2D outgoing_end_{};
    Point2D intersection_{};
};

double sign_of(double value) {
    if (value > 0.0) {
        return 1.0;
    }
    if (value < 0.0) {
        return -1.0;
    }
    return 0.0;
}

template <typename Function>
void bracket_minimum(
    double& ax,
    double& bx,
    double& cx,
    double& fa,
    double& fb,
    double& fc,
    const Function& function) {
    constexpr double golden_ratio = 1.618034;
    constexpr double growth_limit = 100.0;
    constexpr double tiny = 1.0e-20;

    fa = function(ax);
    fb = function(bx);
    if (fb > fa) {
        std::swap(ax, bx);
        std::swap(fa, fb);
    }
    cx = bx + golden_ratio * (bx - ax);
    fc = function(cx);

    int iteration = 0;
    while (fb > fc) {
        const double r = (bx - ax) * (fb - fc);
        const double q = (bx - cx) * (fb - fa);
        double denominator = q - r;
        if (std::abs(denominator) < tiny) {
            denominator = tiny;
        }
        double u = bx - ((bx - cx) * q - (bx - ax) * r) /
            (2.0 * denominator);
        const double upper_limit = bx + growth_limit * (cx - bx);
        double fu = 0.0;
        if ((bx - u) * (u - cx) > 0.0) {
            fu = function(u);
            if (fu < fc) {
                ax = bx;
                fa = fb;
                bx = u;
                fb = fu;
                return;
            }
            if (fu > fb) {
                cx = u;
                fc = fu;
                return;
            }
            u = cx + golden_ratio * (cx - bx);
            fu = function(u);
        } else if ((cx - u) * (u - upper_limit) > 0.0) {
            fu = function(u);
            if (fu < fc) {
                bx = cx;
                cx = u;
                u = cx + golden_ratio * (cx - bx);
                fb = fc;
                fc = fu;
                fu = function(u);
            }
        } else if ((u - upper_limit) * (upper_limit - cx) >= 0.0) {
            u = upper_limit;
            fu = function(u);
        } else {
            u = cx + golden_ratio * (cx - bx);
            fu = function(u);
        }
        ax = bx;
        bx = cx;
        cx = u;
        fa = fb;
        fb = fc;
        fc = fu;
        if (++iteration > 10) {
            break;
        }
    }
}

template <typename Function>
double brent_minimum(
    double ax,
    double bx,
    double cx,
    double tolerance,
    double& minimum_x,
    const Function& function) {
    constexpr int maximum_iterations = 100;
    constexpr double complementary_golden_ratio = 0.381966;
    constexpr double epsilon = 1.0e-10;

    double lower = std::min(ax, cx);
    double upper = std::max(ax, cx);
    double v = bx;
    double w = bx;
    double x = bx;
    double e = 0.0;
    double d = 0.0;
    double fx = function(x);
    double fv = fx;
    double fw = fx;
    for (int iteration = 1; iteration <= maximum_iterations; ++iteration) {
        const double midpoint = 0.5 * (lower + upper);
        const double tolerance1 = tolerance * std::abs(x) + epsilon;
        const double tolerance2 = 2.0 * tolerance1;
        if (std::abs(x - midpoint) <=
            tolerance2 - 0.5 * (upper - lower)) {
            break;
        }

        bool use_golden_step = true;
        if (std::abs(e) > tolerance1) {
            const double r = (x - w) * (fx - fv);
            double q = (x - v) * (fx - fw);
            double p = (x - v) * q - (x - w) * r;
            q = 2.0 * (q - r);
            if (q > 0.0) {
                p = -p;
            }
            q = std::abs(q);
            const double previous_e = e;
            e = d;
            if (std::abs(p) < std::abs(0.5 * q * previous_e) &&
                p > q * (lower - x) && p < q * (upper - x)) {
                d = p / q;
                const double trial = x + d;
                if (trial - lower < tolerance2 || upper - trial < tolerance2) {
                    d = std::abs(tolerance1) * sign_of(midpoint - x);
                }
                use_golden_step = false;
            }
        }
        if (use_golden_step) {
            e = x >= midpoint ? lower - x : upper - x;
            d = complementary_golden_ratio * e;
        }

        const double u = std::abs(d) >= tolerance1
            ? x + d
            : x + std::abs(tolerance1) * sign_of(d);
        const double fu = function(u);
        if (fu <= fx) {
            if (u >= x) {
                lower = x;
            } else {
                upper = x;
            }
            v = w;
            fv = fw;
            w = x;
            fw = fx;
            x = u;
            fx = fu;
        } else {
            if (u < x) {
                lower = u;
            } else {
                upper = u;
            }
            if (fu <= fw || w == x) {
                v = w;
                fv = fw;
                w = u;
                fw = fu;
            } else if (fu <= fv || v == x || v == w) {
                v = u;
                fv = fu;
            }
        }
    }
    minimum_x = x;
    return fx;
}

void line_minimize(
    Parameters& parameters,
    Parameters& direction,
    const CurveObjective& objective,
    double& objective_value) {
    const Parameters origin = parameters;
    const Parameters search_direction = direction;
    const auto one_dimensional = [&](double multiplier) {
        Parameters candidate{};
        for (std::size_t dimension = 0; dimension < candidate.size(); ++dimension) {
            candidate[dimension] = origin[dimension] +
                multiplier * search_direction[dimension];
        }
        return objective(candidate);
    };

    double ax = 0.0;
    double xx = 1.0;
    double bx = 0.0;
    double fa = 0.0;
    double fx = 0.0;
    double fb = 0.0;
    bracket_minimum(ax, xx, bx, fa, fx, fb, one_dimensional);
    double minimum_x = 0.0;
    objective_value = brent_minimum(
        ax, xx, bx, 0.0001, minimum_x, one_dimensional);
    for (std::size_t dimension = 0; dimension < parameters.size(); ++dimension) {
        direction[dimension] = minimum_x * search_direction[dimension];
        parameters[dimension] += direction[dimension];
    }
}

Parameters powell_minimize(
    const CurveObjective& objective,
    Parameters parameters,
    const ReconstructionOptions& options,
    double& final_objective) {
    Directions directions{{
        Parameters{{1.0, 0.0, 0.0}},
        Parameters{{0.0, 1.0, 0.0}},
        Parameters{{0.0, 0.0, 1.0}}
    }};
    Parameters previous_parameters = parameters;
    final_objective = objective(parameters);
    const std::size_t maximum_iterations =
        options.horizontal_optimization_max_iterations;

    for (std::size_t iteration = 1; iteration <= maximum_iterations; ++iteration) {
        const double previous_objective = final_objective;
        std::size_t largest_decrease_direction = 0;
        double largest_decrease = 0.0;
        for (std::size_t direction_index = 0;
             direction_index < directions.size(); ++direction_index) {
            Parameters direction = directions[direction_index];
            const double before_line_search = final_objective;
            line_minimize(parameters, direction, objective, final_objective);
            if (std::abs(before_line_search - final_objective) > largest_decrease) {
                largest_decrease = std::abs(before_line_search - final_objective);
                largest_decrease_direction = direction_index;
            }
        }
        if (2.0 * std::abs(previous_objective - final_objective) <=
            options.horizontal_optimization_tolerance *
                (std::abs(previous_objective) + std::abs(final_objective))) {
            return parameters;
        }
        if (iteration == maximum_iterations) {
            return parameters;
        }

        Parameters extrapolated{};
        Parameters displacement{};
        for (std::size_t dimension = 0; dimension < parameters.size(); ++dimension) {
            extrapolated[dimension] =
                2.0 * parameters[dimension] - previous_parameters[dimension];
            displacement[dimension] =
                parameters[dimension] - previous_parameters[dimension];
            previous_parameters[dimension] = parameters[dimension];
        }
        const double extrapolated_objective = objective(extrapolated);
        if (extrapolated_objective >= previous_objective) {
            continue;
        }
        const double curvature =
            previous_objective - 2.0 * final_objective + extrapolated_objective;
        const double test =
            2.0 * curvature *
                std::pow(previous_objective - final_objective - largest_decrease, 2.0) -
            largest_decrease *
                std::pow(previous_objective - extrapolated_objective, 2.0);
        if (test >= 0.0) {
            continue;
        }
        line_minimize(parameters, displacement, objective, final_objective);
        directions[largest_decrease_direction] = displacement;
    }
    return parameters;
}

} // namespace

HorizontalCurveParameters fit_horizontal_curve(
    const std::vector<Point2D>& points,
    std::size_t curve_first_index,
    std::size_t curve_last_index,
    std::size_t objective_first_index,
    std::size_t objective_last_index,
    const LineFitResult& incoming_line,
    const LineFitResult& outgoing_line,
    const Point2D& incoming_start,
    const Point2D& incoming_end,
    const Point2D& outgoing_start,
    const Point2D& outgoing_end,
    const Point2D& intersection,
    const ReconstructionOptions& options) {
    (void)incoming_line;
    (void)outgoing_line;
    const CircleFitResult circle = fit_circle_to_range(
        points, curve_first_index, curve_last_index);
    if (!circle.valid) {
        throw std::invalid_argument("Circular sample fit is degenerate");
    }

    const double incoming_distance = distance_to_line(
        circle.center, incoming_start, incoming_end);
    const double outgoing_distance = distance_to_line(
        circle.center, outgoing_start, outgoing_end);
    const double incoming_radius = incoming_distance - circle.radius_m < 1.0e-6
        ? incoming_distance
        : circle.radius_m;
    const double outgoing_radius = outgoing_distance - circle.radius_m < 1.0e-6
        ? outgoing_distance
        : circle.radius_m;
    const double initial_radius = std::min(incoming_radius, outgoing_radius);
    const double entry_shift = incoming_distance - initial_radius;
    const double exit_shift = outgoing_distance - initial_radius;
    const Parameters initial{{
        std::sqrt(24.0 * initial_radius * entry_shift),
        initial_radius,
        std::sqrt(24.0 * initial_radius * exit_shift)
    }};

    const CurveObjective objective(
        points, objective_first_index, objective_last_index,
        incoming_start, incoming_end, outgoing_start, outgoing_end,
        intersection);
    double final_objective = std::numeric_limits<double>::infinity();
    const Parameters optimized = powell_minimize(
        objective, initial, options, final_objective);
    return {optimized[1], optimized[0], optimized[2], final_objective};
}

} // namespace alignment_reconstruct::detail
