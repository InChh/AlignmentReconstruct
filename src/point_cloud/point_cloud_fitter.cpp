#include "point_cloud/point_cloud_fitter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace alignment_reconstruct::detail {
namespace {

constexpr double bin_width_m = 1.0;
constexpr long double bin_boundary_tolerance = 1.0e-12L;
constexpr double plan_half_window_m = 12.0;
constexpr double height_half_window_m = 6.0;
constexpr std::size_t robust_iterations = 3;
constexpr std::size_t minimum_anchor_count = 20;
constexpr std::size_t minimum_points_per_bin = 3;
constexpr std::size_t minimum_fit_anchor_count = 3;
constexpr std::size_t local_minimum_neighbors = 9;
// One nominal tail observation at the 1% boundary needs at least 100 samples.
constexpr std::size_t minimum_quantile_neighbors = 100;
constexpr std::size_t maximum_output_point_count = 10'000'000;
constexpr double endpoint_remainder_tolerance_m = 0.01;
constexpr long double minimum_plan_variance_ratio = 4.0L;
constexpr double axis_snap_sine = 0.10452846326765347; // sin(6 degrees)
constexpr double section_relative_tolerance = 1.0e-4;

struct LocalPointCloud {
    Point3D origin;
    std::vector<Point3D> points;
};

struct AnchorSet {
    std::vector<Point3D> centers;
    std::vector<std::size_t> counts;
};

struct RobustFit {
    std::vector<double> fitted;
    std::vector<double> robust_weights;
};

struct PlanBasis {
    double longitudinal_x = 1.0;
    double longitudinal_y = 0.0;
    double transverse_x = 0.0;
    double transverse_y = 1.0;
};

bool point_less(const Point3D& left, const Point3D& right) {
    if (left.x != right.x) {
        return left.x < right.x;
    }
    if (left.y != right.y) {
        return left.y < right.y;
    }
    return left.z < right.z;
}

[[noreturn]] void fail(
    StatusCode status,
    DiagnosticCode diagnostic_code,
    const std::string& message) {
    throw PointCloudFitError(status, diagnostic_code, message);
}

double median(std::vector<double> values) {
    if (values.empty()) {
        fail(
            StatusCode::numerical_failure,
            DiagnosticCode::degenerate_fit,
            "Cannot calculate a median from an empty point set");
    }

    const std::size_t middle = values.size() / 2;
    std::nth_element(
        values.begin(),
        values.begin() + static_cast<std::ptrdiff_t>(middle),
        values.end());
    const double upper = values[middle];
    if ((values.size() % 2U) != 0U) {
        return upper;
    }

    const double lower = *std::max_element(
        values.begin(),
        values.begin() + static_cast<std::ptrdiff_t>(middle));
    return lower + (upper - lower) * 0.5;
}

double quantile(std::vector<double> values, double probability) {
    std::sort(values.begin(), values.end());
    const double position = probability * static_cast<double>(values.size() - 1);
    const auto lower = static_cast<std::size_t>(position);
    const auto upper = std::min(lower + 1, values.size() - 1);
    return values[lower] + (position - static_cast<double>(lower)) *
        (values[upper] - values[lower]);
}

Point3D coordinate_median(const std::vector<Point3D>& points) {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    x.reserve(points.size());
    y.reserve(points.size());
    z.reserve(points.size());
    for (const auto& point : points) {
        x.push_back(point.x);
        y.push_back(point.y);
        z.push_back(point.z);
    }
    return {median(std::move(x)), median(std::move(y)), median(std::move(z))};
}

double checked_difference(double value, double origin) {
    const long double difference =
        static_cast<long double>(value) - static_cast<long double>(origin);
    const long double maximum =
        static_cast<long double>(std::numeric_limits<double>::max());
    if (difference < -maximum || difference > maximum) {
        fail(
            StatusCode::numerical_failure,
            DiagnosticCode::degenerate_fit,
            "Point-cloud coordinate range exceeds supported floating-point precision");
    }
    return static_cast<double>(difference);
}

LocalPointCloud center_points(const std::vector<Point3D>& points) {
    LocalPointCloud local;
    local.origin = coordinate_median(points);
    local.points.reserve(points.size());
    for (const auto& point : points) {
        local.points.push_back({
            checked_difference(point.x, local.origin.x),
            checked_difference(point.y, local.origin.y),
            checked_difference(point.z, local.origin.z)
        });
    }
    return local;
}

PlanBasis estimate_plan_basis(const std::vector<Point3D>& points) {
    std::vector<Point3D> ordered = points;
    std::sort(
        ordered.begin(),
        ordered.end(),
        point_less);

    long double mean_x = 0.0L;
    long double mean_y = 0.0L;
    for (const Point3D& point : ordered) {
        mean_x += static_cast<long double>(point.x);
        mean_y += static_cast<long double>(point.y);
    }
    const long double point_count =
        static_cast<long double>(ordered.size());
    mean_x /= point_count;
    mean_y /= point_count;

    long double sum_xx = 0.0L;
    long double sum_xy = 0.0L;
    long double sum_yy = 0.0L;
    for (const Point3D& point : ordered) {
        const long double x =
            static_cast<long double>(point.x) - mean_x;
        const long double y =
            static_cast<long double>(point.y) - mean_y;
        sum_xx += x * x;
        sum_xy += x * y;
        sum_yy += y * y;
    }
    if (!std::isfinite(sum_xx) ||
        !std::isfinite(sum_xy) ||
        !std::isfinite(sum_yy)) {
        fail(
            StatusCode::numerical_failure,
            DiagnosticCode::degenerate_fit,
            "Point-cloud plan-direction covariance is not finite");
    }

    const long double trace = sum_xx + sum_yy;
    const long double difference = sum_xx - sum_yy;
    const long double discriminant =
        std::hypot(difference, 2.0L * sum_xy);
    const long double major_variance =
        (trace + discriminant) * 0.5L;
    const long double minor_variance = std::max(
        0.0L, (trace - discriminant) * 0.5L);
    if (!std::isfinite(major_variance) ||
        !std::isfinite(minor_variance) ||
        major_variance <= 0.0L ||
        major_variance < minimum_plan_variance_ratio * minor_variance) {
        fail(
            StatusCode::invalid_input,
            DiagnosticCode::insufficient_spatial_support,
            "Point cloud does not have a stable plan direction");
    }

    const long double angle =
        0.5L * std::atan2(2.0L * sum_xy, difference);
    double longitudinal_x = static_cast<double>(std::cos(angle));
    double longitudinal_y = static_cast<double>(std::sin(angle));
    bool x_dominant =
        std::abs(longitudinal_x) >= std::abs(longitudinal_y);
    if ((x_dominant && longitudinal_x < 0.0) ||
        (!x_dominant && longitudinal_y < 0.0)) {
        longitudinal_x = -longitudinal_x;
        longitudinal_y = -longitudinal_y;
    }

    if (x_dominant &&
        std::abs(longitudinal_y) <= axis_snap_sine) {
        return {1.0, 0.0, 0.0, 1.0};
    }
    if (!x_dominant &&
        std::abs(longitudinal_x) <= axis_snap_sine) {
        return {0.0, 1.0, 1.0, 0.0};
    }

    x_dominant =
        std::abs(longitudinal_x) >= std::abs(longitudinal_y);
    if (x_dominant) {
        return {
            longitudinal_x,
            longitudinal_y,
            -longitudinal_y,
            longitudinal_x
        };
    }
    return {
        longitudinal_x,
        longitudinal_y,
        longitudinal_y,
        -longitudinal_x
    };
}

std::vector<Point3D> transform_to_plan_basis(
    const std::vector<Point3D>& points,
    const PlanBasis& basis) {
    std::vector<Point3D> transformed;
    transformed.reserve(points.size());
    for (const Point3D& point : points) {
        transformed.push_back({
            basis.longitudinal_x * point.x +
                basis.longitudinal_y * point.y,
            basis.transverse_x * point.x +
                basis.transverse_y * point.y,
            point.z
        });
    }
    return transformed;
}

void transform_from_plan_basis(
    std::vector<Point3D>& points,
    const PlanBasis& basis,
    const Point3D& origin) {
    for (Point3D& point : points) {
        const double local_x =
            basis.longitudinal_x * point.x +
            basis.transverse_x * point.y;
        const double local_y =
            basis.longitudinal_y * point.x +
            basis.transverse_y * point.y;
        point.x = local_x + origin.x;
        point.y = local_y + origin.y;
        point.z += origin.z;
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            fail(
                StatusCode::numerical_failure,
                DiagnosticCode::degenerate_fit,
                "Point-cloud centerline output contains a non-finite coordinate");
        }
    }
}

long long bin_identifier(double x, double minimum_x, double width_m) {
    const long double normalized =
        (static_cast<long double>(x) - static_cast<long double>(minimum_x)) /
        static_cast<long double>(width_m);
    const long double raw_identifier =
        std::floor(normalized + bin_boundary_tolerance);
    const long double minimum =
        static_cast<long double>(std::numeric_limits<long long>::min());
    const long double maximum =
        static_cast<long double>(std::numeric_limits<long long>::max());
    if (raw_identifier < minimum || raw_identifier > maximum) {
        fail(
            StatusCode::invalid_input,
            DiagnosticCode::insufficient_spatial_support,
            "Point-cloud longitudinal range is too large for spatial bins");
    }
    return static_cast<long long>(raw_identifier);
}

AnchorSet spatial_bin_anchors(
    const std::vector<Point3D>& points,
    bool use_means = false) {
    const auto minimum_iterator = std::min_element(
        points.begin(),
        points.end(),
        [](const Point3D& left, const Point3D& right) {
            return left.x < right.x;
        });
    const double minimum_x = minimum_iterator->x;
    const auto maximum_iterator = std::max_element(
        points.begin(), points.end(),
        [](const Point3D& left, const Point3D& right) { return left.x < right.x; });
    const double range_m = maximum_iterator->x - minimum_x;
    const double width_m = range_m > 0.0
        ? std::min(bin_width_m, range_m / static_cast<double>(minimum_anchor_count))
        : bin_width_m;

    std::map<long long, std::vector<Point3D>> bins;
    for (const auto& point : points) {
        bins[bin_identifier(point.x, minimum_x, width_m)].push_back(point);
    }

    AnchorSet anchors;
    anchors.centers.reserve(bins.size());
    anchors.counts.reserve(bins.size());
    for (const auto& bin : bins) {
        if (use_means) {
            std::vector<Point3D> ordered = bin.second;
            std::sort(ordered.begin(), ordered.end(), point_less);
            long double x = 0.0L;
            long double y = 0.0L;
            long double z = 0.0L;
            for (const auto& point : ordered) {
                x += point.x;
                y += point.y;
                z += point.z;
            }
            const long double count = static_cast<long double>(ordered.size());
            anchors.centers.push_back({static_cast<double>(x / count),
                static_cast<double>(y / count), static_cast<double>(z / count)});
        } else {
            anchors.centers.push_back(coordinate_median(bin.second));
        }
        anchors.counts.push_back(bin.second.size());
    }
    std::stable_sort(
        anchors.centers.begin(),
        anchors.centers.end(),
        [](const Point3D& left, const Point3D& right) {
            return left.x < right.x;
        });
    return anchors;
}

template<std::size_t Order>
std::array<long double, Order> solve_symmetric_system(
    std::array<std::array<long double, Order>, Order> matrix,
    std::array<long double, Order> right_hand_side) {
    long double maximum_coefficient = 0.0L;
    for (const auto& row : matrix) {
        for (const long double value : row) {
            maximum_coefficient = std::max(maximum_coefficient, std::abs(value));
        }
    }
    const long double pivot_tolerance =
        std::numeric_limits<long double>::epsilon() *
        std::max(1.0L, maximum_coefficient) * 64.0L;

    for (std::size_t column = 0; column < Order; ++column) {
        std::size_t pivot_row = column;
        long double pivot_magnitude = std::abs(matrix[column][column]);
        for (std::size_t row = column + 1; row < Order; ++row) {
            const long double candidate = std::abs(matrix[row][column]);
            if (candidate > pivot_magnitude) {
                pivot_magnitude = candidate;
                pivot_row = row;
            }
        }
        if (pivot_magnitude <= pivot_tolerance) {
            fail(
                StatusCode::numerical_failure,
                DiagnosticCode::degenerate_fit,
                "Local quadratic point-cloud fit is numerically degenerate");
        }
        if (pivot_row != column) {
            std::swap(matrix[pivot_row], matrix[column]);
            std::swap(right_hand_side[pivot_row], right_hand_side[column]);
        }

        for (std::size_t row = column + 1; row < Order; ++row) {
            const long double factor =
                matrix[row][column] / matrix[column][column];
            for (std::size_t inner = column; inner < Order; ++inner) {
                matrix[row][inner] -= factor * matrix[column][inner];
            }
            right_hand_side[row] -= factor * right_hand_side[column];
        }
    }

    std::array<long double, Order> solution{};
    for (std::size_t reverse = Order; reverse > 0; --reverse) {
        const std::size_t row = reverse - 1;
        long double value = right_hand_side[row];
        for (std::size_t column = row + 1; column < Order; ++column) {
            value -= matrix[row][column] * solution[column];
        }
        solution[row] = value / matrix[row][row];
    }
    return solution;
}

std::array<long double, 3> local_coefficients(
    const std::vector<double>& x,
    const std::vector<double>& values,
    const std::vector<std::size_t>& selected,
    const std::vector<double>& weights,
    double target,
    double coordinate_scale_m,
    bool quadratic = true) {
    std::array<std::array<long double, 3>, 3> normal{};
    std::array<long double, 3> right_hand_side{};
    for (std::size_t selected_index = 0;
         selected_index < selected.size(); ++selected_index) {
        const std::size_t input_index = selected[selected_index];
        const long double dx =
            (static_cast<long double>(x[input_index]) -
             static_cast<long double>(target)) /
            static_cast<long double>(coordinate_scale_m);
        const std::array<long double, 3> basis{1.0L, dx, quadratic ? dx * dx : 0.0L};
        const long double weight = static_cast<long double>(
            std::max(weights[selected_index], 1.0e-15));
        const long double value = static_cast<long double>(values[input_index]);
        for (std::size_t row = 0; row < 3; ++row) {
            right_hand_side[row] += weight * basis[row] * value;
            for (std::size_t column = 0; column < 3; ++column) {
                normal[row][column] += weight * basis[row] * basis[column];
            }
        }
    }
    if (!quadratic) {
        normal[2][2] = 1.0L;
    }
    return solve_symmetric_system(std::move(normal), std::move(right_hand_side));
}

double local_polynomial_prediction(
    const std::vector<double>& x,
    const std::vector<double>& values,
    double target,
    double half_window_m,
    const std::vector<double>& robust_weights,
    bool adapt_to_support = false,
    std::size_t adaptive_neighbor_count = minimum_fit_anchor_count,
    bool preserve_quadratic_support = false) {
    std::vector<double> distances(x.size());
    std::vector<std::size_t> selected;
    selected.reserve(x.size());
    for (std::size_t index = 0; index < x.size(); ++index) {
        distances[index] = std::abs(x[index] - target);
        if (distances[index] <= half_window_m) {
            selected.push_back(index);
        }
    }

    const std::size_t minimum_neighbors =
        std::min(adapt_to_support ? adaptive_neighbor_count : local_minimum_neighbors,
                 x.size());
    if (selected.size() < minimum_neighbors) {
        selected.resize(x.size());
        std::iota(selected.begin(), selected.end(), std::size_t{0});
        std::stable_sort(
            selected.begin(),
            selected.end(),
            [&distances](std::size_t left, std::size_t right) {
                return distances[left] < distances[right];
            });
        selected.resize(minimum_neighbors);
        std::sort(selected.begin(), selected.end());
    }

    double effective_half_window_m = half_window_m;
    double coordinate_scale_m = 0.0;
    if (adapt_to_support) {
        for (const std::size_t index : selected) {
            coordinate_scale_m = std::max(coordinate_scale_m, distances[index]);
            effective_half_window_m = std::max(
                effective_half_window_m, 1.01 * distances[index]);
        }
        if (coordinate_scale_m == 0.0) {
            fail(StatusCode::invalid_input, DiagnosticCode::insufficient_spatial_support,
                "Local point-cloud fit needs distinct longitudinal positions");
        }
    }

    std::vector<double> distance_weights;
    std::vector<double> weights;
    distance_weights.reserve(selected.size());
    weights.reserve(selected.size());
    std::size_t positive_weight_count = 0;
    for (const std::size_t index : selected) {
        const double scaled =
            std::min(distances[index] / effective_half_window_m, 1.0);
        const double one_minus_cube = 1.0 - scaled * scaled * scaled;
        const double distance_weight =
            one_minus_cube * one_minus_cube * one_minus_cube;
        const double weight = distance_weight * robust_weights[index];
        distance_weights.push_back(distance_weight);
        weights.push_back(weight);
        if (weight > 1.0e-12) {
            ++positive_weight_count;
        }
    }
    if (positive_weight_count < 3) {
        weights = distance_weights;
    }

    std::array<long double, 3> coefficients{};
    try {
        coefficients = local_coefficients(
            x, values, selected, weights, target,
            adapt_to_support ? coordinate_scale_m : 1.0);
    } catch (const PointCloudFitError& error) {
        if (!preserve_quadratic_support ||
            error.status() != StatusCode::numerical_failure ||
            error.diagnostic_code() != DiagnosticCode::degenerate_fit) {
            throw;
        }
        // Three positive robust weights can still have insufficient leverage.
        // Retain the full selected support and the same quadratic model, as in
        // the existing fewer-than-three-positive-weights guard above.
        coefficients = local_coefficients(
            x, values, selected, distance_weights, target,
            adapt_to_support ? coordinate_scale_m : 1.0);
    }
    const double prediction = static_cast<double>(coefficients[0]);
    if (!std::isfinite(prediction)) {
        fail(
            StatusCode::numerical_failure,
            DiagnosticCode::degenerate_fit,
            "Local quadratic point-cloud fit produced a non-finite coordinate");
    }
    return prediction;
}

double band_center_prediction(
    const std::vector<double>& x,
    const std::vector<double>& y,
    double target,
    double half_window_m,
    const std::vector<double>& observation_weights) {
    std::vector<std::size_t> nearest(x.size());
    std::iota(nearest.begin(), nearest.end(), std::size_t{0});
    std::stable_sort(nearest.begin(), nearest.end(), [&](std::size_t left, std::size_t right) {
        return std::abs(x[left] - target) < std::abs(x[right] - target);
    });
    const double distinct_tolerance_m = 1.0e-9 * std::max(1.0, x.back() - x.front());
    std::vector<double> distinct_positions;
    std::size_t neighbor_count = 0;
    double support_radius_m = 0.0;
    for (const std::size_t index : nearest) {
        ++neighbor_count;
        if (std::none_of(distinct_positions.begin(), distinct_positions.end(),
            [&](double position) { return std::abs(position - x[index]) <= distinct_tolerance_m; })) {
            distinct_positions.push_back(x[index]);
        }
        support_radius_m = std::abs(x[index] - target);
        if (distinct_positions.size() >= minimum_fit_anchor_count &&
            neighbor_count >= std::min(minimum_quantile_neighbors, x.size())) {
            break;
        }
    }
    half_window_m = std::max(half_window_m, 1.25 * support_radius_m);
    std::vector<std::size_t> selected;
    for (std::size_t index = 0; index < x.size(); ++index) {
        if (std::abs(x[index] - target) <= half_window_m) {
            selected.push_back(index);
        }
    }
    std::vector<double> weights;
    weights.reserve(selected.size());
    for (const std::size_t index : selected) {
        const double distance = std::abs(x[index] - target) / half_window_m;
        const double complement = 1.0 - distance * distance * distance;
        weights.push_back(complement * complement * complement *
            (observation_weights.empty() ? 1.0 : observation_weights[index]));
    }
    const auto preliminary = local_coefficients(x, y, selected, weights,
        target, half_window_m);
    std::vector<std::pair<double, std::size_t>> residual_order;
    for (std::size_t index = 0; index < selected.size(); ++index) {
        const std::size_t point_index = selected[index];
        const long double dx = (static_cast<long double>(x[point_index]) - target) / half_window_m;
        residual_order.push_back({y[point_index] - static_cast<double>(
            preliminary[0] + dx * (preliminary[1] + dx * preliminary[2])), index});
    }
    std::sort(residual_order.begin(), residual_order.end());
    std::vector<double> residual_values;
    for (const auto& value : residual_order) {
        residual_values.push_back(value.first);
    }
    const double lower_width_m = quantile(residual_values, 0.01);
    const double upper_width_m = quantile(residual_values, 0.99);
    // Both boundaries share one center quadratic. Width may change linearly;
    // independent boundary curvatures otherwise bend toward sparse end samples.
    std::array<long double, 5> coefficients{
        preliminary[0] + (lower_width_m + upper_width_m) * 0.5,
        preliminary[1], preliminary[2],
        (upper_width_m - lower_width_m) * 0.5, 0.0L};
    constexpr long double residual_smoothing_squared_m2 = 0.0001L;
    const auto loss = [&](const std::array<long double, 5>& parameters) {
        long double total = 0.0L;
        for (std::size_t index = 0; index < selected.size(); ++index) {
            const std::size_t point_index = selected[index];
            const long double dx =
                (static_cast<long double>(x[point_index]) - target) / half_window_m;
            const long double center_m =
                parameters[0] + dx * (parameters[1] + dx * parameters[2]);
            const long double width_m = parameters[3] + dx * parameters[4];
            for (const int side : {-1, 1}) {
                const long double residual_m =
                    y[point_index] - center_m - side * width_m;
                total += weights[index] * (
                    std::sqrt(residual_m * residual_m + residual_smoothing_squared_m2) +
                    static_cast<long double>(side) * 0.98L * residual_m);
            }
        }
        return total;
    };
    for (std::size_t iteration = 0; iteration < 60; ++iteration) {
        std::array<std::array<long double, 5>, 5> normal{};
        std::array<long double, 5> right_hand_side{};
        for (std::size_t index = 0; index < selected.size(); ++index) {
            const std::size_t point_index = selected[index];
            const long double dx =
                (static_cast<long double>(x[point_index]) - target) / half_window_m;
            for (const int side : {-1, 1}) {
                const std::array<long double, 5> basis{
                    1.0L, dx, dx * dx,
                    static_cast<long double>(side), static_cast<long double>(side) * dx};
                long double prediction_m = 0.0L;
                for (std::size_t coefficient = 0; coefficient < basis.size(); ++coefficient) {
                    prediction_m += basis[coefficient] * coefficients[coefficient];
                }
                const long double residual_m = y[point_index] - prediction_m;
                const long double absolute_m = std::sqrt(
                    residual_m * residual_m + residual_smoothing_squared_m2);
                const long double observation_weight = std::max(weights[index], 1.0e-15);
                const long double curvature_weight = observation_weight *
                    residual_smoothing_squared_m2 / (absolute_m * absolute_m * absolute_m);
                const long double score = observation_weight *
                    (residual_m / absolute_m + static_cast<long double>(side) * 0.98L);
                for (std::size_t row = 0; row < basis.size(); ++row) {
                    right_hand_side[row] += basis[row] * score;
                    for (std::size_t column = 0; column < basis.size(); ++column) {
                        normal[row][column] +=
                            curvature_weight * basis[row] * basis[column];
                    }
                }
            }
        }
        const auto direction = solve_symmetric_system(
            std::move(normal), std::move(right_hand_side));
        long double step_scale = 1.0L;
        const long double old_loss = loss(coefficients);
        std::array<long double, 5> candidate{};
        bool accepted = false;
        for (std::size_t backtrack = 0; backtrack < 40; ++backtrack) {
            for (std::size_t index = 0; index < candidate.size(); ++index) {
                candidate[index] = coefficients[index] + step_scale * direction[index];
            }
            if (loss(candidate) <= old_loss) {
                accepted = true;
                break;
            }
            step_scale *= 0.5L;
        }
        if (!accepted) {
            break;
        }
        long double change = 0.0L;
        for (std::size_t index = 0; index < candidate.size(); ++index) {
            change = std::max(change, std::abs(candidate[index] - coefficients[index]));
        }
        coefficients = candidate;
        if (change < 1.0e-8L) {
            break;
        }
    }
    return static_cast<double>(coefficients[0]);
}

std::vector<double> local_polynomial_predict(
    const std::vector<double>& x,
    const std::vector<double>& values,
    const std::vector<double>& targets,
    double half_window_m,
    const std::vector<double>& robust_weights,
    bool adapt_to_support = false,
    std::size_t adaptive_neighbor_count = minimum_fit_anchor_count,
    bool preserve_quadratic_support = false) {
    std::vector<double> predictions;
    predictions.reserve(targets.size());
    for (const double target : targets) {
        predictions.push_back(local_polynomial_prediction(
            x, values, target, half_window_m, robust_weights, adapt_to_support,
            adaptive_neighbor_count, preserve_quadratic_support));
    }
    return predictions;
}

RobustFit robust_local_quadratic(
    const std::vector<double>& x,
    const std::vector<double>& values,
    double half_window_m,
    bool adapt_to_support = false,
    std::size_t adaptive_neighbor_count = minimum_fit_anchor_count,
    const std::vector<double>& observation_weights = {},
    bool preserve_quadratic_support = false) {
    RobustFit result;
    result.robust_weights.assign(x.size(), 1.0);
    if (!observation_weights.empty()) {
        result.robust_weights = observation_weights;
    }
    result.fitted = values;

    for (std::size_t iteration = 0;
         iteration < robust_iterations;
         ++iteration) {
        result.fitted = local_polynomial_predict(
            x, values, x, half_window_m, result.robust_weights, adapt_to_support,
            adaptive_neighbor_count, preserve_quadratic_support);

        std::vector<double> residuals(values.size());
        for (std::size_t index = 0; index < values.size(); ++index) {
            residuals[index] = values[index] - result.fitted[index];
        }
        const double residual_center = median(residuals);
        std::vector<double> absolute_deviations;
        absolute_deviations.reserve(residuals.size());
        for (const double residual : residuals) {
            absolute_deviations.push_back(
                std::abs(residual - residual_center));
        }
        const double mad = median(std::move(absolute_deviations));
        const double scale = std::max(1.4826 * mad, 1.0e-9);

        for (std::size_t index = 0; index < residuals.size(); ++index) {
            const double normalized =
                (residuals[index] - residual_center) / (6.0 * scale);
            if (std::abs(normalized) < 1.0) {
                const double one_minus_square =
                    1.0 - normalized * normalized;
                result.robust_weights[index] =
                    one_minus_square * one_minus_square;
            } else {
                result.robust_weights[index] = 0.0;
            }
            if (!observation_weights.empty()) {
                result.robust_weights[index] *= observation_weights[index];
            }
        }
    }

    result.fitted = local_polynomial_predict(
        x, values, x, half_window_m, result.robust_weights, adapt_to_support,
        adaptive_neighbor_count, preserve_quadratic_support);
    return result;
}

std::vector<Point3D> build_curve(
    const std::vector<Point3D>& points,
    const AnchorSet& anchors,
    bool adapt_to_support = false,
    double plan_window_m = plan_half_window_m,
    std::size_t adaptive_neighbor_count = minimum_fit_anchor_count,
    bool preserve_quadratic_support = false) {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> observation_weights;
    x.reserve(anchors.centers.size());
    y.reserve(anchors.centers.size());
    z.reserve(anchors.centers.size());
    for (const auto& anchor : anchors.centers) {
        x.push_back(anchor.x);
        y.push_back(anchor.y);
        z.push_back(anchor.z);
    }
    if (adapt_to_support) {
        observation_weights.reserve(anchors.counts.size());
        for (const std::size_t count : anchors.counts) {
            observation_weights.push_back(static_cast<double>(count));
        }
    }

    const RobustFit y_fit =
        robust_local_quadratic(x, y, plan_window_m, adapt_to_support,
            adaptive_neighbor_count, observation_weights, preserve_quadratic_support);
    const RobustFit z_fit =
        robust_local_quadratic(x, z, height_half_window_m, adapt_to_support,
            adaptive_neighbor_count, observation_weights, preserve_quadratic_support);

    const auto minimum_maximum_x = std::minmax_element(
        points.begin(),
        points.end(),
        [](const Point3D& left, const Point3D& right) {
            return left.x < right.x;
        });
    const std::vector<double> endpoint_x{
        minimum_maximum_x.first->x,
        minimum_maximum_x.second->x
    };
    const std::vector<double> endpoint_y = local_polynomial_predict(
        x, y, endpoint_x, plan_window_m, y_fit.robust_weights, adapt_to_support,
        adaptive_neighbor_count, preserve_quadratic_support);
    const std::vector<double> endpoint_z = local_polynomial_predict(
        x, z, endpoint_x, height_half_window_m, z_fit.robust_weights, adapt_to_support,
        adaptive_neighbor_count, preserve_quadratic_support);

    std::vector<Point3D> candidates;
    candidates.reserve(anchors.centers.size() + 2);
    candidates.push_back({endpoint_x[0], endpoint_y[0], endpoint_z[0]});
    for (std::size_t index = 0; index < x.size(); ++index) {
        candidates.push_back({x[index], y_fit.fitted[index], z_fit.fitted[index]});
    }
    candidates.push_back({endpoint_x[1], endpoint_y[1], endpoint_z[1]});

    std::vector<Point3D> curve;
    curve.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        if (curve.empty() ||
            candidate.x - curve.back().x >
                std::numeric_limits<double>::epsilon()) {
            curve.push_back(candidate);
        }
    }
    if (curve.size() < 2) {
        fail(
            StatusCode::invalid_input,
            DiagnosticCode::insufficient_spatial_support,
            "Point-cloud centerline does not contain two distinct longitudinal positions");
    }
    return curve;
}

double section_longitudinal_coordinate(
    const std::vector<Point3D>& ordered,
    const Point3D& point,
    double half_window_m) {
    const auto first = std::lower_bound(
        ordered.begin(), ordered.end(), point.x - half_window_m,
        [](const Point3D& candidate, double value) { return candidate.x < value; });
    const auto last = std::upper_bound(
        first, ordered.end(), point.x + half_window_m,
        [](double value, const Point3D& candidate) { return value < candidate.x; });
    const long double count = static_cast<long double>(last - first);
    long double mean_x = 0.0L;
    long double mean_y = 0.0L;
    for (auto iterator = first; iterator != last; ++iterator) {
        mean_x += iterator->x;
        mean_y += iterator->y;
    }
    mean_x /= count;
    mean_y /= count;
    long double xx = 0.0L;
    long double xy = 0.0L;
    long double yy = 0.0L;
    for (auto iterator = first; iterator != last; ++iterator) {
        const long double dx = static_cast<long double>(iterator->x) - mean_x;
        const long double dy = static_cast<long double>(iterator->y) - mean_y;
        xx += dx * dx;
        xy += dx * dy;
        yy += dy * dy;
    }
    const long double angle = 0.5L * std::atan2(2.0L * xy, xx - yy);
    const long double cosine = std::cos(angle);
    const long double sine = std::sin(angle);
    return static_cast<double>(mean_x + cosine *
        ((static_cast<long double>(point.x) - mean_x) * cosine +
         (static_cast<long double>(point.y) - mean_y) * sine));
}

double interpolate_transverse(
    const std::vector<Point3D>& curve,
    double longitudinal_m,
    double* slope_out = nullptr) {
    auto right = std::upper_bound(
        curve.begin(), curve.end(), longitudinal_m,
        [](double value, const Point3D& point) { return value < point.x; });
    if (right == curve.begin()) {
        ++right;
    } else if (right == curve.end()) {
        --right;
    }
    const Point3D& left = *(right - 1);
    const double slope = (right->y - left.y) / (right->x - left.x);
    if (slope_out != nullptr) {
        *slope_out = slope;
    }
    return left.y + slope * (longitudinal_m - left.x);
}

std::vector<Point3D> build_band_curve(
    const std::vector<Point3D>& ordered,
    double half_window_m,
    const std::vector<double>& observation_weights) {
    const AnchorSet anchors = spatial_bin_anchors(ordered, true);
    if (anchors.centers.size() < minimum_fit_anchor_count) {
        fail(StatusCode::invalid_input, DiagnosticCode::insufficient_spatial_support,
            "At least three distinct longitudinal anchors are required");
    }
    std::vector<double> raw_x;
    std::vector<double> raw_y;
    raw_x.reserve(ordered.size());
    raw_y.reserve(ordered.size());
    for (const auto& point : ordered) {
        raw_x.push_back(point.x);
        raw_y.push_back(point.y);
    }
    double minimum_x_m = raw_x.back();
    double maximum_x_m = raw_x.front();
    // Negligible-influence separated tails still enter the fit but do not
    // extend its supported longitudinal domain.
    for (std::size_t index = 0; index < raw_x.size(); ++index) {
        if (observation_weights[index] >= 0.5) {
            minimum_x_m = std::min(minimum_x_m, raw_x[index]);
            maximum_x_m = std::max(maximum_x_m, raw_x[index]);
        }
    }
    std::vector<double> anchor_x;
    std::vector<double> anchor_z;
    std::vector<double> counts;
    for (std::size_t index = 0; index < anchors.centers.size(); ++index) {
        anchor_x.push_back(anchors.centers[index].x);
        anchor_z.push_back(anchors.centers[index].z);
        counts.push_back(static_cast<double>(anchors.counts[index]));
    }
    // Quantile fits need enough independent sections. Their evaluation grid
    // follows occupied anchors, so a large gap never creates synthetic input.
    const double knot_spacing_m = std::max(bin_width_m, half_window_m / 50.0);
    std::vector<double> targets{minimum_x_m};
    for (const double x : anchor_x) {
        if (x <= maximum_x_m && x - targets.back() >= knot_spacing_m) {
            targets.push_back(x);
        }
    }
    if (maximum_x_m - targets.back() >= knot_spacing_m * 0.01) {
        targets.push_back(maximum_x_m);
    } else {
        targets.back() = maximum_x_m;
    }
    // Keep three model positions even when the occupied support is short.
    if (targets.size() < minimum_fit_anchor_count) {
        targets = anchor_x;
    }
    std::vector<double> center_y;
    center_y.reserve(targets.size());
    for (const double target : targets) {
        center_y.push_back(band_center_prediction(raw_x, raw_y, target, half_window_m, observation_weights));
    }
    const auto y_fit = robust_local_quadratic(targets, center_y, plan_half_window_m,
        true, minimum_fit_anchor_count, {}, true);
    const auto z_fit = robust_local_quadratic(anchor_x, anchor_z, height_half_window_m,
        true, local_minimum_neighbors, counts, true);
    const auto heights = local_polynomial_predict(anchor_x, anchor_z, targets,
        height_half_window_m, z_fit.robust_weights, true, local_minimum_neighbors, true);
    std::vector<Point3D> curve;
    curve.reserve(targets.size());
    for (std::size_t index = 0; index < targets.size(); ++index) {
        curve.push_back({targets[index], y_fit.fitted[index], heights[index]});
    }
    return curve;
}

std::vector<double> separated_tail_weights(const std::vector<double>& residuals) {
    // A separated minority residual population has bounded influence. Compare
    // gaps across the full support rather than clipping by a center-biased MAD,
    // which can erase the valid opposite edge of a densely sampled band.
    std::vector<std::pair<double, std::size_t>> ordered;
    for (std::size_t index = 0; index < residuals.size(); ++index) {
        ordered.push_back({residuals[index], index});
    }
    std::sort(ordered.begin(), ordered.end());
    double largest_gap = 0.0;
    double second_gap = 0.0;
    std::size_t split = 0;
    for (std::size_t index = 1; index < ordered.size(); ++index) {
        const double gap = ordered[index].first - ordered[index - 1].first;
        if (gap > largest_gap) {
            second_gap = largest_gap;
            largest_gap = gap;
            split = index;
        } else {
            second_gap = std::max(second_gap, gap);
        }
    }
    std::vector<double> weights(residuals.size(), 1.0);
    if (largest_gap > 3.0 * second_gap && largest_gap > 1.0e-4 &&
        10 * std::min(split, ordered.size() - split) < ordered.size()) {
        for (std::size_t index = 0; index < ordered.size(); ++index) {
            if ((split <= ordered.size() / 2 && index < split) ||
                (split > ordered.size() / 2 && index >= split)) {
                weights[ordered[index].second] = 1.0e-9;
            }
        }
    }
    return weights;
}

void sort_section_points(
    std::vector<Point3D>& points,
    std::vector<double>& weights) {
    std::vector<std::pair<Point3D, double>> ordered;
    for (std::size_t index = 0; index < points.size(); ++index) {
        ordered.push_back({points[index], weights[index]});
    }
    std::sort(ordered.begin(), ordered.end(), [](const auto& left, const auto& right) {
        return point_less(left.first, right.first);
    });
    for (std::size_t index = 0; index < points.size(); ++index) {
        points[index] = ordered[index].first;
        weights[index] = ordered[index].second;
    }
}

bool has_concentrated_scatter_shape(const std::vector<double>& residuals) {
    const double interquartile = quantile(residuals, 0.75) -
        quantile(residuals, 0.25);
    const double central_span = quantile(residuals, 0.90) -
        quantile(residuals, 0.10);
    const double tail_span = quantile(residuals, 0.99) -
        quantile(residuals, 0.01);
    const double octile_span = quantile(residuals, 0.875) -
        quantile(residuals, 0.125);
    return interquartile > 0.0 && tail_span > 1.5 * central_span &&
        octile_span > 1.55 * interquartile && octile_span < 2.1 * interquartile;
}

bool has_spatially_supported_scatter_shape(
    const std::vector<Point3D>& points,
    const std::vector<double>& residuals) {
    if (!has_concentrated_scatter_shape(residuals)) {
        return false;
    }
    constexpr std::size_t partition_count = 4;
    std::array<std::vector<double>, partition_count> local_residuals;
    const auto extent = std::minmax_element(points.begin(), points.end(),
        [](const Point3D& left, const Point3D& right) { return left.x < right.x; });
    const double range_m = extent.second->x - extent.first->x;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const double fraction = (points[index].x - extent.first->x) / range_m;
        const std::size_t partition = std::min(partition_count - 1,
            static_cast<std::size_t>(fraction * static_cast<double>(partition_count)));
        local_residuals[partition].push_back(residuals[index]);
    }
    std::size_t supported_partitions = 0;
    std::size_t scatter_partitions = 0;
    for (const auto& local : local_residuals) {
        if (local.size() >= 4 * local_minimum_neighbors) {
            ++supported_partitions;
            const double central_span_m = quantile(local, 0.90) - quantile(local, 0.10);
            const double tail_span_m = quantile(local, 0.99) - quantile(local, 0.01);
            if (tail_span_m > 1.5 * central_span_m) {
                ++scatter_partitions;
            }
        }
    }
    // Width changes can produce heavy pooled tails even when each observed
    // cross-section is bounded. Confirm the distribution shape spatially.
    // With too few observations to test local distributions, retain the
    // globally supported population model rather than inventing band edges.
    return supported_partitions < 3 || 2 * scatter_partitions > supported_partitions;
}

std::vector<Point3D> build_sparse_curve(
    const std::vector<Point3D>& points,
    const AnchorSet& anchors) {
    const std::vector<Point3D> initial = build_curve(points, anchors, true);
    std::vector<double> residuals;
    residuals.reserve(points.size());
    for (const Point3D& point : points) {
        residuals.push_back(point.y - interpolate_transverse(initial, point.x));
    }
    const double center = median(residuals);
    const double residual_span_m = quantile(residuals, 0.95) - quantile(residuals, 0.05);
    const auto signed_residuals = residuals;
    for (double& residual : residuals) {
        residual = std::abs(residual - center);
    }
    const double transverse_scale_m = std::max(
        1.4826 * median(residuals), residual_span_m / 3.29);
    const double plan_window_m = std::max(
        plan_half_window_m, 10.0 * transverse_scale_m);
    if (plan_window_m == plan_half_window_m) {
        return initial;
    }

    // A band's geometric center is the midpoint of its two robust boundaries;
    // a population mean follows sampling density and can drift to either side.
    std::vector<Point3D> ordered = points;
    std::sort(ordered.begin(), ordered.end(), point_less);
    std::vector<Point3D> section_points = ordered;
    for (Point3D& point : section_points) {
        // Use the fitted local tangent to remove a section's transverse
        // contribution to the global longitudinal coordinate, including at
        // the endpoints. Z still belongs to that same physical section.
        point.x = section_longitudinal_coordinate(ordered, point,
            std::max(plan_half_window_m, 6.0 * transverse_scale_m));
    }
    if (has_spatially_supported_scatter_shape(points, signed_residuals)) {
        const double scatter_window_m = std::max(plan_half_window_m,
            6.0 * (1.4826 * median(residuals)));
        if (scatter_window_m == plan_half_window_m) {
            return initial;
        }
        section_points = ordered;
        for (auto& point : section_points) {
            point.x = section_longitudinal_coordinate(ordered, point, scatter_window_m);
        }
        const auto section_anchors = spatial_bin_anchors(section_points, true);
        return build_curve(section_points, section_anchors, true, scatter_window_m,
            local_minimum_neighbors);
    }
    const double trend_window_m = std::max(plan_half_window_m, 6.0 * transverse_scale_m);
    const auto trend = build_curve(section_points, spatial_bin_anchors(section_points, true),
        true, trend_window_m, local_minimum_neighbors, true);
    std::vector<double> trend_residuals;
    for (const auto& point : ordered) {
        trend_residuals.push_back(point.y - interpolate_transverse(trend, point.x));
    }
    const auto input_weights = separated_tail_weights(trend_residuals);
    std::vector<double> section_weights = input_weights;
    sort_section_points(section_points, section_weights);
    auto curve = build_band_curve(section_points, plan_window_m, section_weights);
    for (std::size_t iteration = 1; iteration < robust_iterations; ++iteration) {
        std::vector<double> trend_x;
        std::vector<double> trend_y;
        for (const auto& point : curve) {
            trend_x.push_back(point.x);
            trend_y.push_back(point.y);
        }
        // Project against a trend supported over the transverse scale.
        // A last-knot slope amplified by band width can move an endpoint far
        // beyond its observed section even when center predictions look smooth.
        const auto tangent_fit = robust_local_quadratic(trend_x, trend_y,
            std::max(plan_half_window_m, 3.0 * transverse_scale_m), true,
            local_minimum_neighbors, {}, true);
        auto tangent_curve = curve;
        for (std::size_t index = 0; index < tangent_curve.size(); ++index) {
            tangent_curve[index].y = tangent_fit.fitted[index];
        }
        section_points = ordered;
        for (auto& point : section_points) {
            const double source_x = point.x;
            double longitudinal_m = source_x;
            for (std::size_t step = 0; step < 5; ++step) {
                double slope = 0.0;
                const double transverse_m = interpolate_transverse(tangent_curve, longitudinal_m, &slope);
                longitudinal_m += (source_x - longitudinal_m +
                    slope * (point.y - transverse_m)) / (1.0 + slope * slope);
            }
            point.x = longitudinal_m;
        }
        section_weights = input_weights;
        sort_section_points(section_points, section_weights);
        curve = build_band_curve(section_points, plan_window_m, section_weights);
    }
    return curve;
}

struct SpatialSection {
    Point3D base;
    double normal_x = 0.0;
    double normal_y = 1.0;
    double station_m = 0.0;
    std::size_t begin = 0;
    std::size_t end = 0;
};

std::vector<double> section_derivative(
    const std::vector<double>& station,
    const std::vector<double>& values) {
    std::vector<double> derivative(values.size());
    derivative.front() = (values[1] - values[0]) / (station[1] - station[0]);
    derivative.back() = (values.back() - values[values.size() - 2]) /
        (station.back() - station[station.size() - 2]);
    for (std::size_t index = 1; index + 1 < values.size(); ++index) {
        const double before_m = station[index] - station[index - 1];
        const double after_m = station[index + 1] - station[index];
        derivative[index] = -after_m * values[index - 1] /
            (before_m * (before_m + after_m)) +
            (after_m - before_m) * values[index] / (before_m * after_m) +
            before_m * values[index + 1] / (after_m * (before_m + after_m));
    }
    return derivative;
}

bool section_boundaries_support_directions(
    const std::vector<double>& station,
    const std::vector<double>& boundary_midpoints,
    const std::vector<double>& widths) {
    if (station.size() < local_minimum_neighbors) {
        return false;
    }
    // Collinearity alone does not establish a normal section: parallel slices
    // of a bent band are collinear too. A valid direction family must leave no
    // systematic longitudinal drift of the observed two-sided envelope center.
    // Midpoints are evidence only; the output offset still uses all raw tails.
    constexpr std::size_t partition_count = 4;
    const double resolution_m = section_relative_tolerance * median(widths);
    for (std::size_t partition = 0; partition <= partition_count; ++partition) {
        const std::size_t begin = partition == 0 ? 0 :
            (partition - 1) * station.size() / partition_count;
        const std::size_t end = partition == 0 ? station.size() :
            partition * station.size() / partition_count;
        if (end - begin < local_minimum_neighbors) {
            continue;
        }
        std::vector<std::size_t> selected(end - begin);
        std::iota(selected.begin(), selected.end(), begin);
        const std::vector<double> weights(selected.size(), 1.0);
        const double target_m = 0.5 * (station[begin] + station[end - 1]);
        const double scale_m = station[end - 1] - station[begin];
        try {
            const auto coefficients = local_coefficients(station, boundary_midpoints,
                selected, weights, target_m, scale_m);
            std::vector<long double> predicted;
            long double sum_prediction = 0.0L;
            long double residual_squared = 0.0L;
            for (const auto index : selected) {
                const long double dx = (static_cast<long double>(station[index]) - target_m) / scale_m;
                const long double value = coefficients[0] + dx * (coefficients[1] + dx * coefficients[2]);
                predicted.push_back(value);
                sum_prediction += value;
                const long double residual = boundary_midpoints[index] - value;
                residual_squared += residual * residual;
            }
            const long double count = static_cast<long double>(selected.size());
            const long double mean_prediction = sum_prediction / count;
            long double trend_squared = 0.0L;
            for (const auto value : predicted) {
                const long double centered = value - mean_prediction;
                trend_squared += centered * centered;
            }
            // Two nonconstant fitted coefficients: require the observed drift
            // to exceed four residual standard errors jointly (F > 16), with
            // the same spatial resolution used to confirm section geometry.
            if (trend_squared > 32.0L * residual_squared / (count - 3.0L) +
                count * resolution_m * resolution_m) {
                return false;
            }
        } catch (const PointCloudFitError&) {
            return false;
        }
    }
    return true;
}

std::vector<Point3D> recover_section_directions(
    const std::vector<Point3D>& points,
    const std::vector<Point3D>& curve) {
    std::vector<double> residuals;
    for (const auto& point : points) {
        residuals.push_back(point.y - interpolate_transverse(curve, point.x));
    }
    const double center_m = median(residuals);
    const double span_m = quantile(residuals, 0.95) - quantile(residuals, 0.05);
    const bool scatter = has_spatially_supported_scatter_shape(points, residuals);
    for (double& residual : residuals) {
        residual = std::abs(residual - center_m);
    }
    if (10.0 * std::max(1.4826 * median(residuals), span_m / 3.29) <=
        plan_half_window_m || scatter) {
        return {};
    }

    // The existing curve supplies only an approximate spatial ordering. Every
    // observation must then belong to a well-separated, transverse TLS section;
    // row order and a nominal number of points per section carry no information.
    std::vector<Point3D> ordered = points;
    std::sort(ordered.begin(), ordered.end(), point_less);
    std::vector<double> cumulative(curve.size(), 0.0);
    for (std::size_t index = 1; index < curve.size(); ++index) {
        cumulative[index] = cumulative[index - 1] + std::hypot(
            curve[index].x - curve[index - 1].x, curve[index].y - curve[index - 1].y);
    }
    std::vector<std::pair<double, std::size_t>> projected;
    projected.reserve(ordered.size());
    for (std::size_t index = 0; index < ordered.size(); ++index) {
        double best_squared_m2 = std::numeric_limits<double>::infinity();
        double station_m = 0.0;
        for (std::size_t segment = 1; segment < curve.size(); ++segment) {
            const auto& start = curve[segment - 1];
            const double dx = curve[segment].x - start.x;
            const double dy = curve[segment].y - start.y;
            const double fraction = std::clamp(
                ((ordered[index].x - start.x) * dx + (ordered[index].y - start.y) * dy) /
                (dx * dx + dy * dy), 0.0, 1.0);
            const double ex = ordered[index].x - start.x - fraction * dx;
            const double ey = ordered[index].y - start.y - fraction * dy;
            const double squared_m2 = ex * ex + ey * ey;
            if (squared_m2 < best_squared_m2) {
                best_squared_m2 = squared_m2;
                station_m = cumulative[segment - 1] +
                    fraction * (cumulative[segment] - cumulative[segment - 1]);
            }
        }
        projected.push_back({station_m, index});
    }
    std::sort(projected.begin(), projected.end());
    std::vector<double> gaps;
    const double precision_m = 64.0 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, cumulative.back());
    for (std::size_t index = 1; index < projected.size(); ++index) {
        const double gap_m = projected[index].first - projected[index - 1].first;
        if (gap_m > precision_m) {
            gaps.push_back(gap_m);
        }
    }
    if (gaps.size() < 2) {
        return {};
    }
    std::sort(gaps.begin(), gaps.end());
    double separating_gap_m = 0.5 * gaps.front();
    if (gaps.size() > ordered.size() / 3) {
        if (gaps.back() <= 10.0 * median(gaps)) {
            return {};
        }
        double largest_ratio = 1.0;
        separating_gap_m = 0.0;
        for (std::size_t index = gaps.size() / 20; index + 2 < gaps.size(); ++index) {
            const double ratio = gaps[index + 1] / gaps[index];
            if (ratio > largest_ratio) {
                largest_ratio = ratio;
                separating_gap_m = std::sqrt(gaps[index] * gaps[index + 1]);
            }
        }
        if (separating_gap_m == 0.0) {
            return {};
        }
    }

    std::vector<SpatialSection> sections;
    for (std::size_t begin = 0; begin < projected.size();) {
        std::size_t end = begin + 1;
        while (end < projected.size() &&
            projected[end].first - projected[end - 1].first <= separating_gap_m) {
            ++end;
        }
        if (end - begin < minimum_fit_anchor_count) {
            return {};
        }
        SpatialSection section;
        section.begin = begin;
        section.end = end;
        long double sum_x = 0.0L, sum_y = 0.0L, sum_z = 0.0L, sum_station = 0.0L;
        for (std::size_t index = begin; index < end; ++index) {
            const auto& point = ordered[projected[index].second];
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            sum_station += projected[index].first;
        }
        const long double count = static_cast<long double>(end - begin);
        section.base = {static_cast<double>(sum_x / count),
            static_cast<double>(sum_y / count), static_cast<double>(sum_z / count)};
        section.station_m = static_cast<double>(sum_station / count);
        long double xx = 0.0L, xy = 0.0L, yy = 0.0L;
        for (std::size_t index = begin; index < end; ++index) {
            const auto& point = ordered[projected[index].second];
            const long double x = static_cast<long double>(point.x) - section.base.x;
            const long double y = static_cast<long double>(point.y) - section.base.y;
            xx += x * x;
            xy += x * y;
            yy += y * y;
        }
        const long double angle = 0.5L * std::atan2(2.0L * xy, xx - yy);
        section.normal_x = static_cast<double>(std::cos(angle));
        section.normal_y = static_cast<double>(std::sin(angle));
        if (section.normal_y < 0.0) {
            section.normal_x = -section.normal_x;
            section.normal_y = -section.normal_y;
        }
        double lower_m = std::numeric_limits<double>::infinity();
        double upper_m = -lower_m;
        double maximum_residual_m = 0.0;
        for (std::size_t index = begin; index < end; ++index) {
            const auto& point = ordered[projected[index].second];
            const double dx = point.x - section.base.x;
            const double dy = point.y - section.base.y;
            const double across_m = dx * section.normal_x + dy * section.normal_y;
            lower_m = std::min(lower_m, across_m);
            upper_m = std::max(upper_m, across_m);
            maximum_residual_m = std::max(maximum_residual_m,
                std::abs(dx * section.normal_y - dy * section.normal_x));
        }
        if (upper_m - lower_m <= precision_m ||
            maximum_residual_m > section_relative_tolerance * (upper_m - lower_m) ||
            section.normal_y <= 0.0 ||
            (!sections.empty() && (section.station_m <= sections.back().station_m ||
                section.normal_x * sections.back().normal_x +
                section.normal_y * sections.back().normal_y <= 0.9))) {
            return {};
        }
        sections.push_back(section);
        begin = end;
    }
    if (sections.size() < minimum_fit_anchor_count) {
        return {};
    }

    std::vector<double> station, angle, heights, counts;
    for (const auto& section : sections) {
        station.push_back(section.station_m);
        angle.push_back(std::atan2(section.normal_y, section.normal_x));
        heights.push_back(section.base.z);
        counts.push_back(static_cast<double>(section.end - section.begin));
    }
    const auto angle_second = section_derivative(station, section_derivative(station, angle));
    std::vector<double> offset(sections.size(), 0.0);
    std::vector<double> offset_scale(sections.size(), 1.0);
    for (std::size_t index = 1; index < sections.size(); ++index) {
        const double step_m = station[index] - station[index - 1];
        // Integrate the observed directions, including the second-order chord
        // correction for varying curvature and nonuniform section spacing.
        // No circle/spiral type, radius, or transition length is assumed.
        const double chord_normal = 0.5 * (angle[index - 1] + angle[index]) -
            (angle_second[index - 1] + angle_second[index]) * step_m * step_m / 24.0;
        const double nx = std::cos(chord_normal);
        const double ny = std::sin(chord_normal);
        const auto& before = sections[index - 1];
        const auto& after = sections[index];
        const double incoming = before.normal_x * nx + before.normal_y * ny;
        const double outgoing = after.normal_x * nx + after.normal_y * ny;
        if (incoming <= 0.5 || outgoing <= 0.5) {
            return {};
        }
        offset[index] = (offset[index - 1] * incoming -
            (after.base.x - before.base.x) * nx - (after.base.y - before.base.y) * ny) / outgoing;
        offset_scale[index] = offset_scale[index - 1] * incoming / outgoing;
    }
    std::vector<double> across;
    std::vector<double> boundary_midpoints;
    std::vector<double> widths;
    across.reserve(points.size());
    for (std::size_t section_index = 0; section_index < sections.size(); ++section_index) {
        const auto& section = sections[section_index];
        double lower_m = std::numeric_limits<double>::infinity();
        double upper_m = -lower_m;
        for (std::size_t index = section.begin; index < section.end; ++index) {
            const auto& point = ordered[projected[index].second];
            const double value_m = ((point.x - section.base.x) * section.normal_x +
                (point.y - section.base.y) * section.normal_y - offset[section_index]) /
                offset_scale[section_index];
            if (!std::isfinite(value_m)) {
                return {};
            }
            across.push_back(value_m);
            lower_m = std::min(lower_m, value_m);
            upper_m = std::max(upper_m, value_m);
        }
        boundary_midpoints.push_back(0.5 * (lower_m + upper_m));
        widths.push_back(upper_m - lower_m);
    }
    if (!section_boundaries_support_directions(station, boundary_midpoints, widths)) {
        return {};
    }
    const auto tail_weights = separated_tail_weights(across);
    if (std::any_of(tail_weights.begin(), tail_weights.end(),
        [](double weight) { return weight < 0.5; })) {
        // Keep the established robust band model when a separated minority
        // population prevents the full observed envelope defining one offset.
        return {};
    }
    const double global_offset_m = 0.5 * (quantile(across, 0.01) + quantile(across, 0.99));
    std::vector<Point3D> recovered;
    try {
        const auto z_fit = robust_local_quadratic(station, heights, height_half_window_m,
            true, local_minimum_neighbors, counts, true);
        for (std::size_t index = 0; index < sections.size(); ++index) {
            const double displacement_m = offset[index] + offset_scale[index] * global_offset_m;
            const auto& section = sections[index];
            const Point3D point{section.base.x + displacement_m * section.normal_x,
                section.base.y + displacement_m * section.normal_y, z_fit.fitted[index]};
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
                (!recovered.empty() && point.x <= recovered.back().x)) {
                return {};
            }
            recovered.push_back(point);
        }
        // Preserve measured directions between sections too. Linear chords at
        // irregular survey spacing introduce curvature-dependent sag into an
        // otherwise accurate centerline before the final arc-length sampling.
        std::vector<Point3D> interpolated;
        std::vector<double> height_targets;
        interpolated.push_back(recovered.front());
        height_targets.push_back(station.front());
        for (std::size_t index = 1; index < recovered.size(); ++index) {
            const auto& left = recovered[index - 1];
            const auto& right = recovered[index];
            const double dx = right.x - left.x;
            const double count = std::ceil(std::hypot(dx, right.y - left.y) / bin_width_m);
            if (!std::isfinite(count) || count > static_cast<double>(
                maximum_output_point_count - interpolated.size())) {
                return {};
            }
            const std::size_t steps = std::max(std::size_t{1}, static_cast<std::size_t>(count));
            const double left_slope = -sections[index - 1].normal_x / sections[index - 1].normal_y;
            const double right_slope = -sections[index].normal_x / sections[index].normal_y;
            for (std::size_t step = 1; step <= steps; ++step) {
                const double fraction = static_cast<double>(step) / static_cast<double>(steps);
                const double square = fraction * fraction;
                const double cube = square * fraction;
                const double y = (2.0 * cube - 3.0 * square + 1.0) * left.y +
                    (cube - 2.0 * square + fraction) * dx * left_slope +
                    (-2.0 * cube + 3.0 * square) * right.y +
                    (cube - square) * dx * right_slope;
                if (!std::isfinite(y)) {
                    return {};
                }
                interpolated.push_back({left.x + fraction * dx, y, 0.0});
                height_targets.push_back(station[index - 1] +
                    fraction * (station[index] - station[index - 1]));
            }
        }
        const auto interpolated_heights = local_polynomial_predict(station, heights,
            height_targets, height_half_window_m, z_fit.robust_weights,
            true, local_minimum_neighbors, true);
        for (std::size_t index = 0; index < interpolated.size(); ++index) {
            interpolated[index].z = interpolated_heights[index];
        }
        return interpolated;
    } catch (const PointCloudFitError&) {
        // The original valid fit remains authoritative if this optional spatial
        // evidence cannot support the existing height model.
        return {};
    }
}

std::vector<Point3D> sample_curve(
    const std::vector<Point3D>& curve,
    double interval_m) {
    std::vector<double> segment_lengths;
    std::vector<double> cumulative;
    segment_lengths.reserve(curve.size() - 1);
    cumulative.reserve(curve.size());
    cumulative.push_back(0.0);
    for (std::size_t index = 1; index < curve.size(); ++index) {
        const double length = std::hypot(
            curve[index].x - curve[index - 1].x,
            curve[index].y - curve[index - 1].y);
        if (!std::isfinite(length) || length <= 0.0) {
            fail(
                StatusCode::numerical_failure,
                DiagnosticCode::degenerate_fit,
                "Fitted point-cloud centerline contains a zero-length segment");
        }
        segment_lengths.push_back(length);
        const double station = cumulative.back() + length;
        if (!std::isfinite(station)) {
            fail(
                StatusCode::numerical_failure,
                DiagnosticCode::degenerate_fit,
                "Fitted point-cloud centerline length is not finite");
        }
        cumulative.push_back(station);
    }

    const double total_length_m = cumulative.back();
    const long double standard_interval_count = std::floor(
        static_cast<long double>(total_length_m) /
        static_cast<long double>(interval_m));
    if (standard_interval_count >
        static_cast<long double>(maximum_output_point_count - 2)) {
        fail(
            StatusCode::invalid_input,
            DiagnosticCode::invalid_sample_interval,
            "Point-cloud sample interval would produce too many output points");
    }
    const std::size_t last_standard_index =
        static_cast<std::size_t>(standard_interval_count);

    std::vector<double> stations;
    stations.reserve(last_standard_index + 2);
    for (std::size_t index = 0; index <= last_standard_index; ++index) {
        stations.push_back(static_cast<double>(index) * interval_m);
    }
    if (stations.size() == 1 || total_length_m - stations.back() >
        endpoint_remainder_tolerance_m) {
        stations.push_back(total_length_m);
    } else {
        stations.back() = total_length_m;
    }

    std::vector<Point3D> samples;
    samples.reserve(stations.size());
    std::size_t segment_index = 0;
    for (const double station : stations) {
        if (station >= total_length_m) {
            samples.push_back(curve.back());
            continue;
        }
        while (segment_index + 1 < cumulative.size() &&
               cumulative[segment_index + 1] <= station) {
            ++segment_index;
        }
        if (segment_index >= segment_lengths.size()) {
            segment_index = segment_lengths.size() - 1;
        }
        const double fraction =
            (station - cumulative[segment_index]) /
            segment_lengths[segment_index];
        const auto& start = curve[segment_index];
        const auto& end = curve[segment_index + 1];
        samples.push_back({
            start.x + fraction * (end.x - start.x),
            start.y + fraction * (end.y - start.y),
            start.z + fraction * (end.z - start.z)
        });
    }
    return samples;
}

} // namespace

PointCloudFitError::PointCloudFitError(
    StatusCode status,
    DiagnosticCode diagnostic_code,
    std::string message)
    : std::runtime_error(std::move(message)),
      status_(status),
      diagnostic_code_(diagnostic_code) {
}

StatusCode PointCloudFitError::status() const noexcept {
    return status_;
}

DiagnosticCode PointCloudFitError::diagnostic_code() const noexcept {
    return diagnostic_code_;
}

std::vector<Point3D> PointCloudFitter::fit_and_sample(
    const std::vector<Point3D>& points,
    double interval_m) const {
    LocalPointCloud local = center_points(points);
    const PlanBasis basis = estimate_plan_basis(local.points);
    const std::vector<Point3D> oriented_points =
        transform_to_plan_basis(local.points, basis);
    const AnchorSet anchors = spatial_bin_anchors(oriented_points);
    if (anchors.centers.size() < minimum_fit_anchor_count) {
        fail(
            StatusCode::invalid_input,
            DiagnosticCode::insufficient_spatial_support,
            "At least three distinct longitudinal anchors are required");
    }
    bool sparse_support = anchors.centers.size() < minimum_anchor_count ||
        std::any_of(
            anchors.counts.begin(),
            anchors.counts.end(),
            [](std::size_t count) {
                return count < minimum_points_per_bin;
            });
    for (std::size_t index = 1; index < anchors.centers.size(); ++index) {
        if (anchors.centers[index].x - anchors.centers[index-1].x >=
            height_half_window_m * 0.5) {
            sparse_support = true;
        }
    }

    std::vector<Point3D> curve = sparse_support
        ? build_sparse_curve(oriented_points, anchors)
        : build_curve(oriented_points, anchors);
    const auto directional_curve = recover_section_directions(oriented_points, curve);
    if (!directional_curve.empty()) {
        curve = directional_curve;
    }
    std::vector<Point3D> samples = sample_curve(curve, interval_m);
    transform_from_plan_basis(samples, basis, local.origin);
    return samples;
}

} // namespace alignment_reconstruct::detail
