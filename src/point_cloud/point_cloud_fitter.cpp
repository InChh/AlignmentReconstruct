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
constexpr std::size_t maximum_output_point_count = 10'000'000;
constexpr double endpoint_remainder_tolerance_m = 0.01;
constexpr long double minimum_plan_variance_ratio = 4.0L;
constexpr double axis_snap_sine = 0.10452846326765347; // sin(6 degrees)

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

std::array<long double, 3> solve_symmetric_system(
    std::array<std::array<long double, 3>, 3> matrix,
    std::array<long double, 3> right_hand_side) {
    long double maximum_coefficient = 0.0L;
    for (const auto& row : matrix) {
        for (const long double value : row) {
            maximum_coefficient = std::max(maximum_coefficient, std::abs(value));
        }
    }
    const long double pivot_tolerance =
        std::numeric_limits<long double>::epsilon() *
        std::max(1.0L, maximum_coefficient) * 64.0L;

    for (std::size_t column = 0; column < 3; ++column) {
        std::size_t pivot_row = column;
        long double pivot_magnitude = std::abs(matrix[column][column]);
        for (std::size_t row = column + 1; row < 3; ++row) {
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

        for (std::size_t row = column + 1; row < 3; ++row) {
            const long double factor =
                matrix[row][column] / matrix[column][column];
            for (std::size_t inner = column; inner < 3; ++inner) {
                matrix[row][inner] -= factor * matrix[column][inner];
            }
            right_hand_side[row] -= factor * right_hand_side[column];
        }
    }

    std::array<long double, 3> solution{};
    for (std::size_t reverse = 3; reverse > 0; --reverse) {
        const std::size_t row = reverse - 1;
        long double value = right_hand_side[row];
        for (std::size_t column = row + 1; column < 3; ++column) {
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
    std::size_t adaptive_neighbor_count = minimum_fit_anchor_count) {
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

    const auto coefficients = local_coefficients(
        x, values, selected, weights, target, adapt_to_support ? coordinate_scale_m : 1.0);
    const double prediction = static_cast<double>(coefficients[0]);
    if (!std::isfinite(prediction)) {
        fail(
            StatusCode::numerical_failure,
            DiagnosticCode::degenerate_fit,
            "Local quadratic point-cloud fit produced a non-finite coordinate");
    }
    return prediction;
}

std::array<long double, 3> quantile_coefficients(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<std::size_t>& selected,
    const std::vector<double>& distance_weights,
    double target,
    double half_window_m,
    double probability,
    bool quadratic = true) {
    auto coefficients = local_coefficients(
        x, y, selected, distance_weights, target, half_window_m, quadratic);
    std::vector<double> weights(selected.size());
    for (std::size_t iteration = 0; iteration < 60; ++iteration) {
        for (std::size_t index = 0; index < selected.size(); ++index) {
            const std::size_t point_index = selected[index];
            const long double dx =
                (static_cast<long double>(x[point_index]) - target) / half_window_m;
            const double residual = y[point_index] - static_cast<double>(
                coefficients[0] + dx * (coefficients[1] + dx * coefficients[2]));
            const double asymmetry = residual >= 0.0 ? probability : 1.0 - probability;
            weights[index] = distance_weights[index] * asymmetry /
                std::max(std::abs(residual), 1.0e-4);
        }
        const auto updated = local_coefficients(
            x, y, selected, weights, target, half_window_m, quadratic);
        long double change = 0.0L;
        for (std::size_t index = 0; index < coefficients.size(); ++index) {
            change = std::max(change, std::abs(updated[index] - coefficients[index]));
        }
        coefficients = updated;
        if (change < 1.0e-7L) {
            break;
        }
    }
    return coefficients;
}

double band_center_prediction(
    const std::vector<double>& x,
    const std::vector<double>& y,
    double target,
    double half_window_m) {
    std::vector<std::size_t> selected;
    for (std::size_t index = 0; index < x.size(); ++index) {
        if (std::abs(x[index] - target) < half_window_m) {
            selected.push_back(index);
        }
    }
    const std::size_t minimum_neighbors = std::min(local_minimum_neighbors, x.size());
    if (selected.size() < minimum_neighbors) {
        selected.resize(x.size());
        std::iota(selected.begin(), selected.end(), std::size_t{0});
        std::stable_sort(selected.begin(), selected.end(), [&](std::size_t left, std::size_t right) {
            return std::abs(x[left] - target) < std::abs(x[right] - target);
        });
        selected.resize(minimum_neighbors);
        std::sort(selected.begin(), selected.end());
    }
    for (const std::size_t index : selected) {
        half_window_m = std::max(half_window_m, 1.01 * std::abs(x[index] - target));
    }
    std::vector<double> weights;
    weights.reserve(selected.size());
    for (const std::size_t index : selected) {
        const double distance = std::abs(x[index] - target) / half_window_m;
        const double complement = 1.0 - distance * distance * distance;
        weights.push_back(complement * complement * complement);
    }
    auto lower = quantile_coefficients(x, y, selected, weights, target, half_window_m, 0.01);
    auto upper = quantile_coefficients(x, y, selected, weights, target, half_window_m, 0.99);
    // If the two boundaries do not agree on curvature, a quadratic term is
    // fitting boundary noise. Estimate the local trend with a linear model.
    if (std::abs(lower[2] + upper[2]) <= 2.0L * std::abs(lower[2] - upper[2])) {
        lower = quantile_coefficients(x, y, selected, weights, target, half_window_m, 0.01, false);
        upper = quantile_coefficients(x, y, selected, weights, target, half_window_m, 0.99, false);
    }
    return static_cast<double>((lower[0] + upper[0]) * 0.5L);
}

std::vector<double> local_polynomial_predict(
    const std::vector<double>& x,
    const std::vector<double>& values,
    const std::vector<double>& targets,
    double half_window_m,
    const std::vector<double>& robust_weights,
    bool adapt_to_support = false,
    std::size_t adaptive_neighbor_count = minimum_fit_anchor_count) {
    std::vector<double> predictions;
    predictions.reserve(targets.size());
    for (const double target : targets) {
        predictions.push_back(local_polynomial_prediction(
            x, values, target, half_window_m, robust_weights, adapt_to_support,
            adaptive_neighbor_count));
    }
    return predictions;
}

RobustFit robust_local_quadratic(
    const std::vector<double>& x,
    const std::vector<double>& values,
    double half_window_m,
    bool adapt_to_support = false,
    std::size_t adaptive_neighbor_count = minimum_fit_anchor_count,
    const std::vector<double>& observation_weights = {}) {
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
            adaptive_neighbor_count);

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
        adaptive_neighbor_count);
    return result;
}

std::vector<Point3D> build_curve(
    const std::vector<Point3D>& points,
    const AnchorSet& anchors,
    bool adapt_to_support = false,
    double plan_window_m = plan_half_window_m,
    std::size_t adaptive_neighbor_count = minimum_fit_anchor_count) {
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
            adaptive_neighbor_count, observation_weights);
    const RobustFit z_fit =
        robust_local_quadratic(x, z, height_half_window_m, adapt_to_support,
            adaptive_neighbor_count, observation_weights);

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
        adaptive_neighbor_count);
    const std::vector<double> endpoint_z = local_polynomial_predict(
        x, z, endpoint_x, height_half_window_m, z_fit.robust_weights, adapt_to_support,
        adaptive_neighbor_count);

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
    double half_window_m) {
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
    std::vector<double> targets{raw_x.front()};
    for (const double x : anchor_x) {
        if (x - targets.back() >= knot_spacing_m) {
            targets.push_back(x);
        }
    }
    if (raw_x.back() > targets.back()) {
        targets.push_back(raw_x.back());
    }
    // Keep three model positions even when the occupied support is short.
    if (targets.size() < minimum_fit_anchor_count) {
        targets = anchor_x;
    }
    std::vector<double> center_y;
    center_y.reserve(targets.size());
    for (const double target : targets) {
        center_y.push_back(band_center_prediction(raw_x, raw_y, target, half_window_m));
    }
    const auto y_fit = robust_local_quadratic(targets, center_y, plan_half_window_m, true);
    const auto z_fit = robust_local_quadratic(anchor_x, anchor_z, height_half_window_m,
        true, local_minimum_neighbors, counts);
    const auto heights = local_polynomial_predict(anchor_x, anchor_z, targets,
        height_half_window_m, z_fit.robust_weights, true, local_minimum_neighbors);
    std::vector<Point3D> curve;
    curve.reserve(targets.size());
    for (std::size_t index = 0; index < targets.size(); ++index) {
        curve.push_back({targets[index], y_fit.fitted[index], heights[index]});
    }
    return curve;
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
    for (double& residual : residuals) {
        residual = std::abs(residual - center);
    }
    const double transverse_scale_m = std::max(
        1.4826 * median(std::move(residuals)), residual_span_m / 3.29);
    const double plan_window_m = std::max(
        plan_half_window_m, 10.0 * transverse_scale_m);
    if (plan_window_m == plan_half_window_m) {
        return initial;
    }

    // A band's geometric center is the midpoint of its two robust boundaries;
    // a population mean follows sampling density and can drift to either side.
    std::vector<Point3D> ordered = points;
    std::sort(ordered.begin(), ordered.end(), point_less);
    std::vector<Point3D> section_points = points;
    for (Point3D& point : section_points) {
        // Use the fitted local tangent to remove a section's transverse
        // contribution to the global longitudinal coordinate, including at
        // the endpoints. Z still belongs to that same physical section.
        point.x = section_longitudinal_coordinate(ordered, point,
            std::max(plan_half_window_m, 6.0 * transverse_scale_m));
    }
    std::sort(section_points.begin(), section_points.end(), point_less);
    auto curve = build_band_curve(section_points, plan_window_m);
    for (std::size_t iteration = 1; iteration < robust_iterations; ++iteration) {
        section_points = ordered;
        for (auto& point : section_points) {
            const double source_x = point.x;
            double longitudinal_m = source_x;
            for (std::size_t step = 0; step < 5; ++step) {
                double slope = 0.0;
                const double transverse_m = interpolate_transverse(curve, longitudinal_m, &slope);
                longitudinal_m += (source_x - longitudinal_m +
                    slope * (point.y - transverse_m)) / (1.0 + slope * slope);
            }
            point.x = longitudinal_m;
        }
        std::sort(section_points.begin(), section_points.end(), point_less);
        curve = build_band_curve(section_points, plan_window_m);
    }
    return curve;
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

    const std::vector<Point3D> curve = sparse_support
        ? build_sparse_curve(oriented_points, anchors)
        : build_curve(oriented_points, anchors);
    std::vector<Point3D> samples = sample_curve(curve, interval_m);
    transform_from_plan_basis(samples, basis, local.origin);
    return samples;
}

} // namespace alignment_reconstruct::detail
