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
constexpr double plan_half_window_m = 12.0;
constexpr double height_half_window_m = 6.0;
constexpr std::size_t robust_iterations = 3;
constexpr std::size_t minimum_anchor_count = 20;
constexpr std::size_t minimum_points_per_bin = 3;
constexpr std::size_t local_minimum_neighbors = 9;
constexpr std::size_t maximum_output_point_count = 10'000'000;
constexpr double endpoint_remainder_tolerance_m = 0.01;

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

long long bin_identifier(double x, double minimum_x) {
    const long double raw_identifier = std::floor(
        (static_cast<long double>(x) - static_cast<long double>(minimum_x)) /
        static_cast<long double>(bin_width_m));
    const long double minimum =
        static_cast<long double>(std::numeric_limits<long long>::min());
    const long double maximum =
        static_cast<long double>(std::numeric_limits<long long>::max());
    if (raw_identifier < minimum || raw_identifier > maximum) {
        fail(
            StatusCode::invalid_input,
            DiagnosticCode::insufficient_spatial_support,
            "Point-cloud X range is too large for one-metre spatial bins");
    }
    return static_cast<long long>(raw_identifier);
}

AnchorSet spatial_bin_medians(const std::vector<Point3D>& points) {
    const auto minimum_iterator = std::min_element(
        points.begin(),
        points.end(),
        [](const Point3D& left, const Point3D& right) {
            return left.x < right.x;
        });
    const double minimum_x = minimum_iterator->x;

    std::map<long long, std::vector<Point3D>> bins;
    for (const auto& point : points) {
        bins[bin_identifier(point.x, minimum_x)].push_back(point);
    }

    AnchorSet anchors;
    anchors.centers.reserve(bins.size());
    anchors.counts.reserve(bins.size());
    for (const auto& bin : bins) {
        anchors.centers.push_back(coordinate_median(bin.second));
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

double local_polynomial_prediction(
    const std::vector<double>& x,
    const std::vector<double>& values,
    double target,
    double half_window_m,
    const std::vector<double>& robust_weights) {
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
        std::min(local_minimum_neighbors, x.size());
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

    std::vector<double> distance_weights;
    std::vector<double> weights;
    distance_weights.reserve(selected.size());
    weights.reserve(selected.size());
    std::size_t positive_weight_count = 0;
    for (const std::size_t index : selected) {
        const double scaled =
            std::min(distances[index] / half_window_m, 1.0);
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

    std::array<std::array<long double, 3>, 3> normal{};
    std::array<long double, 3> right_hand_side{};
    for (std::size_t selected_index = 0;
         selected_index < selected.size();
         ++selected_index) {
        const std::size_t input_index = selected[selected_index];
        const long double dx =
            static_cast<long double>(x[input_index]) -
            static_cast<long double>(target);
        const std::array<long double, 3> basis{1.0L, dx, dx * dx};
        const long double weight = static_cast<long double>(
            std::max(weights[selected_index], 1.0e-15));
        const long double value =
            static_cast<long double>(values[input_index]);
        for (std::size_t row = 0; row < 3; ++row) {
            right_hand_side[row] += weight * basis[row] * value;
            for (std::size_t column = 0; column < 3; ++column) {
                normal[row][column] +=
                    weight * basis[row] * basis[column];
            }
        }
    }

    const auto coefficients =
        solve_symmetric_system(std::move(normal), std::move(right_hand_side));
    const double prediction = static_cast<double>(coefficients[0]);
    if (!std::isfinite(prediction)) {
        fail(
            StatusCode::numerical_failure,
            DiagnosticCode::degenerate_fit,
            "Local quadratic point-cloud fit produced a non-finite coordinate");
    }
    return prediction;
}

std::vector<double> local_polynomial_predict(
    const std::vector<double>& x,
    const std::vector<double>& values,
    const std::vector<double>& targets,
    double half_window_m,
    const std::vector<double>& robust_weights) {
    std::vector<double> predictions;
    predictions.reserve(targets.size());
    for (const double target : targets) {
        predictions.push_back(local_polynomial_prediction(
            x, values, target, half_window_m, robust_weights));
    }
    return predictions;
}

RobustFit robust_local_quadratic(
    const std::vector<double>& x,
    const std::vector<double>& values,
    double half_window_m) {
    RobustFit result;
    result.robust_weights.assign(x.size(), 1.0);
    result.fitted = values;

    for (std::size_t iteration = 0;
         iteration < robust_iterations;
         ++iteration) {
        result.fitted = local_polynomial_predict(
            x, values, x, half_window_m, result.robust_weights);

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
        }
    }

    result.fitted = local_polynomial_predict(
        x, values, x, half_window_m, result.robust_weights);
    return result;
}

std::vector<Point3D> build_curve(
    const std::vector<Point3D>& points,
    const AnchorSet& anchors) {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    x.reserve(anchors.centers.size());
    y.reserve(anchors.centers.size());
    z.reserve(anchors.centers.size());
    for (const auto& anchor : anchors.centers) {
        x.push_back(anchor.x);
        y.push_back(anchor.y);
        z.push_back(anchor.z);
    }

    const RobustFit y_fit =
        robust_local_quadratic(x, y, plan_half_window_m);
    const RobustFit z_fit =
        robust_local_quadratic(x, z, height_half_window_m);

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
        x, y, endpoint_x, plan_half_window_m, y_fit.robust_weights);
    const std::vector<double> endpoint_z = local_polynomial_predict(
        x, z, endpoint_x, height_half_window_m, z_fit.robust_weights);

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
            "Point-cloud centerline does not contain two distinct X positions");
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
    if (total_length_m - stations.back() >
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
    const AnchorSet anchors = spatial_bin_medians(local.points);
    if (anchors.centers.size() < minimum_anchor_count) {
        fail(
            StatusCode::invalid_input,
            DiagnosticCode::insufficient_spatial_support,
            "At least twenty populated one-metre X bins are required");
    }
    if (std::any_of(
            anchors.counts.begin(),
            anchors.counts.end(),
            [](std::size_t count) {
                return count < minimum_points_per_bin;
            })) {
        fail(
            StatusCode::invalid_input,
            DiagnosticCode::insufficient_spatial_support,
            "Every populated one-metre X bin must contain at least three points");
    }

    const std::vector<Point3D> curve = build_curve(local.points, anchors);
    std::vector<Point3D> samples = sample_curve(curve, interval_m);
    for (auto& sample : samples) {
        sample.x += local.origin.x;
        sample.y += local.origin.y;
        sample.z += local.origin.z;
        if (!std::isfinite(sample.x) ||
            !std::isfinite(sample.y) ||
            !std::isfinite(sample.z)) {
            fail(
                StatusCode::numerical_failure,
                DiagnosticCode::degenerate_fit,
                "Point-cloud centerline output contains a non-finite coordinate");
        }
    }
    return samples;
}

} // namespace alignment_reconstruct::detail
