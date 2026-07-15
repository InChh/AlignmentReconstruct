#include "vertical/vertical_reconstructor.h"

#include "vertical/vertical_alignment.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace alignment_reconstruct::detail {
namespace {

struct FittedGradeLine {
    ProfileClassificationRun run;
    double grade = 0.0;
    double intercept = 0.0;
};

FittedGradeLine fit_grade_line(
    const std::vector<ProfileSample>& samples,
    ProfileClassificationRun run) {
    std::size_t first = run.first_index;
    std::size_t last = run.last_index;
    if (first == last) {
        if (first == 0) {
            last = 1;
        } else {
            --first;
        }
    }
    double mean_station = 0.0;
    double mean_elevation = 0.0;
    const double count = static_cast<double>(last - first + 1);
    for (std::size_t index = first; index <= last; ++index) {
        mean_station += samples[index].station_m;
        mean_elevation += samples[index].elevation_m;
    }
    mean_station /= count;
    mean_elevation /= count;
    double station_variance = 0.0;
    double covariance = 0.0;
    for (std::size_t index = first; index <= last; ++index) {
        const double station_delta = samples[index].station_m - mean_station;
        station_variance += station_delta * station_delta;
        covariance += station_delta * (samples[index].elevation_m - mean_elevation);
    }
    if (station_variance <= 1.0e-18) {
        throw std::invalid_argument("Vertical grade fit is degenerate");
    }
    const double grade = covariance / station_variance;
    return {run, grade, mean_elevation - grade * mean_station};
}

double line_elevation(const FittedGradeLine& line, double station_m) {
    return line.grade * station_m + line.intercept;
}

VerticalControlPoint intersect_grade_lines(
    const FittedGradeLine& incoming,
    const FittedGradeLine& outgoing) {
    const double difference = incoming.grade - outgoing.grade;
    if (std::abs(difference) <= 1.0e-12) {
        throw std::invalid_argument("Adjacent fitted vertical grades are parallel");
    }
    const double station = (outgoing.intercept - incoming.intercept) / difference;
    return {station, line_elevation(incoming, station), 0.0};
}

double circular_profile_elevation(
    double station_m,
    const VerticalControlPoint& intersection,
    double incoming_grade,
    double outgoing_grade,
    double radius_m) {
    if (radius_m <= 0.0) {
        const double grade = station_m < intersection.station_m
            ? incoming_grade : outgoing_grade;
        return intersection.elevation_m + grade * (station_m - intersection.station_m);
    }
    const double incoming_angle = std::atan(incoming_grade);
    const double outgoing_angle = std::atan(outgoing_grade);
    const double signed_deflection = outgoing_angle - incoming_angle;
    const int turn_sign = signed_deflection > 0.0 ? 1 : -1;
    const double tangent = radius_m * std::tan(0.5 * std::abs(signed_deflection));
    const double start_station = intersection.station_m - tangent * std::cos(incoming_angle);
    const double end_station = intersection.station_m + tangent * std::cos(outgoing_angle);
    if (station_m < start_station) {
        return intersection.elevation_m + incoming_grade *
            (station_m - intersection.station_m);
    }
    if (station_m > end_station) {
        return intersection.elevation_m + outgoing_grade *
            (station_m - intersection.station_m);
    }
    const double start_elevation = intersection.elevation_m - tangent * std::sin(incoming_angle);
    const double center_station = start_station -
        static_cast<double>(turn_sign) * radius_m * std::sin(incoming_angle);
    const double center_elevation = start_elevation +
        static_cast<double>(turn_sign) * radius_m * std::cos(incoming_angle);
    const double horizontal = station_m - center_station;
    const double vertical = std::sqrt(std::max(
        0.0, radius_m * radius_m - horizontal * horizontal));
    return center_elevation - static_cast<double>(turn_sign) * vertical;
}

double radius_objective(
    const std::vector<ProfileSample>& samples,
    std::size_t first,
    std::size_t last,
    const VerticalControlPoint& intersection,
    double incoming_grade,
    double outgoing_grade,
    double radius_m) {
    double sum = 0.0;
    for (std::size_t index = first; index <= last; ++index) {
        const double residual = circular_profile_elevation(
            samples[index].station_m, intersection,
            incoming_grade, outgoing_grade, radius_m) - samples[index].elevation_m;
        sum += residual * residual;
    }
    return sum / static_cast<double>(last - first + 1);
}

double fit_vertical_radius(
    const std::vector<ProfileSample>& samples,
    std::size_t first,
    std::size_t last,
    const VerticalControlPoint& intersection,
    double incoming_grade,
    double outgoing_grade,
    const ReconstructionOptions& options) {
    if (last - first + 1 < 3 ||
        std::abs(outgoing_grade - incoming_grade) < 1.0e-7) {
        return 0.0;
    }
    double lower = options.minimum_vertical_curve_radius_m;
    double upper = options.maximum_vertical_curve_radius_m;
    if (!(lower > 0.0) || !(upper >= lower)) {
        throw std::invalid_argument("Invalid vertical curve radius search bounds");
    }
    constexpr double inverse_phi = 0.6180339887498948482;
    double left = upper - inverse_phi * (upper - lower);
    double right = lower + inverse_phi * (upper - lower);
    double left_value = radius_objective(
        samples, first, last, intersection, incoming_grade, outgoing_grade, left);
    double right_value = radius_objective(
        samples, first, last, intersection, incoming_grade, outgoing_grade, right);
    for (int iteration = 0; iteration < 80; ++iteration) {
        if (left_value < right_value) {
            upper = right;
            right = left;
            right_value = left_value;
            left = upper - inverse_phi * (upper - lower);
            left_value = radius_objective(
                samples, first, last, intersection, incoming_grade, outgoing_grade, left);
        } else {
            lower = left;
            left = right;
            left_value = right_value;
            right = lower + inverse_phi * (upper - lower);
            right_value = radius_objective(
                samples, first, last, intersection, incoming_grade, outgoing_grade, right);
        }
    }
    double radius = 0.5 * (lower + upper);
    const double rounding = options.vertical_curve_radius_rounding_m;
    if (rounding > 0.0) {
        radius = std::floor(radius / rounding) * rounding;
    }
    return std::max(options.minimum_vertical_curve_radius_m,
                    std::min(radius, options.maximum_vertical_curve_radius_m));
}

void recalculate_grade_constrained_elevations(
    std::vector<VerticalControlPoint>& controls,
    const ReconstructionOptions& options) {
    if (controls.size() < 2) {
        return;
    }
    const double grade_step = options.grade_rounding_per_mille * 0.001;
    for (std::size_t index = 0; index + 1 < controls.size(); ++index) {
        const double length = controls[index + 1].station_m - controls[index].station_m;
        double grade = (controls[index + 1].elevation_m - controls[index].elevation_m) / length;
        if (grade_step > 0.0) {
            grade = std::round(grade / grade_step) * grade_step;
        }
        if (options.maximum_up_grade > 0.0) {
            grade = std::min(grade, options.maximum_up_grade * 0.001);
        }
        if (options.maximum_down_grade > 0.0) {
            grade = std::max(grade, -options.maximum_down_grade * 0.001);
        }
        controls[index + 1].elevation_m = controls[index].elevation_m + grade * length;
    }
}

void enforce_grade_difference_limit(
    std::vector<VerticalControlPoint>& controls,
    double maximum_difference_per_mille) {
    if (maximum_difference_per_mille <= 0.0 || controls.size() < 3) {
        return;
    }
    const double limit = maximum_difference_per_mille * 0.001;
    for (std::size_t pass = 0; pass < controls.size(); ++pass) {
        bool changed = false;
        for (std::size_t index = 1; index + 1 < controls.size(); ++index) {
            const double incoming_length =
                controls[index].station_m - controls[index - 1].station_m;
            const double outgoing_length =
                controls[index + 1].station_m - controls[index].station_m;
            const double incoming_grade =
                (controls[index].elevation_m - controls[index - 1].elevation_m) /
                incoming_length;
            const double outgoing_grade =
                (controls[index + 1].elevation_m - controls[index].elevation_m) /
                outgoing_length;
            const double difference = outgoing_grade - incoming_grade;
            if (std::abs(difference) <= limit + 1.0e-12) {
                continue;
            }
            const double target_difference = difference > 0.0 ? limit : -limit;
            controls[index].elevation_m =
                (controls[index + 1].elevation_m / outgoing_length +
                 controls[index - 1].elevation_m / incoming_length - target_difference) /
                (1.0 / incoming_length + 1.0 / outgoing_length);
            changed = true;
        }
        if (!changed) {
            break;
        }
    }
}

void merge_small_grade_differences(std::vector<VerticalControlPoint>& controls) {
    for (std::size_t index = 1; index + 1 < controls.size();) {
        const double incoming = (controls[index].elevation_m - controls[index - 1].elevation_m) /
            (controls[index].station_m - controls[index - 1].station_m);
        const double outgoing = (controls[index + 1].elevation_m - controls[index].elevation_m) /
            (controls[index + 1].station_m - controls[index].station_m);
        if (std::abs(outgoing - incoming) < 0.0001) {
            controls.erase(controls.begin() + static_cast<std::ptrdiff_t>(index));
        } else {
            ++index;
        }
    }
}

void merge_short_grades(
    std::vector<VerticalControlPoint>& controls,
    double minimum_length_m) {
    if (minimum_length_m <= 0.0) {
        return;
    }
    for (std::size_t pass = 0; pass < controls.size(); ++pass) {
        if (controls.size() < 4) {
            return;
        }
        std::size_t shortest = controls.size();
        double shortest_length = minimum_length_m;
        for (std::size_t index = 0; index + 1 < controls.size(); ++index) {
            const double length = controls[index + 1].station_m - controls[index].station_m;
            if (length < shortest_length) {
                shortest_length = length;
                shortest = index;
            }
        }
        if (shortest == controls.size()) {
            return;
        }
        std::size_t remove = 0;
        if (shortest == 0) {
            const double first_grade = (controls[1].elevation_m - controls[0].elevation_m) /
                (controls[1].station_m - controls[0].station_m);
            const double second_grade = (controls[2].elevation_m - controls[1].elevation_m) /
                (controls[2].station_m - controls[1].station_m);
            if (std::abs(second_grade - first_grade) >= 0.002) {
                return;
            }
            remove = 1;
        } else if (shortest + 2 == controls.size()) {
            const std::size_t last = controls.size() - 1;
            const double first_grade = (controls[last - 1].elevation_m - controls[last - 2].elevation_m) /
                (controls[last - 1].station_m - controls[last - 2].station_m);
            const double second_grade = (controls[last].elevation_m - controls[last - 1].elevation_m) /
                (controls[last].station_m - controls[last - 1].station_m);
            if (std::abs(second_grade - first_grade) >= 0.002) {
                return;
            }
            remove = last - 1;
        } else {
            const double left_difference = std::abs(
                (controls[shortest].elevation_m - controls[shortest - 1].elevation_m) /
                    (controls[shortest].station_m - controls[shortest - 1].station_m) -
                (controls[shortest + 1].elevation_m - controls[shortest].elevation_m) /
                    (controls[shortest + 1].station_m - controls[shortest].station_m));
            const double right_difference = std::abs(
                (controls[shortest + 1].elevation_m - controls[shortest].elevation_m) /
                    (controls[shortest + 1].station_m - controls[shortest].station_m) -
                (controls[shortest + 2].elevation_m - controls[shortest + 1].elevation_m) /
                    (controls[shortest + 2].station_m - controls[shortest + 1].station_m));
            remove = left_difference <= right_difference ? shortest : shortest + 1;
            if (remove == 0) {
                remove = 1;
            }
        }
        controls.erase(controls.begin() + static_cast<std::ptrdiff_t>(remove));
    }
}

std::vector<VerticalControlPoint> reconstruct_for_threshold(
    const std::vector<ProfileSample>& samples,
    double threshold,
    const ReconstructionOptions& options) {
    const auto analysis = classify_profile_samples(samples, threshold);
    std::vector<FittedGradeLine> lines;
    std::vector<ProfileClassificationRun> curves;
    for (const auto& run : analysis.runs) {
        if (run.curved) {
            curves.push_back(run);
        } else {
            lines.push_back(fit_grade_line(samples, run));
        }
    }
    if (lines.empty() || analysis.runs.front().curved || analysis.runs.back().curved ||
        lines.size() != curves.size() + 1) {
        throw std::invalid_argument("Vertical straight/curve segmentation is inconsistent");
    }

    std::vector<VerticalControlPoint> controls;
    controls.reserve(lines.size() + 1);
    controls.push_back({samples.front().station_m, samples.front().elevation_m, 0.0});
    for (std::size_t index = 0; index < curves.size(); ++index) {
        auto intersection = intersect_grade_lines(lines[index], lines[index + 1]);
        const double minimum_station = samples[lines[index].run.first_index].station_m;
        const double maximum_station = samples[lines[index + 1].run.last_index].station_m;
        if (intersection.station_m <= minimum_station || intersection.station_m >= maximum_station) {
            throw std::invalid_argument("Vertical grade intersection falls outside adjacent segments");
        }
        if (options.station_rounding_m > 0.0) {
            intersection.station_m = std::round(
                intersection.station_m / options.station_rounding_m) * options.station_rounding_m;
            intersection.elevation_m = 0.5 * (
                line_elevation(lines[index], intersection.station_m) +
                line_elevation(lines[index + 1], intersection.station_m));
        }
        const std::size_t objective_first = lines[index].run.last_index > 5
            ? std::max(lines[index].run.first_index, lines[index].run.last_index - 5)
            : lines[index].run.first_index;
        const std::size_t objective_last = std::min(
            lines[index + 1].run.last_index, lines[index + 1].run.first_index + 5);
        const std::size_t curve_sample_count =
            curves[index].last_index - curves[index].first_index + 1;
        intersection.vertical_curve_radius_m = curve_sample_count >= 3
            ? fit_vertical_radius(
                samples, objective_first, objective_last, intersection,
                lines[index].grade, lines[index + 1].grade, options)
            : 0.0;
        controls.push_back(intersection);
    }
    controls.push_back({samples.back().station_m, samples.back().elevation_m, 0.0});
    merge_small_grade_differences(controls);
    merge_short_grades(controls, options.minimum_grade_length_m);
    recalculate_grade_constrained_elevations(controls, options);
    enforce_grade_difference_limit(controls, options.maximum_grade_difference_per_mille);
    controls.front().vertical_curve_radius_m = 0.0;
    controls.back().vertical_curve_radius_m = 0.0;
    const VerticalAlignment closure(controls);
    (void)closure;
    return controls;
}

double reconstruction_score(
    const std::vector<ProfileSample>& samples,
    const std::vector<VerticalControlPoint>& controls) {
    const VerticalAlignment alignment(controls);
    double sum = 0.0;
    for (const auto& sample : samples) {
        const double residual = alignment.evaluate(sample.station_m).elevation_m - sample.elevation_m;
        sum += residual * residual;
    }
    return sum / static_cast<double>(samples.size()) +
        1.0e-10 * static_cast<double>(controls.size());
}

} // namespace

VerticalReconstructor::VerticalReconstructor(ReconstructionOptions options)
    : options_(std::move(options)) {}

std::vector<VerticalControlPoint> VerticalReconstructor::reconstruct(
    const std::vector<ProfileSample>& ordered_samples) const {
    if (ordered_samples.size() < 2) {
        throw std::invalid_argument("At least two vertical samples are required");
    }
    std::vector<double> thresholds = options_.vertical_second_difference_threshold_candidates;
    if (thresholds.empty()) {
        thresholds.push_back(5.0e-6);
    }
    std::vector<VerticalControlPoint> best;
    double best_score = std::numeric_limits<double>::infinity();
    for (const double threshold : thresholds) {
        try {
            auto candidate = reconstruct_for_threshold(ordered_samples, threshold, options_);
            const double score = reconstruction_score(ordered_samples, candidate);
            if (score < best_score) {
                best_score = score;
                best = std::move(candidate);
            }
        } catch (const std::invalid_argument&) {
            continue;
        }
    }
    if (best.empty()) {
        throw std::invalid_argument("No vertical classification threshold produced feasible geometry");
    }
    return best;
}

} // namespace alignment_reconstruct::detail
