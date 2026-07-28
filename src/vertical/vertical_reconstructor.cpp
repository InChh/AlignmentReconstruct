#include "vertical/vertical_reconstructor.h"

#include "vertical/vertical_alignment.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace alignment_reconstruct::detail {
namespace {

constexpr double parallel_grade_tolerance = 1.0e-12;
constexpr double grade_merge_tolerance = 0.0001;
constexpr double protected_end_grade_difference = 0.002;
constexpr double intersection_margin_m = 0.1;
constexpr double initial_curve_half_window_m = 100.0;
constexpr double active_maximum_vertical_curve_radius_m = 30000.0;

struct WorkingSample {
    double station_m = 0.0;
    double elevation_m = 0.0;
    bool curved = false;
};

struct GradeLine {
    std::size_t first_index = 0;
    std::size_t last_index = 0;
    std::ptrdiff_t next_curve_sample_count = 0;
    double start_station_m = 0.0;
    double start_elevation_m = 0.0;
    double grade = 0.0;
};

struct SlopePoint {
    double station_m = 0.0;
    double elevation_m = 0.0;
    double vertical_curve_radius_m = 0.0;
    double grade = 0.0;
    double length_m = 0.0;
};

struct ReconstructionCandidate {
    std::vector<SlopePoint> slopes;
    std::vector<VerticalControlPoint> controls;
};

double line_elevation(const GradeLine& line, double station_m) {
    return line.start_elevation_m +
        line.grade * (station_m - line.start_station_m);
}

void refresh_curve_sample_counts(std::vector<GradeLine>& lines) {
    for (std::size_t index = 0; index < lines.size(); ++index) {
        lines[index].next_curve_sample_count =
            index + 1 < lines.size()
                ? static_cast<std::ptrdiff_t>(
                    lines[index + 1].first_index) -
                    static_cast<std::ptrdiff_t>(
                        lines[index].last_index) -
                    1
                : 0;
    }
}

std::vector<GradeLine> form_grade_lines(
    const std::vector<WorkingSample>& samples) {
    std::vector<GradeLine> lines;
    std::size_t index = 0;
    while (index < samples.size()) {
        while (index < samples.size() && samples[index].curved) {
            ++index;
        }
        if (index == samples.size()) {
            break;
        }
        GradeLine line;
        line.first_index = index;
        while (index + 1 < samples.size() && !samples[index + 1].curved) {
            ++index;
        }
        line.last_index = index;
        while (index + 1 < samples.size() && samples[index + 1].curved) {
            ++index;
        }
        line.next_curve_sample_count =
            static_cast<std::ptrdiff_t>(index - line.last_index);
        lines.push_back(line);
        ++index;
    }
    if (lines.empty()) {
        throw std::invalid_argument("Vertical classification contains no grade samples");
    }
    return lines;
}

void fit_grade_line(
    std::vector<WorkingSample>& samples,
    GradeLine& line) {
    double start_station_m = samples[line.first_index].station_m;
    if (line.last_index <= line.first_index) {
        if (line.first_index == 0) {
            if (line.last_index + 1 >= samples.size()) {
                throw std::invalid_argument("Vertical grade fit is degenerate");
            }
            ++line.last_index;
            samples[line.last_index].curved = false;
        } else {
            line.last_index = line.first_index;
            --line.first_index;
            samples[line.first_index].curved = false;
        }
    }

    const std::size_t count = line.last_index - line.first_index + 1;
    double station_sum = 0.0;
    double elevation_sum = 0.0;
    double squared_station_sum = 0.0;
    double station_elevation_sum = 0.0;
    double squared_elevation_sum = 0.0;
    for (std::size_t index = line.first_index;
         index <= line.last_index; ++index) {
        const double station_m = samples[index].station_m;
        const double elevation_m = samples[index].elevation_m;
        station_sum += station_m;
        elevation_sum += elevation_m;
        squared_station_sum += station_m * station_m;
        station_elevation_sum += station_m * elevation_m;
        squared_elevation_sum += elevation_m * elevation_m;
    }

    const double sample_count = static_cast<double>(count);
    double denominator =
        squared_station_sum * sample_count - station_sum * station_sum;
    double intercept = 0.0;
    if (std::abs(denominator) < 0.000001) {
        denominator =
            squared_elevation_sum * sample_count -
            elevation_sum * elevation_sum;
        if (std::abs(denominator) < 0.000001) {
            throw std::invalid_argument("Vertical grade fit is degenerate");
        }
        const double inverse_grade =
            (sample_count * station_elevation_sum -
             station_sum * elevation_sum) / denominator;
        intercept =
            -(station_sum - inverse_grade * elevation_sum) / sample_count;
        line.grade = 1.0e10;
    } else {
        line.grade =
            (sample_count * station_elevation_sum -
             station_sum * elevation_sum) / denominator;
        intercept =
            (elevation_sum - line.grade * station_sum) / sample_count;
    }
    line.start_station_m = start_station_m;
    line.start_elevation_m = intercept + line.grade * line.start_station_m;
}

void fit_all_grade_lines(
    std::vector<WorkingSample>& samples,
    std::vector<GradeLine>& lines) {
    for (auto& line : lines) {
        fit_grade_line(samples, line);
        refresh_curve_sample_counts(lines);
    }
}

double point_to_line_distance(
    const WorkingSample& sample,
    const GradeLine& line) {
    return std::abs(line_elevation(line, sample.station_m) -
                    sample.elevation_m) /
        std::sqrt(1.0 + line.grade * line.grade);
}

double line_mean_error(
    const std::vector<WorkingSample>& samples,
    const GradeLine& line) {
    const std::size_t count = line.last_index - line.first_index + 1;
    if (count < 3) {
        return 0.0;
    }
    double squared_distance_sum = 0.0;
    for (std::size_t index = line.first_index;
         index <= line.last_index; ++index) {
        const double distance = point_to_line_distance(samples[index], line);
        squared_distance_sum += distance * distance;
    }
    return std::sqrt(
        squared_distance_sum / static_cast<double>(count - 2));
}

bool sample_fits_line_mean_error(
    const std::vector<WorkingSample>& samples,
    std::size_t sample_index,
    const GradeLine& line) {
    return point_to_line_distance(samples[sample_index], line) <
        2.0 * line_mean_error(samples, line);
}

void absorb_line_samples_by_mean_error(
    std::vector<WorkingSample>& samples,
    std::vector<GradeLine>& lines) {
    for (std::ptrdiff_t line_index = 0;
         line_index < static_cast<std::ptrdiff_t>(lines.size());
         ++line_index) {
        bool merged_with_previous = false;
        const auto current_index = static_cast<std::size_t>(line_index);
        if (lines[current_index].last_index -
                lines[current_index].first_index <= 1) {
            continue;
        }

        if (line_index > 0 && lines[current_index].first_index > 0) {
            for (std::ptrdiff_t sample_index =
                    static_cast<std::ptrdiff_t>(
                        lines[current_index].first_index) - 1;
                 line_index > 0 &&
                 line_index < static_cast<std::ptrdiff_t>(lines.size()) &&
                 sample_index >= 0 &&
                 sample_index >=
                    static_cast<std::ptrdiff_t>(
                        lines[static_cast<std::size_t>(line_index)]
                            .first_index) -
                    static_cast<std::ptrdiff_t>(
                        lines[static_cast<std::size_t>(line_index - 1)]
                            .next_curve_sample_count);
                 --sample_index) {
                const auto candidate =
                    static_cast<std::size_t>(sample_index);
                const auto line =
                    static_cast<std::size_t>(line_index);
                if (!sample_fits_line_mean_error(
                        samples, candidate, lines[line])) {
                    break;
                }

                samples[candidate].curved = false;
                if (lines[line - 1].next_curve_sample_count <= 1) {
                    fit_grade_line(samples, lines[line]);
                    refresh_curve_sample_counts(lines);
                    const double grade_difference = std::abs(
                        lines[line - 1].grade - lines[line].grade);
                    if (grade_difference < grade_merge_tolerance) {
                        lines[line].first_index =
                            lines[line - 1].first_index;
                        lines.erase(
                            lines.begin() +
                            static_cast<std::ptrdiff_t>(line - 1));
                        --line_index;
                        merged_with_previous = true;
                    } else {
                        break;
                    }
                } else {
                    lines[line].first_index = candidate;
                    --lines[line - 1].next_curve_sample_count;
                }
            }
        }

        if (merged_with_previous) {
            line_index -= 2;
            if (line_index < -1) {
                line_index = -1;
            }
            continue;
        }

        if (line_index < 0 ||
            line_index >= static_cast<std::ptrdiff_t>(lines.size())) {
            continue;
        }
        for (std::ptrdiff_t sample_index =
                static_cast<std::ptrdiff_t>(
                    lines[static_cast<std::size_t>(line_index)]
                        .last_index) +
                1;
             line_index >= 0 &&
             line_index < static_cast<std::ptrdiff_t>(lines.size()) &&
             sample_index < static_cast<std::ptrdiff_t>(samples.size()) &&
             sample_index <=
                static_cast<std::ptrdiff_t>(
                    lines[static_cast<std::size_t>(line_index)]
                        .last_index) +
                lines[static_cast<std::size_t>(line_index)]
                    .next_curve_sample_count;
             ++sample_index) {
            const auto line = static_cast<std::size_t>(line_index);
            if (!sample_fits_line_mean_error(
                    samples,
                    static_cast<std::size_t>(sample_index),
                    lines[line])) {
                break;
            }

            if (lines[line].next_curve_sample_count <= 1 &&
                line + 1 < lines.size()) {
                samples[static_cast<std::size_t>(sample_index)].curved =
                    false;
                fit_grade_line(samples, lines[line]);
                refresh_curve_sample_counts(lines);
                const double grade_difference = std::abs(
                    lines[line + 1].grade - lines[line].grade);
                if (grade_difference < grade_merge_tolerance) {
                    lines[line + 1].first_index =
                        lines[line].first_index;
                    lines.erase(
                        lines.begin() +
                        static_cast<std::ptrdiff_t>(line));
                    --line_index;
                } else {
                    break;
                }
            } else {
                lines[line].last_index =
                    static_cast<std::size_t>(sample_index);
                --lines[line].next_curve_sample_count;
            }
        }
    }
}

bool intersect_grade_lines(
    const GradeLine& incoming,
    const GradeLine& outgoing,
    double& station_m,
    double& elevation_m) {
    const double difference = incoming.grade - outgoing.grade;
    if (std::abs(difference) <= parallel_grade_tolerance) {
        return false;
    }
    const double incoming_intercept =
        incoming.start_elevation_m -
        incoming.grade * incoming.start_station_m;
    const double outgoing_intercept =
        outgoing.start_elevation_m -
        outgoing.grade * outgoing.start_station_m;
    station_m = (outgoing_intercept - incoming_intercept) / difference;
    elevation_m = incoming.grade * station_m + incoming_intercept;
    return std::isfinite(station_m) && std::isfinite(elevation_m);
}

void recalculate_slope_segments(std::vector<SlopePoint>& slopes) {
    for (std::size_t index = 0; index < slopes.size(); ++index) {
        if (index + 1 == slopes.size()) {
            slopes[index].length_m = 0.0;
            slopes[index].grade = 0.0;
            continue;
        }
        const double length =
            slopes[index + 1].station_m - slopes[index].station_m;
        if (!(length > 1.0e-9)) {
            throw std::invalid_argument(
                "Vertical control stations are not strictly increasing");
        }
        slopes[index].length_m = length;
        slopes[index].grade =
            (slopes[index + 1].elevation_m - slopes[index].elevation_m) /
            length;
    }
}

std::vector<SlopePoint> form_slope_points(
    const std::vector<WorkingSample>& samples,
    std::vector<GradeLine>& lines,
    const ReconstructionOptions& options) {
    for (std::size_t index = 1; index < lines.size(); ++index) {
        double station_m = 0.0;
        double elevation_m = 0.0;
        if (intersect_grade_lines(
                lines[index - 1], lines[index], station_m, elevation_m)) {
            const double start_station_m = lines[index - 1].start_station_m;
            const double end_station_m =
                samples[lines[index].last_index].station_m;
            if (station_m > start_station_m + intersection_margin_m &&
                station_m + intersection_margin_m < end_station_m &&
                station_m - start_station_m >
                    options.minimum_grade_length_m &&
                (index + 1 < lines.size() ||
                 samples.back().station_m - station_m > 200.0)) {
                lines[index].start_station_m = station_m;
                lines[index].start_elevation_m = elevation_m;
            }
        }
    }

    std::vector<SlopePoint> slopes;
    slopes.reserve(lines.size() + 1);
    for (std::size_t index = 0; index < lines.size(); ++index) {
        slopes.push_back({
            lines[index].start_station_m,
            lines[index].start_elevation_m,
            index == 0
                ? 0.0
                : options.minimum_vertical_curve_radius_m,
            lines[index].grade,
            0.0
        });
    }

    if (lines.back().start_station_m + intersection_margin_m <
        samples.back().station_m) {
        slopes.push_back({
            samples.back().station_m,
            line_elevation(lines.back(), samples.back().station_m),
            0.0,
            0.0,
            0.0
        });
    }
    if (slopes.size() < 2) {
        throw std::invalid_argument(
            "Vertical reconstruction produced too few control points");
    }

    slopes.front().station_m = samples.front().station_m;
    slopes.front().elevation_m = samples.front().elevation_m;
    for (std::size_t index = 0; index + 1 < slopes.size();) {
        if (std::abs(
                slopes[index + 1].station_m -
                slopes[index].station_m) < intersection_margin_m) {
            slopes.erase(
                slopes.begin() +
                static_cast<std::ptrdiff_t>(index + 1));
        } else {
            ++index;
        }
    }
    recalculate_slope_segments(slopes);
    return slopes;
}

void merge_small_grade_differences(std::vector<SlopePoint>& slopes) {
    for (std::size_t index = 1; index + 1 < slopes.size();) {
        if (std::abs(slopes[index].grade - slopes[index - 1].grade) <
            grade_merge_tolerance) {
            slopes.erase(
                slopes.begin() + static_cast<std::ptrdiff_t>(index));
            recalculate_slope_segments(slopes);
        } else {
            ++index;
        }
    }
}

bool intersect_slope_lines(
    const SlopePoint& first,
    const SlopePoint& second,
    double& station_m,
    double& elevation_m) {
    const double difference = first.grade - second.grade;
    if (std::abs(difference) <= parallel_grade_tolerance) {
        return false;
    }
    const double first_intercept =
        first.elevation_m - first.grade * first.station_m;
    const double second_intercept =
        second.elevation_m - second.grade * second.station_m;
    station_m = (second_intercept - first_intercept) / difference;
    elevation_m = first.grade * station_m + first_intercept;
    return std::isfinite(station_m) && std::isfinite(elevation_m);
}

void round_slope_station(
    SlopePoint& slope,
    double rounding_m) {
    if (!(rounding_m > 0.0)) {
        return;
    }
    const double whole = std::trunc(slope.station_m / rounding_m);
    const double remainder = slope.station_m - whole * rounding_m;
    const double increment = remainder > rounding_m * 0.5 ? 1.0 : 0.0;
    slope.station_m = (whole + increment) * rounding_m;
}

bool merge_one_short_slope(
    std::vector<SlopePoint>& slopes,
    double minimum_length_m,
    bool round_intersection,
    double station_rounding_m) {
    if (!(minimum_length_m > 0.0) || slopes.size() < 4) {
        return true;
    }

    double minimum_lengths[3]{
        minimum_length_m, minimum_length_m, minimum_length_m
    };
    std::size_t candidate_indices[3]{0, 0, 0};
    for (std::size_t index = 1; index < slopes.size(); ++index) {
        const double length =
            slopes[index].station_m - slopes[index - 1].station_m;
        if (length < minimum_lengths[2]) {
            if (length < minimum_lengths[0]) {
                minimum_lengths[2] = minimum_lengths[1];
                candidate_indices[2] = candidate_indices[1];
                minimum_lengths[1] = minimum_lengths[0];
                candidate_indices[1] = candidate_indices[0];
                minimum_lengths[0] = length;
                candidate_indices[0] = index - 1;
            } else if (length < minimum_lengths[1]) {
                minimum_lengths[2] = minimum_lengths[1];
                candidate_indices[2] = candidate_indices[1];
                minimum_lengths[1] = length;
                candidate_indices[1] = index - 1;
            } else {
                minimum_lengths[2] = length;
                candidate_indices[2] = index - 1;
            }
        }
    }
    if (minimum_lengths[0] > minimum_length_m - 0.001) {
        return true;
    }

    std::size_t segment_index = slopes.size();
    std::size_t last_candidate_index = 0;
    for (const std::size_t index : candidate_indices) {
        last_candidate_index = index;
        if (index == 0) {
            if (std::abs(slopes[0].grade - slopes[1].grade) <
                protected_end_grade_difference) {
                slopes.erase(slopes.begin() + 1);
                recalculate_slope_segments(slopes);
                return false;
            }
            continue;
        }
        if (index + 2 == slopes.size()) {
            if (std::abs(slopes[index - 1].grade - slopes[index].grade) <
                protected_end_grade_difference) {
                slopes.erase(
                    slopes.begin() + static_cast<std::ptrdiff_t>(index));
                recalculate_slope_segments(slopes);
                return false;
            }
            continue;
        }
        segment_index = index;
        break;
    }
    if (segment_index == slopes.size()) {
        return last_candidate_index != 0 &&
            last_candidate_index != slopes.size() - 2;
    }

    double station_m = 0.0;
    double elevation_m = 0.0;
    const bool intersects = intersect_slope_lines(
        slopes[segment_index - 1],
        slopes[segment_index + 1],
        station_m,
        elevation_m);
    if (intersects &&
        station_m > slopes[segment_index].station_m &&
        station_m < slopes[segment_index + 1].station_m) {
        slopes[segment_index].station_m = station_m;
        slopes[segment_index].elevation_m = elevation_m;
        slopes.erase(
            slopes.begin() +
            static_cast<std::ptrdiff_t>(segment_index + 1));
        if (round_intersection) {
            round_slope_station(
                slopes[segment_index], station_rounding_m);
        }
    } else {
        const double difference_before = std::abs(
            slopes[segment_index].grade -
            slopes[segment_index - 1].grade);
        const double difference_after = std::abs(
            slopes[segment_index + 1].grade -
            slopes[segment_index].grade);
        const std::size_t remove_index =
            difference_before < difference_after
                ? segment_index : segment_index + 1;
        slopes.erase(
            slopes.begin() +
            static_cast<std::ptrdiff_t>(remove_index));
    }
    recalculate_slope_segments(slopes);
    return false;
}

enum class ShortSlopePassMode {
    fixed_initial_size,
    dynamic_size
};

void merge_short_slopes(
    std::vector<SlopePoint>& slopes,
    const ReconstructionOptions& options,
    bool round_intersection,
    ShortSlopePassMode pass_mode) {
    const std::size_t initial_pass_count = slopes.size();
    for (std::size_t pass = 0;
         pass < (pass_mode == ShortSlopePassMode::dynamic_size
             ? slopes.size() : initial_pass_count);
         ++pass) {
        if (merge_one_short_slope(
                slopes,
                options.minimum_grade_length_m,
                round_intersection,
                options.station_rounding_m)) {
            break;
        }
        merge_small_grade_differences(slopes);
    }
}

void classify_around_slope_points(
    std::vector<WorkingSample>& samples,
    const std::vector<SlopePoint>& slopes) {
    double previous_curve_end = samples.front().station_m;
    for (std::size_t index = 1; index + 1 < slopes.size(); ++index) {
        const double start =
            slopes[index].station_m - initial_curve_half_window_m;
        const double end =
            slopes[index].station_m + initial_curve_half_window_m;
        for (auto& sample : samples) {
            if (sample.station_m >= previous_curve_end &&
                sample.station_m <= start) {
                sample.curved = false;
            }
            if (sample.station_m >= end) {
                break;
            }
            if (sample.station_m > start) {
                sample.curved = true;
            }
        }
        previous_curve_end = end;
    }
    for (std::size_t index = samples.size(); index-- > 0;) {
        if (samples[index].station_m < previous_curve_end) {
            break;
        }
        samples[index].curved = false;
    }
    samples.front().curved = false;
    samples.back().curved = false;
}

double circular_profile_elevation(
    double station_m,
    const SlopePoint& intersection,
    double incoming_grade,
    double outgoing_grade,
    double radius_m) {
    if (!(radius_m > 0.0)) {
        const double grade = station_m < intersection.station_m
            ? incoming_grade : outgoing_grade;
        return intersection.elevation_m +
            grade * (station_m - intersection.station_m);
    }
    const double incoming_angle = std::atan(incoming_grade);
    const double outgoing_angle = std::atan(outgoing_grade);
    const double signed_deflection = outgoing_angle - incoming_angle;
    if (std::abs(signed_deflection) <= 1.0e-15) {
        return intersection.elevation_m +
            incoming_grade * (station_m - intersection.station_m);
    }
    const int turn_sign = signed_deflection > 0.0 ? 1 : -1;
    const double tangent =
        radius_m * std::tan(0.5 * std::abs(signed_deflection));
    const double start_station =
        intersection.station_m - tangent * std::cos(incoming_angle);
    const double start_elevation =
        intersection.elevation_m - tangent * std::sin(incoming_angle);
    const double center_station =
        start_station -
        static_cast<double>(turn_sign) *
            radius_m * std::sin(incoming_angle);
    const double center_elevation =
        start_elevation +
        static_cast<double>(turn_sign) *
            radius_m * std::cos(incoming_angle);
    const double horizontal = station_m - center_station;
    const double vertical = std::sqrt(std::max(
        0.0, radius_m * radius_m - horizontal * horizontal));
    return center_elevation -
        static_cast<double>(turn_sign) * vertical;
}

double fit_vertical_radius(
    const std::vector<WorkingSample>& samples,
    const std::vector<SlopePoint>& slopes,
    std::size_t slope_index,
    const ReconstructionOptions& options) {
    const double incoming_grade = slopes[slope_index - 1].grade;
    const double outgoing_grade = slopes[slope_index].grade;
    const double incoming_length =
        std::sqrt(1.0 + incoming_grade * incoming_grade);
    const double outgoing_length =
        std::sqrt(1.0 + outgoing_grade * outgoing_grade);
    const double incoming_x = 1.0 / incoming_length;
    const double incoming_y = incoming_grade / incoming_length;
    const double outgoing_x = 1.0 / outgoing_length;
    const double outgoing_y = outgoing_grade / outgoing_length;
    const double difference_x = outgoing_x - incoming_x;
    const double difference_y = outgoing_y - incoming_y;
    const double difference_length =
        std::sqrt(difference_x * difference_x +
                  difference_y * difference_y);
    if (difference_length <= 1.0e-15) {
        return 0.0;
    }

    const double half_angle = 0.5 * std::abs(
        std::atan(outgoing_grade) - std::atan(incoming_grade));
    const double cosine = std::cos(half_angle);
    if (std::abs(cosine) <= 1.0e-15) {
        return 0.0;
    }
    const double center_direction_x =
        difference_x / difference_length / cosine;
    const double center_direction_y =
        difference_y / difference_length / cosine;
    const double denominator_factor =
        center_direction_x * center_direction_x +
        center_direction_y * center_direction_y - 1.0;
    if (std::abs(denominator_factor) <= 1.0e-18) {
        return options.minimum_vertical_curve_radius_m;
    }

    double radius = options.minimum_vertical_curve_radius_m;
    for (int iteration = 0; iteration < 100; ++iteration) {
        const double previous_radius = radius;
        const double tangent =
            previous_radius * std::tan(half_angle);
        const double start_station =
            slopes[slope_index].station_m - tangent;
        const double end_station =
            slopes[slope_index].station_m + tangent;
        const auto first = std::upper_bound(
            samples.begin(), samples.end(), start_station,
            [](double station, const WorkingSample& sample) {
                return station < sample.station_m;
            });
        const auto after_last = std::upper_bound(
            samples.begin(), samples.end(), end_station,
            [](double station, const WorkingSample& sample) {
                return station < sample.station_m;
            });
        if (first == after_last) {
            break;
        }

        double station_delta_sum = 0.0;
        double elevation_delta_sum = 0.0;
        for (auto sample = first; sample != after_last; ++sample) {
            station_delta_sum +=
                sample->station_m - slopes[slope_index].station_m;
            elevation_delta_sum +=
                sample->elevation_m - slopes[slope_index].elevation_m;
        }
        const double count =
            static_cast<double>(after_last - first);
        const double fitted =
            (center_direction_x * station_delta_sum +
             center_direction_y * elevation_delta_sum) /
            denominator_factor / count;
        if (!std::isfinite(fitted) || fitted < 0.001) {
            radius = options.minimum_vertical_curve_radius_m;
            break;
        }
        radius = fitted;
        if (std::abs(radius - previous_radius) <= 0.1) {
            break;
        }
    }

    if (options.vertical_curve_radius_rounding_m > 0.0) {
        radius = std::floor(
            radius / options.vertical_curve_radius_rounding_m) *
            options.vertical_curve_radius_rounding_m;
    }
    return std::max(
        options.minimum_vertical_curve_radius_m,
        std::min(
            radius,
            std::min(
                options.maximum_vertical_curve_radius_m,
                active_maximum_vertical_curve_radius_m)));
}

void set_vertical_radii(
    const std::vector<WorkingSample>& samples,
    std::vector<SlopePoint>& slopes,
    const ReconstructionOptions& options) {
    if (slopes.empty()) {
        return;
    }
    slopes.front().vertical_curve_radius_m = 0.0;
    slopes.back().vertical_curve_radius_m = 0.0;
    for (std::size_t index = 1; index + 1 < slopes.size(); ++index) {
        const double grade_difference_per_mille =
            std::abs(slopes[index].grade - slopes[index - 1].grade) *
            1000.0;
        slopes[index].vertical_curve_radius_m =
            grade_difference_per_mille >
                    options.minimum_vertical_curve_grade_difference_per_mille
                ? fit_vertical_radius(samples, slopes, index, options)
                : 0.0;
    }
}

std::size_t adjust_grade_line_boundaries(
    const std::vector<WorkingSample>& samples,
    const std::vector<SlopePoint>& slopes,
    std::vector<GradeLine>& lines) {
    std::size_t changed_sample_count = 0;
    std::vector<double> curve_starts(slopes.size(), 0.0);
    std::vector<double> curve_ends(slopes.size(), 0.0);
    for (std::size_t index = 0; index < slopes.size(); ++index) {
        if (index == 0 || index + 1 == slopes.size()) {
            curve_starts[index] = slopes[index].station_m;
            curve_ends[index] = slopes[index].station_m;
            continue;
        }
        const double incoming_angle =
            std::atan(slopes[index - 1].grade);
        const double outgoing_angle =
            std::atan(slopes[index].grade);
        const double half_angle =
            0.5 * std::abs(outgoing_angle - incoming_angle);
        const double tangent =
            slopes[index].vertical_curve_radius_m *
            std::tan(half_angle);
        curve_starts[index] =
            slopes[index].station_m -
            tangent * std::cos(std::abs(incoming_angle));
        curve_ends[index] =
            slopes[index].station_m +
            tangent * std::cos(std::abs(outgoing_angle));
    }

    const std::size_t boundary_count = std::min(
        lines.size() > 0 ? lines.size() - 1 : 0,
        slopes.size() > 1 ? slopes.size() - 2 : 0);
    for (std::size_t index = 1; index <= boundary_count; ++index) {
        auto& incoming = lines[index - 1];
        auto& outgoing = lines[index];
        const std::ptrdiff_t expected_curve_sample_count =
            static_cast<std::ptrdiff_t>(outgoing.first_index) -
            static_cast<std::ptrdiff_t>(incoming.last_index) - 1;
        if (incoming.next_curve_sample_count != expected_curve_sample_count) {
            throw std::invalid_argument(
                "Vertical grade line curve count drift before slope " +
                std::to_string(index) +
                ": incoming_last=" + std::to_string(incoming.last_index) +
                ", outgoing_first=" + std::to_string(outgoing.first_index) +
                ", stored_curve_count=" +
                    std::to_string(incoming.next_curve_sample_count) +
                ", expected_curve_count=" +
                    std::to_string(expected_curve_sample_count));
        }
        double desired_start = curve_starts[index];
        double desired_end = curve_ends[index];
        if (desired_start < curve_ends[index - 1]) {
            desired_start = curve_ends[index - 1];
        }
        if (desired_end > slopes[index + 1].station_m) {
            desired_end = slopes[index + 1].station_m;
        }

        const std::ptrdiff_t curve_start = std::clamp(
            static_cast<std::ptrdiff_t>(incoming.last_index) + 1,
            std::ptrdiff_t{0},
            static_cast<std::ptrdiff_t>(samples.size() - 1));
        const std::ptrdiff_t curve_end = std::clamp(
            curve_start +
                static_cast<std::ptrdiff_t>(
                    incoming.next_curve_sample_count) -
                1,
            std::ptrdiff_t{0},
            static_cast<std::ptrdiff_t>(samples.size() - 1));

        if (samples[static_cast<std::size_t>(curve_start)].station_m >
            desired_start) {
            for (std::ptrdiff_t sample_index = curve_start;
                 sample_index >= 0 &&
                  samples[static_cast<std::size_t>(sample_index)].station_m >
                     desired_start;
                  --sample_index) {
                if (incoming.last_index == 0) {
                    throw std::invalid_argument(
                        "Vertical grade boundary falls before the first sample");
                }
                --incoming.last_index;
                ++incoming.next_curve_sample_count;
                ++changed_sample_count;
            }
        } else if (
            samples[static_cast<std::size_t>(curve_start)].station_m <
            desired_start) {
            for (std::ptrdiff_t sample_index = curve_start;
                 sample_index <
                    static_cast<std::ptrdiff_t>(samples.size()) &&
                    samples[static_cast<std::size_t>(sample_index)].station_m <
                     desired_start;
                  ++sample_index) {
                if (incoming.last_index + 1 >= samples.size()) {
                    throw std::invalid_argument(
                        "Vertical grade start boundary falls after the last sample"
                        " at slope " + std::to_string(index) +
                        ", desired=" + std::to_string(desired_start) +
                        ", curve_start=" + std::to_string(curve_start) +
                        ", incoming_last=" +
                            std::to_string(incoming.last_index) +
                        ", curve_count=" +
                            std::to_string(incoming.next_curve_sample_count));
                }
                ++incoming.last_index;
                --incoming.next_curve_sample_count;
                ++changed_sample_count;
            }
        }

        if (samples[static_cast<std::size_t>(curve_end)].station_m >
            desired_end) {
            for (std::ptrdiff_t sample_index = curve_end;
                 sample_index >= 0 &&
                 samples[static_cast<std::size_t>(sample_index)].station_m >
                    desired_end;
                 --sample_index) {
                if (outgoing.first_index == 0 || sample_index == 0) {
                    throw std::invalid_argument(
                        "Vertical grade boundary falls before the first sample");
                }
                --outgoing.first_index;
                --incoming.next_curve_sample_count;
                ++changed_sample_count;
            }
        } else if (
            samples[static_cast<std::size_t>(curve_end)].station_m <
            desired_end) {
            for (std::ptrdiff_t sample_index = curve_end;
                 sample_index <
                    static_cast<std::ptrdiff_t>(samples.size()) &&
                 samples[static_cast<std::size_t>(sample_index)].station_m <
                    desired_end;
                 ++sample_index) {
                if (outgoing.first_index + 1 >= samples.size()) {
                    throw std::invalid_argument(
                        "Vertical grade end boundary falls after the last sample"
                        " at slope " + std::to_string(index) +
                        ", desired=" + std::to_string(desired_end) +
                        ", curve_start=" + std::to_string(curve_start) +
                        ", curve_end=" + std::to_string(curve_end) +
                        ", incoming_last=" +
                            std::to_string(incoming.last_index) +
                        ", outgoing_first=" +
                            std::to_string(outgoing.first_index) +
                        ", curve_count=" +
                            std::to_string(incoming.next_curve_sample_count));
                }
                ++outgoing.first_index;
                ++incoming.next_curve_sample_count;
                ++changed_sample_count;
            }
        }
    }
    return changed_sample_count;
}

void remove_near_coincident_slope_points(
    std::vector<SlopePoint>& slopes) {
    for (std::size_t index = 1; index < slopes.size(); ++index) {
        if (std::abs(
                slopes[index].station_m -
                slopes[index - 1].station_m) < intersection_margin_m) {
            std::size_t last_coincident = index;
            while (last_coincident + 1 < slopes.size() &&
                   std::abs(
                       slopes[last_coincident + 1].station_m -
                       slopes[last_coincident].station_m) <
                       intersection_margin_m) {
                ++last_coincident;
            }
            slopes.erase(
                slopes.begin() +
                    static_cast<std::ptrdiff_t>(index - 1),
                slopes.begin() +
                    static_cast<std::ptrdiff_t>(last_coincident));
        }
    }
    recalculate_slope_segments(slopes);
}

void adjust_slope_points_for_spiral_overlap(
    std::vector<SlopePoint>& slopes,
    const std::vector<HorizontalSpiralInterval>& horizontal_spirals,
    const ReconstructionOptions& options) {
    if (horizontal_spirals.empty() || slopes.size() < 3) {
        return;
    }
    for (std::size_t index = 1; index + 1 < slopes.size(); ++index) {
        const double vertical_curve_length =
            slopes[index].vertical_curve_radius_m *
            std::abs(slopes[index].grade - slopes[index - 1].grade);
        if (!(vertical_curve_length > 0.001)) {
            continue;
        }
        const double vertical_start =
            slopes[index].station_m - vertical_curve_length * 0.5;
        const double vertical_end =
            slopes[index].station_m + vertical_curve_length * 0.5;

        bool overlaps = false;
        double overlap_start = 0.0;
        double overlap_end = 0.0;
        for (const auto& spiral : horizontal_spirals) {
            const double common_start =
                std::max(vertical_start, spiral.start_station_m);
            const double common_end =
                std::min(vertical_end, spiral.end_station_m);
            if (common_end - common_start > 0.01) {
                overlap_start =
                    spiral.start_station_m - vertical_curve_length * 0.5;
                overlap_end =
                    spiral.end_station_m + vertical_curve_length * 0.5;
                overlaps = true;
                break;
            }
        }
        if (!overlaps) {
            continue;
        }
        if (options.station_rounding_m > 0.0) {
            overlap_start =
                std::floor(overlap_start / options.station_rounding_m) *
                options.station_rounding_m;
            overlap_end =
                std::ceil(overlap_end / options.station_rounding_m) *
                options.station_rounding_m;
        }

        const double distance_before =
            std::abs(slopes[index].station_m - overlap_start);
        const double distance_after =
            std::abs(overlap_end - slopes[index].station_m);
        const double remaining_before =
            slopes[index - 1].length_m - distance_before;
        const double remaining_after =
            slopes[index].length_m - distance_after;
        if (std::min(remaining_before, remaining_after) + 0.0001 >
            options.minimum_grade_length_m) {
            slopes[index].station_m =
                distance_before < distance_after
                    ? overlap_start : overlap_end;
        } else if (remaining_before + 0.001 >
                   options.minimum_grade_length_m) {
            slopes[index].station_m = overlap_start;
        } else if (remaining_after + 0.001 >
                   options.minimum_grade_length_m) {
            slopes[index].station_m = overlap_end;
        } else {
            slopes.erase(
                slopes.begin() + static_cast<std::ptrdiff_t>(index));
            recalculate_slope_segments(slopes);
            --index;
            continue;
        }
        recalculate_slope_segments(slopes);
    }
    recalculate_slope_segments(slopes);
}

double round_grade(double grade, double step_per_mille) {
    if (!(step_per_mille > 0.0)) {
        return grade;
    }
    const double scaled = grade * 1000.0 / step_per_mille;
    const double whole = std::trunc(scaled);
    const double remainder = scaled - whole;
    double increment = 0.0;
    if (remainder < -0.5) {
        increment = -1.0;
    } else if (remainder > 0.5) {
        increment = 1.0;
    }
    return (whole + increment) * step_per_mille / 1000.0;
}

void round_slope_grades(
    std::vector<SlopePoint>& slopes,
    double step_per_mille) {
    for (std::size_t index = 0; index + 1 < slopes.size(); ++index) {
        slopes[index].grade =
            round_grade(slopes[index].grade, step_per_mille);
    }
}

void rebuild_intermediate_elevations(
    std::vector<SlopePoint>& slopes) {
    for (std::size_t index = 1; index + 1 < slopes.size(); ++index) {
        slopes[index].station_m =
            slopes[index - 1].station_m +
            slopes[index - 1].length_m;
        slopes[index].elevation_m =
            slopes[index - 1].elevation_m +
            slopes[index - 1].grade *
                slopes[index - 1].length_m;
    }
}

void refresh_output_elevations(
    std::vector<SlopePoint>& slopes) {
    if (slopes.empty()) {
        return;
    }
    for (std::size_t index = 0; index + 1 < slopes.size(); ++index) {
        slopes[index].length_m =
            slopes[index + 1].station_m - slopes[index].station_m;
        slopes[index + 1].elevation_m =
            slopes[index].elevation_m +
            slopes[index].grade * slopes[index].length_m;
        slopes[index].grade =
            (slopes[index + 1].elevation_m -
             slopes[index].elevation_m) /
            slopes[index].length_m;
    }
    slopes.back().grade = 0.0;
    slopes.back().length_m = 0.0;
}

double truncate_limit_grade_per_mille(
    double grade_per_mille,
    double precision_step_per_mille) {
    if (!(precision_step_per_mille > 0.0)) {
        return grade_per_mille;
    }
    return std::trunc(
        grade_per_mille / precision_step_per_mille + 0.00001) *
        precision_step_per_mille;
}

double signed_grade_limit(
    const SlopePoint& slope,
    const ReconstructionOptions& options) {
    const double limit_per_mille = slope.grade >= 0.0
        ? options.maximum_up_grade
        : options.maximum_down_grade;
    if (!(limit_per_mille > 0.0)) {
        return slope.grade >= 0.0
            ? std::numeric_limits<double>::infinity()
            : -std::numeric_limits<double>::infinity();
    }
    const double truncated = truncate_limit_grade_per_mille(
        limit_per_mille, options.grade_rounding_per_mille) * 0.001;
    return slope.grade >= 0.0 ? truncated : -truncated;
}

void recalculate_slope_segment(
    std::vector<SlopePoint>& slopes,
    std::size_t index) {
    if (index + 1 >= slopes.size()) {
        return;
    }
    const double length =
        slopes[index + 1].station_m - slopes[index].station_m;
    if (!(length > 1.0e-9)) {
        throw std::invalid_argument(
            "Vertical control stations are not strictly increasing");
    }
    slopes[index].length_m = length;
    slopes[index].grade =
        (slopes[index + 1].elevation_m - slopes[index].elevation_m) /
        length;
}

void apply_grade_limits(
    std::vector<SlopePoint>& slopes,
    const ReconstructionOptions& options) {
    if (slopes.size() < 2 ||
        (!(options.maximum_up_grade > 0.0) &&
         !(options.maximum_down_grade > 0.0))) {
        return;
    }
    for (std::size_t index = 0; index + 1 < slopes.size(); ++index) {
        const double limit = signed_grade_limit(slopes[index], options);
        if (std::abs(slopes[index].grade) <= std::abs(limit)) {
            continue;
        }
        slopes[index].grade = limit;
        slopes[index + 1].elevation_m =
            slopes[index].elevation_m +
            limit * slopes[index].length_m;
        recalculate_slope_segment(slopes, index + 1);
    }
}

struct GradeLimits {
    double down = -std::numeric_limits<double>::infinity();
    double up = std::numeric_limits<double>::infinity();
};

GradeLimits grade_limits_for_segment(
    const SlopePoint& slope,
    const ReconstructionOptions& options) {
    const double limit_per_mille = slope.grade >= 0.0
        ? options.maximum_up_grade
        : options.maximum_down_grade;
    if (!(limit_per_mille > 0.0)) {
        return {};
    }
    const double limit = truncate_limit_grade_per_mille(
        limit_per_mille, options.grade_rounding_per_mille) * 0.001;
    return {-limit, limit};
}

bool slope_point_elevation_range(
    const std::vector<SlopePoint>& slopes,
    std::size_t index,
    const ReconstructionOptions& options,
    double maximum_difference_per_mille,
    double& minimum_elevation_m,
    double& maximum_elevation_m) {
    const double front_length =
        slopes[index].station_m - slopes[index - 1].station_m;
    const double back_length =
        slopes[index + 1].station_m - slopes[index].station_m;
    if (!(front_length > 0.0) || !(back_length > 0.0)) {
        return false;
    }
    const double difference_limit =
        maximum_difference_per_mille * 0.001;

    const auto front_limits =
        grade_limits_for_segment(slopes[index - 1], options);
    if (index > 1) {
        const double minimum_grade = std::max(
            slopes[index - 2].grade - difference_limit,
            front_limits.down);
        const double maximum_grade = std::min(
            slopes[index - 2].grade + difference_limit,
            front_limits.up);
        if (minimum_grade > maximum_grade + 0.000001) {
            return false;
        }
    }
    const double front_minimum =
        slopes[index - 1].elevation_m +
        front_limits.down * front_length;
    const double front_maximum =
        slopes[index - 1].elevation_m +
        front_limits.up * front_length;

    const auto back_limits =
        grade_limits_for_segment(slopes[index], options);
    const double minimum_back_grade = std::max(
        slopes[index + 1].grade - difference_limit,
        back_limits.down);
    const double maximum_back_grade = std::min(
        slopes[index + 1].grade + difference_limit,
        back_limits.up);
    if (minimum_back_grade > maximum_back_grade + 0.000001) {
        return false;
    }
    const double back_maximum =
        slopes[index + 1].elevation_m -
        back_limits.down * back_length;
    const double back_minimum =
        slopes[index + 1].elevation_m -
        back_limits.up * back_length;

    minimum_elevation_m = std::max(front_minimum, back_minimum);
    maximum_elevation_m = std::min(front_maximum, back_maximum);
    if (maximum_elevation_m < minimum_elevation_m + 0.0001) {
        return false;
    }

    const double own_maximum =
        (difference_limit * front_length * back_length +
         back_length * slopes[index - 1].elevation_m +
         front_length * slopes[index + 1].elevation_m) /
        (front_length + back_length);
    const double own_minimum =
        (-difference_limit * front_length * back_length +
         back_length * slopes[index - 1].elevation_m +
         front_length * slopes[index + 1].elevation_m) /
        (front_length + back_length);
    maximum_elevation_m =
        std::min(maximum_elevation_m, own_maximum);
    minimum_elevation_m =
        std::max(minimum_elevation_m, own_minimum);
    return maximum_elevation_m >= minimum_elevation_m + 0.0001;
}

void apply_grade_difference_limit(
    std::vector<SlopePoint>& slopes,
    const ReconstructionOptions& options) {
    if (options.maximum_grade_difference_per_mille < 0.1 ||
        slopes.size() < 3) {
        return;
    }

    for (std::size_t index = 1; index + 1 < slopes.size(); ++index) {
        recalculate_slope_segment(slopes, index - 1);
        recalculate_slope_segment(slopes, index);
        const double grade_difference_per_mille =
            1000.0 *
            std::abs(slopes[index].grade - slopes[index - 1].grade);
        if (grade_difference_per_mille <=
            options.maximum_grade_difference_per_mille) {
            continue;
        }

        double minimum_elevation_m = 0.0;
        double maximum_elevation_m = 0.0;
        if (!slope_point_elevation_range(
                slopes,
                index,
                options,
                options.maximum_grade_difference_per_mille,
                minimum_elevation_m,
                maximum_elevation_m)) {
            continue;
        }
        slopes[index].elevation_m =
            std::abs(slopes[index].elevation_m - minimum_elevation_m) >
                    std::abs(slopes[index].elevation_m - maximum_elevation_m)
                ? maximum_elevation_m : minimum_elevation_m;
        recalculate_slope_segment(slopes, index - 1);
        recalculate_slope_segment(slopes, index);
    }
}

std::vector<VerticalControlPoint> to_public_controls(
    const std::vector<SlopePoint>& slopes) {
    std::vector<VerticalControlPoint> controls;
    controls.reserve(slopes.size());
    for (const auto& slope : slopes) {
        controls.push_back({
            slope.station_m,
            slope.elevation_m,
            slope.vertical_curve_radius_m
        });
    }
    return controls;
}

ReconstructionCandidate reconstruct_for_threshold(
    const std::vector<ProfileSample>& input_samples,
    double threshold,
    const ReconstructionOptions& options,
    const std::vector<HorizontalSpiralInterval>& horizontal_spirals) {
    const auto analysis =
        classify_profile_samples(input_samples, threshold);
    std::vector<WorkingSample> samples;
    samples.reserve(input_samples.size());
    for (std::size_t index = 0; index < input_samples.size(); ++index) {
        samples.push_back({
            input_samples[index].station_m,
            input_samples[index].elevation_m,
            analysis.curved[index]
        });
    }

    auto lines = form_grade_lines(samples);
    fit_all_grade_lines(samples, lines);
    absorb_line_samples_by_mean_error(samples, lines);
    auto slopes = form_slope_points(
        samples,
        lines,
        options);
    merge_small_grade_differences(slopes);
    merge_short_slopes(
        slopes, options, false, ShortSlopePassMode::fixed_initial_size);

    classify_around_slope_points(samples, slopes);
    lines = form_grade_lines(samples);
    for (std::size_t index = 0; index < lines.size(); ++index) {
        fit_grade_line(samples, lines[index]);
        refresh_curve_sample_counts(lines);
    }
    set_vertical_radii(samples, slopes, options);

    for (std::size_t iteration = 0; iteration < 151; ++iteration) {
        const std::size_t changed =
            adjust_grade_line_boundaries(samples, slopes, lines);
        fit_all_grade_lines(samples, lines);
        slopes = form_slope_points(
            samples,
            lines,
            options);
        set_vertical_radii(samples, slopes, options);
        if (changed < 300) {
            break;
        }
    }

    merge_small_grade_differences(slopes);
    merge_short_slopes(
        slopes, options, false, ShortSlopePassMode::dynamic_size);
    for (std::size_t index = 1; index + 1 < slopes.size(); ++index) {
        round_slope_station(
            slopes[index], options.station_rounding_m);
    }
    remove_near_coincident_slope_points(slopes);
    merge_short_slopes(
        slopes, options, true, ShortSlopePassMode::fixed_initial_size);
    adjust_slope_points_for_spiral_overlap(
        slopes, horizontal_spirals, options);
    round_slope_grades(slopes, options.grade_rounding_per_mille);
    rebuild_intermediate_elevations(slopes);
    apply_grade_limits(slopes, options);
    apply_grade_difference_limit(slopes, options);
    set_vertical_radii(samples, slopes, options);

    ReconstructionCandidate candidate;
    candidate.slopes = std::move(slopes);
    auto output_slopes = candidate.slopes;
    refresh_output_elevations(output_slopes);
    candidate.controls = to_public_controls(output_slopes);
    const VerticalAlignment closure(candidate.controls);
    (void)closure;
    return candidate;
}

double evaluate_candidate(
    const std::vector<SlopePoint>& slopes,
    double station_m) {
    for (std::size_t index = 1; index + 1 < slopes.size(); ++index) {
        const double incoming_angle =
            std::atan(slopes[index - 1].grade);
        const double outgoing_angle =
            std::atan(slopes[index].grade);
        const double tangent =
            slopes[index].vertical_curve_radius_m *
            std::tan(0.5 * std::abs(outgoing_angle - incoming_angle));
        const double start =
            slopes[index].station_m -
            tangent * std::cos(incoming_angle);
        const double end =
            slopes[index].station_m +
            tangent * std::cos(outgoing_angle);
        if (station_m < start) {
            return slopes[index].elevation_m +
                slopes[index - 1].grade *
                    (station_m - slopes[index].station_m);
        }
        if (station_m <= end) {
            return circular_profile_elevation(
                station_m,
                slopes[index],
                slopes[index - 1].grade,
                slopes[index].grade,
                slopes[index].vertical_curve_radius_m);
        }
    }
    const std::size_t last_segment = slopes.size() - 2;
    return slopes[last_segment].elevation_m +
        slopes[last_segment].grade *
            (station_m - slopes[last_segment].station_m);
}

long long legacy_centimeter_value(double elevation_m) {
    const double absolute = std::abs(elevation_m);
    long long centimeters =
        static_cast<long long>(absolute * 100.0);
    const long long millimeters =
        static_cast<long long>(absolute * 1000.0);
    if (millimeters % 10 > 4) {
        ++centimeters;
    }
    return centimeters;
}

long double reconstruction_score(
    const std::vector<ProfileSample>& samples,
    const ReconstructionCandidate& candidate) {
    long double squared_centimeter_difference_sum = 0.0L;
    for (const auto& sample : samples) {
        const long long reconstructed = legacy_centimeter_value(
            evaluate_candidate(candidate.slopes, sample.station_m));
        const long long measured =
            legacy_centimeter_value(sample.elevation_m);
        const long double difference =
            static_cast<long double>(reconstructed - measured);
        squared_centimeter_difference_sum += difference * difference;
    }
    return squared_centimeter_difference_sum;
}

} // namespace

VerticalReconstructor::VerticalReconstructor(
    ReconstructionOptions options,
    std::vector<HorizontalSpiralInterval> horizontal_spirals)
    : options_(std::move(options)),
      horizontal_spirals_(std::move(horizontal_spirals)) {}

std::vector<VerticalControlPoint> VerticalReconstructor::reconstruct(
    const std::vector<ProfileSample>& ordered_samples) const {
    if (ordered_samples.size() < 2) {
        throw std::invalid_argument(
            "At least two vertical samples are required");
    }
    std::vector<double> thresholds =
        options_.vertical_second_difference_threshold_candidates;
    if (thresholds.empty()) {
        thresholds = {0.001};
    }

    std::vector<VerticalControlPoint> best;
    long double best_score =
        std::numeric_limits<long double>::infinity();
    std::string last_failure;
    for (const double threshold : thresholds) {
        try {
            auto candidate = reconstruct_for_threshold(
                ordered_samples, threshold, options_, horizontal_spirals_);
            const long double score =
                reconstruction_score(ordered_samples, candidate);
            if (score <= best_score) {
                best_score = score;
                best = std::move(candidate.controls);
            }
        } catch (const std::invalid_argument& error) {
            last_failure = error.what();
            continue;
        }
    }
    if (best.empty()) {
        throw std::invalid_argument(
            "No vertical classification threshold produced feasible geometry: " +
            last_failure);
    }
    return best;
}

} // namespace alignment_reconstruct::detail
