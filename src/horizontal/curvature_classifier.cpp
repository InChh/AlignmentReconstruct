#include "horizontal/curvature_classifier.h"

#include "geometry/circle_fit.h"
#include "geometry/legacy_line_fit.h"
#include "geometry/vector_math.h"
#include "horizontal/legacy_horizontal_alignment.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace alignment_reconstruct::detail {
namespace {

struct LegacyStraightSegment {
    int first_index = 0;
    int last_index = 0;
    int following_curve_sample_count = 0;
    int circular_first_index = 0;
    int circular_last_index = 0;
    int circular_sample_count = 0;
};

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

void capture_line_curve_graph(
    HorizontalSampleAnalysis& analysis,
    const std::vector<int>& property) {
    analysis.curvature_graph.clear();
    analysis.curvature_graph.reserve(property.size());
    for (std::size_t index = 0; index < property.size(); ++index) {
        analysis.curvature_graph.push_back({
            analysis.curvature_inverse_m[index],
            property[index] == 0
                ? HorizontalSampleProperty::straight
                : HorizontalSampleProperty::curve
        });
    }
}

void finalize_legacy_analysis(
    HorizontalSampleAnalysis& analysis,
    const std::vector<int>& property,
    const std::vector<LegacyStraightSegment>& segments) {
    analysis.sample_kind.clear();
    analysis.sample_kind.reserve(property.size());
    for (std::size_t index = 0; index < property.size(); ++index) {
        analysis.curved[index] = property[index] != 0;
        analysis.sample_kind.push_back(
            static_cast<HorizontalSampleKind>(property[index]));
    }
    analysis.straight_segments.clear();
    analysis.straight_segments.reserve(segments.size());
    for (const LegacyStraightSegment& segment : segments) {
        analysis.straight_segments.push_back({
            static_cast<std::size_t>(segment.first_index),
            static_cast<std::size_t>(segment.last_index),
            static_cast<std::size_t>(segment.following_curve_sample_count),
            static_cast<std::size_t>(segment.circular_first_index),
            static_cast<std::size_t>(segment.circular_last_index),
            static_cast<std::size_t>(segment.circular_sample_count)
        });
    }
    analysis.runs = form_runs(analysis.curved);
}

std::vector<LegacyStraightSegment> form_legacy_straight_segments(
    std::vector<int>& property) {
    std::vector<LegacyStraightSegment> segments;
    if (property.empty()) {
        return segments;
    }

    const int count = static_cast<int>(property.size());
    int first_straight_index = 0;
    for (int index = 0; index < count; ++index) {
        if (property[static_cast<std::size_t>(index)] == 0) {
            first_straight_index = index;
            break;
        }
    }

    LegacyStraightSegment segment;
    segment.first_index = first_straight_index;
    if (first_straight_index != 0) {
        property[0] = static_cast<int>(HorizontalSampleKind::straight);
        property[1] = static_cast<int>(HorizontalSampleKind::straight);
        first_straight_index = 0;
        segment.first_index = 0;
    }
    if (property.size() > 4) {
        property[property.size() - 1] =
            static_cast<int>(HorizontalSampleKind::straight);
        property[property.size() - 2] =
            static_cast<int>(HorizontalSampleKind::straight);
    }
    for (int index = first_straight_index; index < count; ++index) {
        if (property[static_cast<std::size_t>(index)] != 0) {
            segment.last_index = index - 1;
            int next_straight_index = index + 1;
            for (; next_straight_index < count; ++next_straight_index) {
                if (property[static_cast<std::size_t>(next_straight_index)] == 0) {
                    break;
                }
            }
            segment.following_curve_sample_count = next_straight_index - index;
            segments.push_back(segment);
            segment.first_index = next_straight_index;
            index = next_straight_index;
        }
    }
    if (segment.first_index != count) {
        segment.last_index = count - 1;
        segment.following_curve_sample_count = 0;
        segments.push_back(segment);
    }
    return segments;
}

bool legacy_intersection_is_between(
    const std::vector<Point2D>& points,
    const LegacyStraightSegment& first,
    const LegacyStraightSegment& second) {
    const std::vector<Point2D> first_points(
        points.begin() + first.first_index,
        points.begin() + first.last_index + 1);
    const std::vector<Point2D> second_points(
        points.begin() + second.first_index,
        points.begin() + second.last_index + 1);
    const LegacyLineFitResult first_line =
        fit_legacy_reconstruction_line(first_points);
    const LegacyLineFitResult second_line =
        fit_legacy_reconstruction_line(second_points);
    if (!first_line.valid || !second_line.valid) {
        return false;
    }

    Point2D intersection{};
    if (!intersect_legacy_lines(first_line, second_line, intersection)) {
        return false;
    }
    return (intersection.x - first_line.end.x) *
            (intersection.x - first_line.start.x) >= 0.0 &&
        (intersection.x - second_line.start.x) *
            (intersection.x - second_line.end.x) >= 0.0;
}

void stabilize_legacy_line_curve_properties(
    const std::vector<Point2D>& points,
    const std::vector<double>& tangent_heading_rad,
    std::vector<double>& curvature_inverse_m,
    std::vector<int>& property,
    bool clear_reclassified_curvature) {
    std::vector<LegacyStraightSegment> segments = form_legacy_straight_segments(property);
    int current_change_count = 0;
    int previous_change_count = 0;

    do {
        previous_change_count = current_change_count;
        current_change_count = 0;
        const int curve_count = static_cast<int>(segments.size()) - 1;
        for (int index = 0; index < curve_count; ++index) {
            const LegacyStraightSegment& segment = segments[static_cast<std::size_t>(index)];
            if (index == 0 || segment.last_index - segment.first_index != 0) {
                continue;
            }
            const LegacyStraightSegment& previous = segments[static_cast<std::size_t>(index - 1)];
            if (segment.following_curve_sample_count < 3 &&
                previous.following_curve_sample_count < 3) {
                continue;
            }

            const int sample_index = segment.first_index;
            const int before_index = sample_index - previous.following_curve_sample_count;
            const int after_index = sample_index + segment.following_curve_sample_count;
            if (before_index < 0 || after_index >= static_cast<int>(property.size())) {
                continue;
            }
            const double before_turn = normalize_signed_angle(
                tangent_heading_rad[static_cast<std::size_t>(sample_index)] -
                tangent_heading_rad[static_cast<std::size_t>(before_index)]);
            const double after_turn = normalize_signed_angle(
                tangent_heading_rad[static_cast<std::size_t>(after_index)] -
                tangent_heading_rad[static_cast<std::size_t>(sample_index)]);
            if (before_turn * after_turn > 0.0) {
                property[static_cast<std::size_t>(sample_index)] = 1;
            } else {
                if (sample_index - 1 >= 0) {
                    property[static_cast<std::size_t>(sample_index - 1)] = 0;
                    if (clear_reclassified_curvature) {
                        curvature_inverse_m[static_cast<std::size_t>(sample_index - 1)] = 0.0;
                    }
                }
                if (sample_index < static_cast<int>(segments.size()) - 1 &&
                    sample_index + 1 < static_cast<int>(property.size())) {
                    property[static_cast<std::size_t>(sample_index + 1)] = 0;
                    if (clear_reclassified_curvature) {
                        curvature_inverse_m[static_cast<std::size_t>(sample_index + 1)] = 0.0;
                    }
                }
            }
            ++current_change_count;
        }
        segments = form_legacy_straight_segments(property);
    } while (previous_change_count != current_change_count);

    do {
        previous_change_count = current_change_count;
        current_change_count = 0;
        const int curve_count = static_cast<int>(segments.size()) - 1;
        for (int index = 0; index < curve_count; ++index) {
            const LegacyStraightSegment& segment = segments[static_cast<std::size_t>(index)];
            if (segment.following_curve_sample_count <= 3 &&
                segment.last_index + segment.following_curve_sample_count + 1 <
                    static_cast<int>(property.size())) {
                const LegacyStraightSegment& next = segments[static_cast<std::size_t>(index + 1)];
                for (int sample = segment.last_index + 1;
                     sample < next.first_index; ++sample) {
                    property[static_cast<std::size_t>(sample)] = 0;
                    if (clear_reclassified_curvature) {
                        curvature_inverse_m[static_cast<std::size_t>(sample)] = 0.0;
                    }
                    ++current_change_count;
                }
            }
        }
        segments = form_legacy_straight_segments(property);
    } while (previous_change_count != current_change_count);

    for (int index = 0; index + 1 < static_cast<int>(segments.size()); ++index) {
        const LegacyStraightSegment& first = segments[static_cast<std::size_t>(index)];
        const LegacyStraightSegment& second = segments[static_cast<std::size_t>(index + 1)];
        if (first.last_index - first.first_index <= 1 ||
            second.last_index - second.first_index <= 1 ||
            legacy_intersection_is_between(points, first, second)) {
            continue;
        }
        for (int offset = 1; offset <= first.following_curve_sample_count; ++offset) {
            property[static_cast<std::size_t>(first.last_index + offset)] = 0;
        }
        LegacyStraightSegment merged = second;
        merged.first_index = first.first_index;
        segments[static_cast<std::size_t>(index + 1)] = merged;
        segments.erase(segments.begin() + index);
        // The legacy loop subtracts two so its increment rechecks the previous
        // pair. Clamp the first pair to -1 to preserve that intent safely.
        index = std::max(-1, index - 2);
    }
}

double average_straight_curvature(
    const std::vector<double>& curvature_inverse_m,
    const std::vector<LegacyStraightSegment>& segments) {
    double sum = 0.0;
    std::size_t count = 0;
    for (const LegacyStraightSegment& segment : segments) {
        for (int sample = segment.first_index; sample <= segment.last_index; ++sample) {
            sum += std::abs(curvature_inverse_m[static_cast<std::size_t>(sample)]);
            ++count;
        }
    }
    return count == 0 ? 0.0 : sum / static_cast<double>(count);
}

bool adjust_properties_by_curvature(
    const std::vector<double>& curvature_inverse_m,
    double threshold_inverse_m,
    std::vector<int>& property,
    int& changed_count) {
    changed_count = 0;
    for (std::size_t index = 0; index < property.size(); ++index) {
        const int adjusted = std::abs(curvature_inverse_m[index]) < threshold_inverse_m
            ? static_cast<int>(HorizontalSampleKind::straight)
            : static_cast<int>(HorizontalSampleKind::circular);
        if (property[index] != adjusted) {
            ++changed_count;
        }
        property[index] = adjusted;
    }
    return changed_count != 0;
}

void dynamically_adjust_line_curve_properties(
    const std::vector<Point2D>& points,
    const std::vector<double>& tangent_heading_rad,
    std::vector<double>& curvature_inverse_m,
    int threshold_multiplier,
    std::vector<int>& property,
    std::vector<LegacyStraightSegment>& segments) {
    int iteration = 0;
    bool adjusted = true;
    int changed_count = 0;
    int previous_changed_count = 0;
    while (adjusted && iteration < 20) {
        const double average = average_straight_curvature(
            curvature_inverse_m, segments);
        adjusted = adjust_properties_by_curvature(
            curvature_inverse_m,
            average * static_cast<double>(threshold_multiplier),
            property,
            changed_count);
        stabilize_legacy_line_curve_properties(
            points, tangent_heading_rad, curvature_inverse_m, property, false);
        segments = form_legacy_straight_segments(property);
        ++iteration;
        if (changed_count == previous_changed_count) {
            break;
        }
        previous_changed_count = changed_count;
    }
}

void split_reverse_curves(
    const std::vector<double>& curvature_inverse_m,
    std::vector<int>& property,
    std::vector<LegacyStraightSegment>& segments) {
    std::vector<std::size_t> reverse_curve_indices;
    for (std::size_t curve_index = 0; curve_index + 1 < segments.size();
         ++curve_index) {
        const int first = segments[curve_index].last_index + 1;
        const int last_exclusive = segments[curve_index + 1].first_index;
        if (first >= last_exclusive) {
            continue;
        }
        const bool positive = curvature_inverse_m[static_cast<std::size_t>(first)] > 0.0;
        for (int sample = first + 1; sample < last_exclusive; ++sample) {
            if ((curvature_inverse_m[static_cast<std::size_t>(sample)] > 0.0) != positive) {
                reverse_curve_indices.push_back(curve_index);
                break;
            }
        }
    }
    for (const std::size_t curve_index : reverse_curve_indices) {
        const int first = segments[curve_index].last_index;
        const int last_exclusive = segments[curve_index + 1].first_index;
        for (int sample = first; sample < last_exclusive; ++sample) {
            property[static_cast<std::size_t>(sample)] =
                std::abs(curvature_inverse_m[static_cast<std::size_t>(sample)]) < 0.0008
                ? static_cast<int>(HorizontalSampleKind::straight)
                : static_cast<int>(HorizontalSampleKind::circular);
        }
    }
    if (!reverse_curve_indices.empty()) {
        segments = form_legacy_straight_segments(property);
    }
}

void remove_short_curve_ranges(
    const std::vector<double>& cumulative_distance_m,
    std::vector<int>& property,
    std::vector<LegacyStraightSegment>& segments) {
    for (std::size_t curve_index = 0; curve_index + 1 < segments.size();
         ++curve_index) {
        const LegacyStraightSegment& incoming = segments[curve_index];
        const LegacyStraightSegment& outgoing = segments[curve_index + 1];
        const double begin = 0.5 * (
            cumulative_distance_m[static_cast<std::size_t>(incoming.last_index)] +
            cumulative_distance_m[static_cast<std::size_t>(incoming.last_index + 1)]);
        const double end = 0.5 * (
            cumulative_distance_m[static_cast<std::size_t>(outgoing.first_index)] +
            cumulative_distance_m[static_cast<std::size_t>(outgoing.first_index - 1)]);
        if (end - begin >= 20.0) {
            continue;
        }
        for (int sample = incoming.last_index + 1;
             sample < outgoing.first_index; ++sample) {
            property[static_cast<std::size_t>(sample)] =
                static_cast<int>(HorizontalSampleKind::straight);
        }
    }
    segments = form_legacy_straight_segments(property);
}

std::vector<LegacyStraightSegment> form_legacy_spiral_segments(
    std::vector<int>& property) {
    std::vector<LegacyStraightSegment> segments =
        form_legacy_straight_segments(property);
    for (std::size_t index = 0; index < segments.size(); ++index) {
        LegacyStraightSegment& segment = segments[index];
        if (index + 1 == segments.size()) {
            segment.circular_first_index = segment.last_index;
            segment.circular_last_index = segment.last_index;
            segment.circular_sample_count = 0;
            continue;
        }

        const int search_end = segments[index + 1].first_index;
        segment.circular_first_index = segment.first_index;
        segment.circular_last_index = segment.first_index - 1;
        int circular_search_start = segment.first_index;
        bool found_circular = false;
        for (int sample = segment.first_index; sample < search_end; ++sample) {
            if (property[static_cast<std::size_t>(sample)] ==
                static_cast<int>(HorizontalSampleKind::circular)) {
                segment.circular_first_index = sample;
                circular_search_start = sample;
                segment.circular_last_index = search_end;
                found_circular = true;
                break;
            }
        }
        if (found_circular) {
            for (int sample = circular_search_start; sample <= search_end; ++sample) {
                if (property[static_cast<std::size_t>(sample)] !=
                    static_cast<int>(HorizontalSampleKind::circular)) {
                    segment.circular_last_index = sample - 1;
                    break;
                }
            }
        }
        segment.circular_sample_count =
            segment.circular_last_index - segment.circular_first_index + 1;
    }
    return segments;
}

struct LegacyTangentPair {
    LegacyLineFitResult incoming_line{};
    LegacyLineFitResult outgoing_line{};
    Point2D incoming_start{};
    Point2D incoming_end{};
    Point2D outgoing_start{};
    Point2D outgoing_end{};
    Point2D intersection{};
    bool valid = false;
};

LegacyTangentPair fit_legacy_tangent_pair(
    const std::vector<Point2D>& points,
    const LegacyStraightSegment& incoming,
    const LegacyStraightSegment& outgoing) {
    LegacyTangentPair result;
    const std::vector<Point2D> incoming_points(
        points.begin() + incoming.first_index,
        points.begin() + incoming.last_index + 1);
    const std::vector<Point2D> outgoing_points(
        points.begin() + outgoing.first_index,
        points.begin() + outgoing.last_index + 1);
    result.incoming_line = fit_legacy_reconstruction_line(incoming_points);
    result.outgoing_line = fit_legacy_reconstruction_line(outgoing_points);
    if (!result.incoming_line.valid || !result.outgoing_line.valid) {
        return result;
    }

    result.incoming_start = result.incoming_line.start;
    result.incoming_end = result.incoming_line.end;
    result.outgoing_start = result.outgoing_line.start;
    result.outgoing_end = result.outgoing_line.end;
    if (!intersect_legacy_lines(
            result.incoming_line, result.outgoing_line, result.intersection)) {
        return result;
    }
    result.valid =
        (result.intersection.x - result.incoming_end.x) *
            (result.intersection.x - result.incoming_start.x) > 0.0 &&
        (result.intersection.x - result.outgoing_start.x) *
            (result.intersection.x - result.outgoing_end.x) > 0.0;
    return result;
}

Point2D perpendicular_foot(
    const Point2D& point,
    const LegacyLineFitResult& line) {
    return project_to_legacy_line(point, line);
}

double signed_distance_to_line(
    const Point2D& point,
    const Point2D& line_start,
    const Point2D& line_end) {
    LegacyLineFitResult line;
    line.start = line_start;
    line.end = line_end;
    if (distance(line_start, line_end) <= 1.0e-12) {
        throw std::invalid_argument("Legacy spiral boundary line is degenerate");
    }
    line.valid = true;
    return legacy_signed_distance_to_line(point, line);
}

bool has_minimum_circular_samples(
    const std::vector<int>& property,
    const std::vector<LegacyStraightSegment>& segments,
    std::size_t curve_index,
    std::size_t minimum_count) {
    int circular_count = 0;
    for (int sample = segments[curve_index].first_index;
         sample < segments[curve_index + 1].first_index; ++sample) {
        if (property[static_cast<std::size_t>(sample)] ==
            static_cast<int>(HorizontalSampleKind::circular)) {
            ++circular_count;
        }
    }
    return circular_count >= static_cast<int>(minimum_count);
}

bool at_least_two_line_samples(
    const std::vector<int>& property,
    const std::vector<LegacyStraightSegment>& segments,
    std::size_t segment_index) {
    const LegacyStraightSegment& segment = segments[segment_index];
    int line_count = 0;
    for (int sample = segment.first_index; sample < segment.last_index; ++sample) {
        if (property[static_cast<std::size_t>(sample)] ==
            static_cast<int>(HorizontalSampleKind::straight)) {
            ++line_count;
        }
    }
    return line_count >= 2 &&
        property[static_cast<std::size_t>(segment.first_index)] ==
            static_cast<int>(HorizontalSampleKind::straight) &&
        property[static_cast<std::size_t>(segment.last_index)] ==
            static_cast<int>(HorizontalSampleKind::straight);
}

std::pair<double, double> estimate_legacy_spiral_lengths(
    const CircleFitResult& circle,
    const LegacyTangentPair& tangents,
    double circular_sample_length_m) {
    const double entry_shift = std::abs(legacy_signed_distance_to_line(
        circle.center, tangents.incoming_line)) -
        circle.radius_m;
    const double exit_shift = std::abs(legacy_signed_distance_to_line(
        circle.center, tangents.outgoing_line)) -
        circle.radius_m;
    double entry_length = entry_shift < 1.0e-6
        ? 0.0
        : std::sqrt(24.0 * circle.radius_m * entry_shift);
    double exit_length = exit_shift < 1.0e-6
        ? 0.0
        : std::sqrt(24.0 * circle.radius_m * exit_shift);
    if (entry_length > 1000.0) {
        entry_length = 0.0;
    }
    if (exit_length > 1000.0) {
        exit_length = 0.0;
    }
    if (entry_length + exit_length > circular_sample_length_m) {
        entry_length = 0.0;
        exit_length = 0.0;
    }
    return {entry_length, exit_length};
}

void identify_legacy_spiral_samples(
    const std::vector<Point2D>& points,
    const std::vector<double>& cumulative_distance_m,
    const ReconstructionOptions& options,
    std::vector<int>& property,
    std::vector<LegacyStraightSegment>& segments) {
    int curve_count = static_cast<int>(segments.size()) - 1;
    if (curve_count <= 0) {
        segments = form_legacy_spiral_segments(property);
        return;
    }
    for (int index = 0; index < curve_count; ++index) {
        LegacyStraightSegment& segment = segments[static_cast<std::size_t>(index)];
        segment.circular_first_index = segment.last_index + 1;
        segment.circular_last_index = segments[static_cast<std::size_t>(index + 1)].first_index - 1;
        segment.circular_sample_count =
            segment.circular_last_index - segment.circular_first_index + 1;
    }

    std::vector<double> entry_lengths(static_cast<std::size_t>(curve_count), 0.0);
    std::vector<double> exit_lengths(static_cast<std::size_t>(curve_count), 0.0);
    const std::size_t minimum_circular_sample_count =
        std::max<std::size_t>(3, options.minimum_curve_sample_count);
    std::size_t iteration = 0;
    bool should_adjust = false;
    do {
        int changed_sample_count = 0;
        should_adjust = false;
        for (int curve_index = 0; curve_index < curve_count; ++curve_index) {
            const std::size_t index = static_cast<std::size_t>(curve_index);
            if (segments[index].circular_sample_count == 0) {
                continue;
            }
            if (segments[index].circular_sample_count <
                static_cast<int>(minimum_circular_sample_count)) {
                throw std::invalid_argument(
                    "Circular sample classification contains too few points");
            }

            const CircleFitResult circle = fit_circle_to_range(
                points,
                static_cast<std::size_t>(segments[index].circular_first_index),
                static_cast<std::size_t>(segments[index].circular_last_index));
            if (!circle.valid) {
                throw std::invalid_argument("Legacy circular sample fit is degenerate");
            }
            const LegacyTangentPair tangents = fit_legacy_tangent_pair(
                points, segments[index], segments[index + 1]);
            if (!tangents.valid) {
                entry_lengths[index] = 0.0;
                exit_lengths[index] = 0.0;
                continue;
            }

            const double previous_entry = iteration > 0 ? entry_lengths[index] : 0.0;
            const double previous_exit = iteration > 0 ? exit_lengths[index] : 0.0;
            const double circular_sample_length =
                cumulative_distance_m[static_cast<std::size_t>(segments[index].circular_last_index)] -
                cumulative_distance_m[static_cast<std::size_t>(segments[index].circular_first_index)];
            const auto lengths = estimate_legacy_spiral_lengths(
                circle, tangents, circular_sample_length);
            entry_lengths[index] = lengths.first;
            exit_lengths[index] = lengths.second;
            if (std::abs(previous_entry - lengths.first) > 5.0 ||
                std::abs(previous_exit - lengths.second) > 5.0) {
                should_adjust = true;
            }

            const double middle_distance = 0.5 * (
                cumulative_distance_m[static_cast<std::size_t>(segments[index].circular_first_index)] +
                cumulative_distance_m[static_cast<std::size_t>(segments[index].circular_last_index)]);
            int middle_index = segments[index].circular_first_index;
            for (int sample = segments[index].circular_first_index;
                 sample <= segments[index].circular_last_index; ++sample) {
                if (cumulative_distance_m[static_cast<std::size_t>(sample)] >= middle_distance) {
                    middle_index = sample;
                    break;
                }
            }

            Point2D foot = perpendicular_foot(circle.center, tangents.incoming_line);
            int side = signed_distance_to_line(
                tangents.intersection, foot, circle.center) > 0.0 ? 1 : -1;
            for (int sample = segments[index].first_index + 2;
                 sample < middle_index; ++sample) {
                const double signed_distance = signed_distance_to_line(
                    points[static_cast<std::size_t>(sample)], foot, circle.center);
                if (std::abs(signed_distance) < 0.5 * lengths.first) {
                    if (property[static_cast<std::size_t>(sample)] !=
                            static_cast<int>(HorizontalSampleKind::spiral) &&
                        has_minimum_circular_samples(
                            property, segments, index,
                            minimum_circular_sample_count) &&
                        at_least_two_line_samples(property, segments, index)) {
                        property[static_cast<std::size_t>(sample)] =
                            static_cast<int>(HorizontalSampleKind::spiral);
                        ++changed_sample_count;
                    }
                } else if (signed_distance * static_cast<double>(side) < 0.0) {
                    if (property[static_cast<std::size_t>(sample)] !=
                            static_cast<int>(HorizontalSampleKind::straight) &&
                        has_minimum_circular_samples(
                            property, segments, index,
                            minimum_circular_sample_count) &&
                        at_least_two_line_samples(property, segments, index)) {
                        property[static_cast<std::size_t>(sample)] =
                            static_cast<int>(HorizontalSampleKind::straight);
                        ++changed_sample_count;
                    }
                } else if (property[static_cast<std::size_t>(sample)] !=
                               static_cast<int>(HorizontalSampleKind::circular) &&
                           at_least_two_line_samples(property, segments, index) &&
                           at_least_two_line_samples(property, segments, index)) {
                    property[static_cast<std::size_t>(sample)] =
                        static_cast<int>(HorizontalSampleKind::circular);
                    ++changed_sample_count;
                }
            }

            foot = perpendicular_foot(circle.center, tangents.outgoing_line);
            side = signed_distance_to_line(
                tangents.intersection, foot, circle.center) > 0.0 ? 1 : -1;
            for (int sample = segments[index + 1].last_index - 2;
                 sample >= middle_index; --sample) {
                const double signed_distance = signed_distance_to_line(
                    points[static_cast<std::size_t>(sample)], foot, circle.center);
                if (std::abs(signed_distance) < 0.5 * lengths.second) {
                    if (property[static_cast<std::size_t>(sample)] !=
                            static_cast<int>(HorizontalSampleKind::spiral) &&
                        has_minimum_circular_samples(
                            property, segments, index,
                            minimum_circular_sample_count) &&
                        at_least_two_line_samples(property, segments, index + 1)) {
                        property[static_cast<std::size_t>(sample)] =
                            static_cast<int>(HorizontalSampleKind::spiral);
                        ++changed_sample_count;
                    }
                } else if (signed_distance * static_cast<double>(side) < 0.0) {
                    if (property[static_cast<std::size_t>(sample)] !=
                            static_cast<int>(HorizontalSampleKind::straight) &&
                        has_minimum_circular_samples(
                            property, segments, index,
                            minimum_circular_sample_count) &&
                        at_least_two_line_samples(property, segments, index + 1)) {
                        property[static_cast<std::size_t>(sample)] =
                            static_cast<int>(HorizontalSampleKind::straight);
                        ++changed_sample_count;
                    }
                } else if (property[static_cast<std::size_t>(sample)] !=
                               static_cast<int>(HorizontalSampleKind::circular) &&
                           at_least_two_line_samples(property, segments, index) &&
                           at_least_two_line_samples(property, segments, index + 1)) {
                    property[static_cast<std::size_t>(sample)] =
                        static_cast<int>(HorizontalSampleKind::circular);
                    ++changed_sample_count;
                }
            }
        }

        segments = form_legacy_spiral_segments(property);
        curve_count = static_cast<int>(segments.size()) - 1;
        entry_lengths.resize(static_cast<std::size_t>(std::max(0, curve_count)), 0.0);
        exit_lengths.resize(static_cast<std::size_t>(std::max(0, curve_count)), 0.0);
        ++iteration;
        if (iteration >= options.maximum_spiral_classification_iterations) {
            should_adjust = false;
        } else if (changed_sample_count == 0) {
            should_adjust = false;
        }
    } while (should_adjust);
}

int resort_legacy_point_properties(
    const std::vector<Point2D>& points,
    const CircleFitResult& circle,
    const LegacyTangentPair& tangents,
    double entry_length_m,
    double exit_length_m,
    std::size_t curve_index,
    std::vector<int>& property,
    std::vector<LegacyStraightSegment>& segments) {
    if (entry_length_m < 0.0 || circle.radius_m < 0.0 || exit_length_m < 0.0) {
        return 0;
    }

    const Point2D incoming_direction = subtract(
        tangents.incoming_end, tangents.incoming_start);
    const Point2D outgoing_direction = subtract(
        tangents.outgoing_end, tangents.outgoing_start);
    const Point2D start = subtract(
        tangents.incoming_start, scale(incoming_direction, 10000.0));
    const Point2D end = add(
        tangents.outgoing_end, scale(outgoing_direction, 10000.0));

    int changed_count = 0;
    try {
        const LegacyHorizontalAlignment alignment(
            start, tangents.intersection, end, circle.radius_m,
            entry_length_m, exit_length_m);
        const int first = segments[curve_index].first_index;
        const int last = segments[curve_index + 1].last_index;
        for (int sample = first; sample <= last; ++sample) {
            const LegacyHorizontalProjection projection =
                alignment.project(points[static_cast<std::size_t>(sample)]);
            int adjusted = property[static_cast<std::size_t>(sample)];
            if (projection.element_type == 1) {
                adjusted = static_cast<int>(HorizontalSampleKind::straight);
            } else if (projection.element_type == 2) {
                adjusted = static_cast<int>(HorizontalSampleKind::circular);
            } else if (projection.element_type == 3 || projection.element_type == 4) {
                adjusted = static_cast<int>(HorizontalSampleKind::spiral);
            }
            if (adjusted != property[static_cast<std::size_t>(sample)]) {
                property[static_cast<std::size_t>(sample)] = adjusted;
                ++changed_count;
            }
        }
    } catch (const std::invalid_argument&) {
        return 0;
    }

    LegacyStraightSegment& incoming = segments[curve_index];
    LegacyStraightSegment& outgoing = segments[curve_index + 1];
    int circular_sample = incoming.first_index;
    for (; circular_sample <= outgoing.last_index; ++circular_sample) {
        if (property[static_cast<std::size_t>(circular_sample)] ==
            static_cast<int>(HorizontalSampleKind::circular)) {
            break;
        }
    }
    if (circular_sample > outgoing.last_index) {
        return changed_count;
    }

    int current_index = incoming.first_index;
    int current_property = property[static_cast<std::size_t>(current_index)];
    for (int sample = current_index + 1; sample <= outgoing.last_index; ++sample) {
        if (property[static_cast<std::size_t>(sample)] != current_property) {
            incoming.last_index = sample - 1;
            current_index = sample;
            current_property = property[static_cast<std::size_t>(sample)];
            break;
        }
    }

    if (current_property == static_cast<int>(HorizontalSampleKind::circular)) {
        incoming.circular_first_index = current_index;
    } else {
        for (int sample = current_index + 1; sample <= outgoing.last_index; ++sample) {
            if (property[static_cast<std::size_t>(sample)] != current_property) {
                incoming.circular_first_index = sample - 1;
                current_index = sample;
                current_property = property[static_cast<std::size_t>(sample)];
                break;
            }
        }
    }
    for (int sample = current_index + 1; sample <= outgoing.last_index; ++sample) {
        if (property[static_cast<std::size_t>(sample)] != current_property) {
            incoming.circular_last_index = sample - 1;
            current_index = sample;
            current_property = property[static_cast<std::size_t>(sample)];
            break;
        }
    }
    if (current_property == static_cast<int>(HorizontalSampleKind::straight)) {
        outgoing.first_index = current_index;
    } else {
        for (int sample = current_index + 1; sample <= outgoing.last_index; ++sample) {
            if (property[static_cast<std::size_t>(sample)] != current_property) {
                outgoing.first_index = sample;
                break;
            }
        }
    }
    incoming.circular_sample_count =
        incoming.circular_last_index - incoming.circular_first_index + 1;
    return changed_count;
}

void swing_legacy_spiral_samples(
    const std::vector<Point2D>& points,
    const std::vector<double>& cumulative_distance_m,
    std::vector<int>& property,
    std::vector<LegacyStraightSegment>& segments) {
    int curve_count = static_cast<int>(segments.size()) - 1;
    if (curve_count <= 0) {
        return;
    }
    for (int curve_index = 0; curve_index < curve_count; ++curve_index) {
        LegacyStraightSegment& segment =
            segments[static_cast<std::size_t>(curve_index)];
        segment.circular_first_index = segment.last_index + 1;
        segment.circular_last_index =
            segments[static_cast<std::size_t>(curve_index + 1)].first_index - 1;
        segment.circular_sample_count =
            segment.circular_last_index - segment.circular_first_index + 1;
    }

    std::vector<double> entry_lengths(static_cast<std::size_t>(curve_count), 0.0);
    std::vector<double> exit_lengths(static_cast<std::size_t>(curve_count), 0.0);
    std::size_t iteration = 0;
    bool should_adjust = false;
    do {
        int changed_count = 0;
        should_adjust = false;
        for (int curve_index = 0; curve_index < curve_count; ++curve_index) {
            const std::size_t index = static_cast<std::size_t>(curve_index);
            LegacyStraightSegment& segment = segments[index];
            if (segment.circular_sample_count < 3) {
                continue;
            }
            const CircleFitResult circle = fit_circle_to_range(
                points,
                static_cast<std::size_t>(segment.circular_first_index),
                static_cast<std::size_t>(segment.circular_last_index));
            if (!circle.valid) {
                continue;
            }
            const LegacyTangentPair tangents = fit_legacy_tangent_pair(
                points, segment, segments[index + 1]);
            if (!tangents.valid) {
                entry_lengths[index] = 0.0;
                exit_lengths[index] = 0.0;
                continue;
            }
            const double previous_entry = iteration > 0 ? entry_lengths[index] : 0.0;
            const double previous_exit = iteration > 0 ? exit_lengths[index] : 0.0;
            const int gap_first = segment.last_index + 1;
            const int gap_last = segments[index + 1].first_index - 1;
            const double curve_length =
                cumulative_distance_m[static_cast<std::size_t>(gap_last)] -
                cumulative_distance_m[static_cast<std::size_t>(gap_first)];
            const auto lengths = estimate_legacy_spiral_lengths(
                circle, tangents, curve_length);
            entry_lengths[index] = lengths.first;
            exit_lengths[index] = lengths.second;
            if (std::abs(previous_entry - lengths.first) > 5.0 ||
                std::abs(previous_exit - lengths.second) > 5.0) {
                should_adjust = true;
            }
            changed_count += resort_legacy_point_properties(
                points, circle, tangents, lengths.first, lengths.second,
                index, property, segments);
        }

        segments = form_legacy_spiral_segments(property);
        curve_count = static_cast<int>(segments.size()) - 1;
        entry_lengths.resize(static_cast<std::size_t>(std::max(0, curve_count)), 0.0);
        exit_lengths.resize(static_cast<std::size_t>(std::max(0, curve_count)), 0.0);
        ++iteration;
        if (iteration >= 500 || changed_count == 0) {
            should_adjust = false;
        }
    } while (should_adjust);
}

double three_point_tangent(
    const Point2D& before,
    const Point2D& point,
    const Point2D& after,
    Point2D& retained_circle_center,
    bool& has_retained_circle_center) {
    const auto legacy_heading = [](const Point2D& from, const Point2D& to) {
        const double dx = to.x - from.x;
        const double dy = to.y - from.y;
        double angle = 0.0;
        if (std::abs(dx) < 1.0e-9) {
            angle = dy >= 0.0 ? 0.5 * pi : 1.5 * pi;
        } else {
            angle = std::atan(dy / dx);
            if (dx < 0.0) {
                angle += pi;
            } else if (dy < 0.0) {
                angle += 2.0 * pi;
            }
            if (angle < 0.0) {
                angle += 2.0 * pi;
            } else if (angle > 2.0 * pi) {
                angle -= 2.0 * pi;
            }
        }
        return std::pair<double, double>{std::hypot(dx, dy), angle};
    };

    const auto incoming_result = legacy_heading(before, point);
    const auto outgoing_result = legacy_heading(point, after);
    const double incoming = incoming_result.second;
    const double outgoing = outgoing_result.second;
    const double deflection = normalize_signed_angle(outgoing - incoming);
    const bool circle_rejected =
        std::abs(incoming_result.first - outgoing_result.first) < 1.0e-5 ||
        std::abs(incoming - outgoing) < 1.0e-5 ||
        std::abs(std::abs(incoming - outgoing) - pi) < 1.0e-5;
    if (!circle_rejected) {
        const Point2D first_midpoint{
            0.5 * (before.x + point.x),
            0.5 * (before.y + point.y)
        };
        const Point2D second_midpoint{
            0.5 * (point.x + after.x),
            0.5 * (point.y + after.y)
        };
        const Point2D first_perpendicular{
            first_midpoint.x + 10.0 * std::cos(incoming + 0.5 * pi),
            first_midpoint.y + 10.0 * std::sin(incoming + 0.5 * pi)
        };
        const Point2D second_perpendicular{
            second_midpoint.x + 10.0 * std::cos(outgoing + 0.5 * pi),
            second_midpoint.y + 10.0 * std::sin(outgoing + 0.5 * pi)
        };
        LegacyLineFitResult first_bisector;
        first_bisector.start = first_midpoint;
        first_bisector.end = first_perpendicular;
        first_bisector.valid = true;
        LegacyLineFitResult second_bisector;
        second_bisector.start = second_midpoint;
        second_bisector.end = second_perpendicular;
        second_bisector.valid = true;
        Point2D center{};
        if (intersect_legacy_lines(first_bisector, second_bisector, center)) {
            retained_circle_center = center;
            has_retained_circle_center = true;
        }
    }

    if (std::abs(deflection) < 0.01) {
        double result = 0.5 * (incoming + outgoing);
        if (result < 0.0) {
            result += 2.0 * pi;
        } else if (result > 2.0 * pi) {
            result -= 2.0 * pi;
        }
        return result;
    }
    if (!has_retained_circle_center) {
        return normalize_heading(incoming + 0.5 * deflection);
    }

    double cad_angle = std::atan2(
        point.x - retained_circle_center.x,
        point.y - retained_circle_center.y);
    if (cad_angle < 0.0) {
        cad_angle += 2.0 * pi;
    }
    double radial_heading = 2.5 * pi - cad_angle;
    if (radial_heading > 2.0 * pi) {
        radial_heading -= 2.0 * pi;
    }
    double result = radial_heading +
        (deflection > 0.0 ? 0.5 * pi : -0.5 * pi);
    if (result < 0.0) {
        result += 2.0 * pi;
    } else if (result > 2.0 * pi) {
        result -= 2.0 * pi;
    }
    return result;
}

} // namespace

HorizontalSampleAnalysis classify_horizontal_samples(
    const std::vector<Point2D>& ordered_points,
    const ReconstructionOptions& options,
    const std::vector<HorizontalCurvatureSample>& curvature_graph) {
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
        if (!std::isfinite(ordered_points[index].x) ||
            !std::isfinite(ordered_points[index].y)) {
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

    if (!curvature_graph.empty()) {
        if (curvature_graph.size() != count) {
            throw std::invalid_argument(
                "Provided curvature graph size does not match horizontal samples");
        }
        std::vector<int> property;
        property.reserve(count);
        for (const HorizontalCurvatureSample& sample : curvature_graph) {
            analysis.curvature_inverse_m[property.size()] = sample.curvature_inverse_m;
            property.push_back(
                sample.property == HorizontalSampleProperty::straight ? 0 : 1);
        }
        analysis.curvature_graph = curvature_graph;
        const std::vector<int> original_property = property;
        std::vector<LegacyStraightSegment> segments =
            form_legacy_straight_segments(property);
        if (property != original_property) {
            throw std::invalid_argument(
                "Provided curvature graph has invalid route-end properties");
        }
        if (segments.size() < 2) {
            LegacyStraightSegment& segment = segments.front();
            segment.following_curve_sample_count = 0;
            segment.circular_first_index = segment.last_index;
            segment.circular_last_index = segment.last_index;
            segment.circular_sample_count = 0;
            finalize_legacy_analysis(analysis, property, segments);
            return analysis;
        }
        identify_legacy_spiral_samples(
            ordered_points, analysis.cumulative_distance_m,
            options, property, segments);
        swing_legacy_spiral_samples(
            ordered_points, analysis.cumulative_distance_m,
            property, segments);
        finalize_legacy_analysis(analysis, property, segments);
        return analysis;
    }

    if (count < 4 ||
        analysis.cumulative_distance_m.back() <= 2.0 * options.curvature_window_m) {
        const double route_heading = heading(ordered_points.front(), ordered_points.back());
        std::fill(analysis.tangent_heading_rad.begin(),
                  analysis.tangent_heading_rad.end(), route_heading);
        analysis.sample_kind.assign(count, HorizontalSampleKind::straight);
        analysis.straight_segments.push_back({0, count - 1, 0, count - 1, count - 1, 0});
        analysis.runs = {{false, 0, count - 1}};
        capture_line_curve_graph(analysis, std::vector<int>(count, 0));
        return analysis;
    }

    std::size_t first_supported = 0;
    while (first_supported < count &&
           analysis.cumulative_distance_m[first_supported] <= options.curvature_window_m) {
        ++first_supported;
    }
    std::size_t last_supported = count - 2;
    while (last_supported > 0 &&
           analysis.cumulative_distance_m.back() -
               analysis.cumulative_distance_m[last_supported] <= options.curvature_window_m) {
        --last_supported;
    }
    if (first_supported >= last_supported) {
        throw std::invalid_argument("Horizontal samples do not span the curvature window");
    }

    Point2D retained_circle_center{};
    bool has_retained_circle_center = false;
    for (std::size_t index = first_supported; index <= last_supported; ++index) {
        std::size_t before = index - 1;
        while (before > 0 && analysis.cumulative_distance_m[index] -
                analysis.cumulative_distance_m[before] <= options.curvature_window_m) {
            --before;
        }
        std::size_t after = index + 1;
        while (after < count && analysis.cumulative_distance_m[after] -
                analysis.cumulative_distance_m[index] <= options.curvature_window_m) {
            ++after;
        }
        if (after >= count) {
            throw std::invalid_argument("Horizontal samples do not span the forward curvature window");
        }
        analysis.tangent_heading_rad[index] = three_point_tangent(
            ordered_points[before], ordered_points[index], ordered_points[after],
            retained_circle_center, has_retained_circle_center);
    }
    std::fill(analysis.tangent_heading_rad.begin(),
              analysis.tangent_heading_rad.begin() + static_cast<std::ptrdiff_t>(first_supported + 1),
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
            while (after < count && analysis.cumulative_distance_m[after] -
                    analysis.cumulative_distance_m[index] <= half_window) {
                ++after;
            }
            if (after >= count) {
                throw std::invalid_argument("Horizontal samples do not span the forward half-window");
            }
        }
        double heading_change =
            analysis.tangent_heading_rad[after] - analysis.tangent_heading_rad[before];
        if (std::abs(heading_change) > pi) {
            heading_change -= std::copysign(2.0 * pi, heading_change);
        }
        analysis.curvature_inverse_m[index] =
            heading_change /
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

    std::vector<int> property(count, 0);
    for (std::size_t index = 0; index < count; ++index) {
        property[index] = std::abs(analysis.curvature_inverse_m[index]) <
                options.curvature_threshold_inverse_m
            ? 0
            : 1;
    }
    stabilize_legacy_line_curve_properties(
        ordered_points, analysis.tangent_heading_rad,
        analysis.curvature_inverse_m, property, true);
    std::vector<LegacyStraightSegment> segments =
        form_legacy_straight_segments(property);
    if (segments.size() < 2) {
        LegacyStraightSegment& segment = segments.front();
        segment.following_curve_sample_count = 0;
        segment.circular_first_index = segment.last_index;
        segment.circular_last_index = segment.last_index;
        segment.circular_sample_count = 0;
        capture_line_curve_graph(analysis, property);
        finalize_legacy_analysis(analysis, property, segments);
        return analysis;
    }
    split_reverse_curves(
        analysis.curvature_inverse_m, property, segments);
    remove_short_curve_ranges(
        analysis.cumulative_distance_m, property, segments);
    dynamically_adjust_line_curve_properties(
        ordered_points, analysis.tangent_heading_rad,
        analysis.curvature_inverse_m, 6, property, segments);
    if (segments.size() < 2) {
        LegacyStraightSegment& segment = segments.front();
        segment.following_curve_sample_count = 0;
        segment.circular_first_index = segment.last_index;
        segment.circular_last_index = segment.last_index;
        segment.circular_sample_count = 0;
        capture_line_curve_graph(analysis, property);
        finalize_legacy_analysis(analysis, property, segments);
        return analysis;
    }
    capture_line_curve_graph(analysis, property);
    identify_legacy_spiral_samples(
        ordered_points, analysis.cumulative_distance_m,
        options, property, segments);
    swing_legacy_spiral_samples(
        ordered_points, analysis.cumulative_distance_m,
        property, segments);
    finalize_legacy_analysis(analysis, property, segments);
    return analysis;
}

} // namespace alignment_reconstruct::detail
