#include "horizontal/horizontal_alignment.h"

#include "geometry/vector_math.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace alignment_reconstruct::detail {
namespace {

constexpr double geometry_tolerance = 1.0e-9;

struct CurveGeometry {
    double incoming_tangent_m = 0.0;
    double outgoing_tangent_m = 0.0;
    double circular_length_m = 0.0;
    double signed_curvature_inverse_m = 0.0;
};

struct SpiralShift {
    double tangent_shift_m = 0.0;
    double radial_shift_m = 0.0;
};

SpiralShift spiral_shift(double radius_m, double length_m) {
    if (length_m <= geometry_tolerance) {
        return {};
    }
    const HorizontalElement spiral(
        HorizontalElementKind::entry_spiral, 0.0, {}, 0.0, length_m,
        0.0, 1.0 / radius_m);
    const auto end = spiral.evaluate(length_m);
    const double deflection = length_m / (2.0 * radius_m);
    return {
        end.point.x - radius_m * std::sin(deflection),
        end.point.y + radius_m * std::cos(deflection) - radius_m
    };
}

CurveGeometry calculate_curve_geometry(
    double signed_deflection_rad,
    double radius_m,
    double entry_spiral_length_m,
    double exit_spiral_length_m) {
    const double angle = std::abs(signed_deflection_rad);
    const double radius = std::abs(radius_m);
    if (radius <= geometry_tolerance || angle <= geometry_tolerance || angle >= pi) {
        throw std::invalid_argument("Invalid horizontal curve radius or deflection");
    }
    if (entry_spiral_length_m < 0.0 || exit_spiral_length_m < 0.0) {
        throw std::invalid_argument("Horizontal spiral length cannot be negative");
    }

    const double entry_deflection = entry_spiral_length_m / (2.0 * radius);
    const double exit_deflection = exit_spiral_length_m / (2.0 * radius);
    const double circular_angle = angle - entry_deflection - exit_deflection;
    if (circular_angle < -geometry_tolerance) {
        throw std::invalid_argument("Horizontal spirals overlap");
    }

    const SpiralShift entry = spiral_shift(radius, entry_spiral_length_m);
    const SpiralShift exit = spiral_shift(radius, exit_spiral_length_m);
    const double sine = std::sin(angle);
    if (std::abs(sine) <= geometry_tolerance) {
        throw std::invalid_argument("Degenerate horizontal curve deflection");
    }

    CurveGeometry geometry;
    geometry.incoming_tangent_m =
        (radius + entry.radial_shift_m) * std::tan(0.5 * angle) +
        entry.tangent_shift_m -
        (entry.radial_shift_m - exit.radial_shift_m) / sine;
    geometry.outgoing_tangent_m =
        (radius + exit.radial_shift_m) * std::tan(0.5 * angle) +
        exit.tangent_shift_m +
        (entry.radial_shift_m - exit.radial_shift_m) / sine;
    geometry.circular_length_m = std::max(0.0, circular_angle * radius);
    geometry.signed_curvature_inverse_m =
        (signed_deflection_rad > 0.0 ? 1.0 : -1.0) / radius;
    return geometry;
}

Point2D tangent_point(const Point2D& intersection, double heading_rad, double distance_m) {
    return add(intersection, scale(direction(heading_rad), distance_m));
}

void append_element(
    std::vector<HorizontalElement>& elements,
    HorizontalElementKind kind,
    double& station_m,
    Point2D& point,
    double& heading_rad,
    double length_m,
    double start_curvature_inverse_m,
    double end_curvature_inverse_m) {
    if (length_m <= geometry_tolerance) {
        return;
    }
    elements.emplace_back(kind, station_m, point, heading_rad, length_m,
                          start_curvature_inverse_m, end_curvature_inverse_m);
    const auto end = elements.back().evaluate(length_m);
    station_m += length_m;
    point = end.point;
    heading_rad = end.heading_rad;
}

double squared_distance_at(
    const HorizontalElement& element,
    const Point2D& point,
    double local_station_m) {
    const Point2D delta = subtract(element.evaluate(local_station_m).point, point);
    return dot(delta, delta);
}

double project_to_element(const HorizontalElement& element, const Point2D& point) {
    if (element.kind() == HorizontalElementKind::straight) {
        return clamp(
            dot(subtract(point, element.start_point()), direction(element.start_heading_rad())),
            0.0, element.length_m());
    }

    constexpr int samples = 16;
    int best_sample = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int sample = 0; sample <= samples; ++sample) {
        const double station = element.length_m() * static_cast<double>(sample) /
            static_cast<double>(samples);
        const double candidate = squared_distance_at(element, point, station);
        if (candidate < best_distance) {
            best_distance = candidate;
            best_sample = sample;
        }
    }

    double lower = element.length_m() * static_cast<double>(std::max(0, best_sample - 1)) /
        static_cast<double>(samples);
    double upper = element.length_m() * static_cast<double>(std::min(samples, best_sample + 1)) /
        static_cast<double>(samples);
    constexpr double inverse_phi = 0.6180339887498948482;
    double left = upper - inverse_phi * (upper - lower);
    double right = lower + inverse_phi * (upper - lower);
    double left_value = squared_distance_at(element, point, left);
    double right_value = squared_distance_at(element, point, right);
    for (int iteration = 0; iteration < 60; ++iteration) {
        if (left_value < right_value) {
            upper = right;
            right = left;
            right_value = left_value;
            left = upper - inverse_phi * (upper - lower);
            left_value = squared_distance_at(element, point, left);
        } else {
            lower = left;
            left = right;
            left_value = right_value;
            right = lower + inverse_phi * (upper - lower);
            right_value = squared_distance_at(element, point, right);
        }
    }
    double station = 0.5 * (lower + upper);
    for (int iteration = 0; iteration < 8; ++iteration) {
        const auto state = element.evaluate(station);
        const Point2D tangent = direction(state.heading_rad);
        const Point2D point_from_curve = subtract(point, state.point);
        const double residual = -dot(point_from_curve, tangent);
        const double signed_offset = cross(tangent, point_from_curve);
        const double derivative = 1.0 - state.curvature_inverse_m * signed_offset;
        if (std::abs(derivative) <= 1.0e-12) {
            break;
        }
        const double next = clamp(station - residual / derivative, 0.0, element.length_m());
        if (std::abs(next - station) <= 1.0e-12) {
            station = next;
            break;
        }
        station = next;
    }
    return station;
}

} // namespace

HorizontalAlignment::HorizontalAlignment(
    const std::vector<HorizontalControlPoint>& control_points,
    double start_station_m)
    : start_station_m_(start_station_m), end_station_m_(start_station_m) {
    if (control_points.size() < 2 || !std::isfinite(start_station_m)) {
        throw std::invalid_argument("At least two horizontal control points are required");
    }
    for (const auto& control : control_points) {
        if (!std::isfinite(control.position.x) || !std::isfinite(control.position.y) ||
            !std::isfinite(control.radius_m) ||
            !std::isfinite(control.entry_spiral_length_m) ||
            !std::isfinite(control.exit_spiral_length_m)) {
            throw std::invalid_argument("Horizontal control point contains non-finite data");
        }
    }

    double station = start_station_m;
    Point2D cursor = control_points.front().position;
    double current_heading = heading(control_points[0].position, control_points[1].position);

    for (std::size_t index = 1; index + 1 < control_points.size(); ++index) {
        const auto& previous = control_points[index - 1].position;
        const auto& intersection = control_points[index].position;
        const auto& next = control_points[index + 1].position;
        if (distance(previous, intersection) <= geometry_tolerance ||
            distance(intersection, next) <= geometry_tolerance) {
            throw std::invalid_argument("Consecutive horizontal control points coincide");
        }
        const double incoming_heading = heading(previous, intersection);
        const double outgoing_heading = heading(intersection, next);
        const double signed_deflection =
            normalize_signed_angle(outgoing_heading - incoming_heading);
        const CurveGeometry curve = calculate_curve_geometry(
            signed_deflection, control_points[index].radius_m,
            control_points[index].entry_spiral_length_m,
            control_points[index].exit_spiral_length_m);

        const Point2D curve_start = tangent_point(
            intersection, incoming_heading, -curve.incoming_tangent_m);
        const double line_length = dot(subtract(curve_start, cursor), direction(incoming_heading));
        if (line_length < -1.0e-6) {
            throw std::invalid_argument("Adjacent horizontal curves overlap");
        }
        current_heading = incoming_heading;
        append_element(elements_, HorizontalElementKind::straight, station, cursor,
                       current_heading, std::max(0.0, line_length), 0.0, 0.0);
        if (distance(cursor, curve_start) > 1.0e-5) {
            throw std::invalid_argument("Horizontal tangent construction is discontinuous");
        }
        cursor = curve_start;

        append_element(elements_, HorizontalElementKind::entry_spiral, station, cursor,
                       current_heading, control_points[index].entry_spiral_length_m,
                       0.0, curve.signed_curvature_inverse_m);
        append_element(elements_, HorizontalElementKind::circular, station, cursor,
                       current_heading, curve.circular_length_m,
                       curve.signed_curvature_inverse_m, curve.signed_curvature_inverse_m);
        append_element(elements_, HorizontalElementKind::exit_spiral, station, cursor,
                       current_heading, control_points[index].exit_spiral_length_m,
                       curve.signed_curvature_inverse_m, 0.0);

        const Point2D expected_curve_end = tangent_point(
            intersection, outgoing_heading, curve.outgoing_tangent_m);
        if (distance(cursor, expected_curve_end) > 1.0e-5 ||
            std::abs(normalize_signed_angle(current_heading - outgoing_heading)) > 1.0e-10) {
            throw std::invalid_argument("Horizontal curve construction failed closure check");
        }
        cursor = expected_curve_end;
        current_heading = outgoing_heading;
    }

    const Point2D end = control_points.back().position;
    const double final_length = dot(subtract(end, cursor), direction(current_heading));
    if (final_length < -1.0e-6 ||
        distance(add(cursor, scale(direction(current_heading), final_length)), end) > 1.0e-5) {
        throw std::invalid_argument("Final horizontal tangent is infeasible");
    }
    append_element(elements_, HorizontalElementKind::straight, station, cursor,
                   current_heading, std::max(0.0, final_length), 0.0, 0.0);
    if (elements_.empty()) {
        throw std::invalid_argument("Horizontal alignment has no positive-length element");
    }
    end_station_m_ = station;
}

HorizontalElementState HorizontalAlignment::evaluate(double station_m) const {
    const double station = clamp(station_m, start_station_m_, end_station_m_);
    const auto iterator = std::lower_bound(
        elements_.begin(), elements_.end(), station,
        [](const HorizontalElement& element, double value) {
            return element.end_station_m() < value;
        });
    const auto& element = iterator == elements_.end() ? elements_.back() : *iterator;
    return element.evaluate(station - element.start_station_m());
}

HorizontalProjection HorizontalAlignment::project(const Point2D& point) const {
    HorizontalProjection result;
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return result;
    }
    double best_squared_distance = std::numeric_limits<double>::infinity();
    for (const auto& element : elements_) {
        const double local_station = project_to_element(element, point);
        const auto state = element.evaluate(local_station);
        const Point2D delta = subtract(point, state.point);
        const double squared_distance = dot(delta, delta);
        if (squared_distance < best_squared_distance) {
            best_squared_distance = squared_distance;
            result.valid = true;
            result.station_m = element.start_station_m() + local_station;
            result.state = state;
            result.signed_offset_m = cross(direction(state.heading_rad), delta);
            result.distance_m = std::sqrt(squared_distance);
            result.element_kind = element.kind();
        }
    }
    return result;
}

double HorizontalAlignment::start_station_m() const noexcept { return start_station_m_; }
double HorizontalAlignment::end_station_m() const noexcept { return end_station_m_; }
const std::vector<HorizontalElement>& HorizontalAlignment::elements() const noexcept {
    return elements_;
}

} // namespace alignment_reconstruct::detail
