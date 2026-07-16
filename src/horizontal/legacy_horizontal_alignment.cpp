#include "horizontal/legacy_horizontal_alignment.h"

#include "geometry/vector_math.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace alignment_reconstruct::detail {
namespace {

struct LegacyCurveGeometry {
    double entry_tangent_m = 0.0;
    double exit_tangent_m = 0.0;
    double total_length_m = 0.0;
};

double legacy_distance(const Point2D& first, const Point2D& second) {
    const double delta_x = first.x - second.x;
    const double delta_y = first.y - second.y;
    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

long legacy_factorial(int value) {
    long result = 1;
    for (int index = 1; index <= value; ++index) {
        result *= index;
    }
    return result;
}

double legacy_spiral_x(double parameter_a_m, double length_m) {
    if (parameter_a_m <= 0.0) {
        return 0.0;
    }
    const double c = 2.0 * parameter_a_m * parameter_a_m;
    double sum = 0.0;
    for (int index = 0; index < 10; ++index) {
        sum += std::pow(-1.0, index + 2) *
            std::pow(length_m, 4 * index + 1) /
            static_cast<double>(4 * index + 1) /
            std::pow(c, 2 * index) /
            static_cast<double>(legacy_factorial(2 * index));
    }
    return sum;
}

double legacy_spiral_y(double parameter_a_m, double length_m) {
    if (parameter_a_m <= 0.0) {
        return 0.0;
    }
    const double t = length_m * length_m / 2.0 /
        parameter_a_m / parameter_a_m;
    double sum = 0.0;
    for (int index = 1; index <= 10; ++index) {
        const double exponent = 0.5 * static_cast<double>(4 * index - 1);
        sum += std::pow(-1.0, index + 1) * std::pow(t, exponent) /
            exponent / static_cast<double>(legacy_factorial(2 * index - 1));
    }
    return sum * parameter_a_m / std::sqrt(2.0);
}

double legacy_spiral_shift_x(double parameter_a_m, double length_m) {
    const double radius_m = parameter_a_m * parameter_a_m / length_m;
    const double angle = parameter_a_m * parameter_a_m /
        (2.0 * radius_m * radius_m);
    return legacy_spiral_x(parameter_a_m, length_m) -
        radius_m * std::sin(angle);
}

double legacy_spiral_shift_y(double parameter_a_m, double length_m) {
    const double radius_m = parameter_a_m * parameter_a_m / length_m;
    const double angle = parameter_a_m * parameter_a_m /
        (2.0 * radius_m * radius_m);
    return legacy_spiral_y(parameter_a_m, length_m) +
        radius_m * std::cos(angle) - radius_m;
}

double legacy_dms(double degrees) {
    const int degree = static_cast<int>(degrees);
    const int minute = static_cast<int>((degrees - degree) * 60.0);
    const double second = (degrees - degree) * 3600.0 - minute * 60.0;
    return degree + minute / 100.0 + second / 10000.0;
}

double legacy_degrees(double dms) {
    const int sign = dms > -0.000001 ? 1 : -1;
    dms = std::abs(dms);
    const int degree = static_cast<int>(dms + 0.000000001);
    const int minute = static_cast<int>(
        (dms - degree) * 100.0 + 0.00000000001);
    const double second = dms * 10000.0 - degree * 10000.0 -
        minute * 100.0;
    return sign * (degree + minute / 60.0 + second / 3600.0);
}

double legacy_radians_to_dms(double radians) {
    return legacy_dms(radians * 180.0 / pi);
}

double legacy_dms_to_radians(double dms) {
    return legacy_degrees(dms) * pi / 180.0;
}

double legacy_round_trip_angle(double radians) {
    return legacy_dms_to_radians(legacy_radians_to_dms(radians));
}

double legacy_heading(const Point2D& from, const Point2D& to) {
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
    return angle;
}

double legacy_turn(double second_heading, double first_heading) {
    double angle = second_heading - first_heading;
    if (std::abs(angle) > pi) {
        angle += angle < 0.0 ? 2.0 * pi : -2.0 * pi;
    }
    return angle;
}

double legacy_side(
    const Point2D& origin,
    const Point2D& line_point,
    const Point2D& point) {
    return (point.x - origin.x) * (line_point.y - origin.y) -
        (line_point.x - origin.x) * (point.y - origin.y);
}

LegacyCurveGeometry legacy_curve_geometry(
    double angle_rad,
    double radius_m,
    double entry_spiral_length_m,
    double exit_spiral_length_m) {
    LegacyCurveGeometry result;
    const double angle_dms = legacy_radians_to_dms(std::abs(angle_rad));
    const double angle = legacy_dms_to_radians(angle_dms);
    const double radius = std::abs(radius_m);
    if (radius < 0.01) {
        return result;
    }

    const double entry_a = std::sqrt(radius * entry_spiral_length_m);
    const double exit_a = std::sqrt(radius * exit_spiral_length_m);
    const double entry_angle = entry_spiral_length_m * 0.5 / radius;
    const double exit_angle = exit_spiral_length_m * 0.5 / radius;
    const double entry_shift = entry_spiral_length_m < 0.0001
        ? 0.0
        : legacy_spiral_shift_y(entry_a, entry_spiral_length_m);
    const double exit_shift = exit_spiral_length_m < 0.0001
        ? 0.0
        : legacy_spiral_shift_y(exit_a, exit_spiral_length_m);
    const double entry_tangent_shift = entry_spiral_length_m < 0.0001
        ? 0.0
        : legacy_spiral_shift_x(entry_a, entry_spiral_length_m);
    const double exit_tangent_shift = exit_spiral_length_m < 0.0001
        ? 0.0
        : legacy_spiral_shift_x(exit_a, exit_spiral_length_m);

    if (angle > pi) {
        result.entry_tangent_m =
            (radius + entry_shift) * std::tan(pi - angle * 0.5) -
            entry_tangent_shift + (entry_shift - exit_shift) / std::sin(angle);
        result.exit_tangent_m =
            (radius + exit_shift) * std::tan(pi - angle * 0.5) -
            exit_tangent_shift - (entry_shift - exit_shift) / std::sin(angle);
    } else {
        result.entry_tangent_m =
            (radius + entry_shift) * std::tan(angle * 0.5) +
            entry_tangent_shift - (entry_shift - exit_shift) / std::sin(angle);
        result.exit_tangent_m =
            (radius + exit_shift) * std::tan(angle * 0.5) +
            exit_tangent_shift + (entry_shift - exit_shift) / std::sin(angle);
    }
    result.total_length_m =
        (angle - entry_angle - exit_angle) * radius +
        entry_spiral_length_m + exit_spiral_length_m;
    return result;
}

} // namespace

LegacyHorizontalAlignment::LegacyHorizontalAlignment(
    Point2D start,
    Point2D intersection,
    Point2D end,
    double radius_m,
    double entry_spiral_length_m,
    double exit_spiral_length_m) {
    if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
        !std::isfinite(intersection.x) || !std::isfinite(intersection.y) ||
        !std::isfinite(end.x) || !std::isfinite(end.y) ||
        !std::isfinite(radius_m) || !std::isfinite(entry_spiral_length_m) ||
        !std::isfinite(exit_spiral_length_m) || radius_m < 0.0 ||
        entry_spiral_length_m < 0.0 || exit_spiral_length_m < 0.0) {
        throw std::invalid_argument("Invalid legacy horizontal alignment");
    }

    const double incoming_heading = legacy_heading(start, intersection);
    const double outgoing_heading = legacy_heading(intersection, end);
    const double signed_turn = legacy_turn(outgoing_heading, incoming_heading);
    const double turn = signed_turn > 0.0 ? 1.0 : -1.0;
    double entry_length = entry_spiral_length_m;
    double exit_length = exit_spiral_length_m;
    LegacyCurveGeometry curve = legacy_curve_geometry(
        signed_turn, radius_m, entry_length, exit_length);
    if (curve.total_length_m < entry_length + exit_length) {
        entry_length = 0.0;
        exit_length = 0.0;
        curve = legacy_curve_geometry(signed_turn, radius_m, 0.0, 0.0);
    }

    const double incoming_distance = legacy_distance(start, intersection);
    const double outgoing_distance = legacy_distance(intersection, end);
    const double incoming_line_length =
        incoming_distance - curve.entry_tangent_m;
    const double outgoing_line_length =
        outgoing_distance - curve.exit_tangent_m;
    Row metadata;
    metadata.type = 10;
    metadata.heading_rad = legacy_round_trip_angle(incoming_heading);
    metadata.station_m = 0.0;
    metadata.radius_m = 1.0e40;
    metadata.point = start;
    rows_.push_back(metadata);

    Row incoming_line;
    incoming_line.type = 1;
    incoming_line.parameter1 = incoming_line_length;
    incoming_line.heading_rad = metadata.heading_rad;
    incoming_line.station_m = 0.0;
    incoming_line.radius_m = 1.0e40;
    incoming_line.point = start;
    rows_.push_back(incoming_line);

    State cursor = evaluate_row(incoming_line, incoming_line_length);
    double station = incoming_line_length;
    if (entry_length > 0.001) {
        Row entry;
        entry.type = 3;
        entry.parameter1 = std::sqrt(entry_length * std::abs(radius_m));
        entry.parameter2 = std::abs(radius_m);
        entry.turn = turn;
        entry.heading_rad = cursor.heading_rad;
        entry.station_m = station;
        entry.radius_m = cursor.radius_m;
        entry.point = cursor.point;
        rows_.push_back(entry);
        cursor = evaluate_row(entry, entry_length);
        station += entry_length;
    }

    const double circular_length =
        curve.total_length_m - entry_length - exit_length;
    Row circular;
    circular.type = 2;
    circular.parameter1 = std::abs(radius_m);
    circular.parameter2 = circular_length;
    circular.turn = turn;
    circular.heading_rad = cursor.heading_rad;
    circular.station_m = station;
    circular.radius_m = cursor.radius_m;
    circular.point = cursor.point;
    rows_.push_back(circular);
    cursor = evaluate_row(circular, circular_length);
    station += circular_length;

    if (exit_length > 0.001) {
        Row exit;
        exit.type = 4;
        exit.parameter1 = std::sqrt(exit_length * std::abs(radius_m));
        exit.parameter2 = std::abs(radius_m);
        exit.turn = turn;
        exit.heading_rad = cursor.heading_rad;
        exit.station_m = station;
        exit.radius_m = cursor.radius_m;
        exit.point = cursor.point;
        rows_.push_back(exit);
        station += exit_length;
    }

    Row outgoing_line;
    outgoing_line.type = 1;
    outgoing_line.parameter1 = outgoing_line_length;
    outgoing_line.heading_rad = outgoing_heading;
    outgoing_line.station_m = station;
    outgoing_line.radius_m = 1.0e40;
    outgoing_line.point = add(
        intersection, scale(direction(outgoing_heading), curve.exit_tangent_m));
    rows_.push_back(outgoing_line);

    Row endpoint;
    endpoint.heading_rad = outgoing_heading;
    endpoint.station_m = station + outgoing_line_length;
    endpoint.radius_m = 1.0e40;
    endpoint.point = add(
        outgoing_line.point,
        scale(direction(outgoing_heading), outgoing_line_length));
    rows_.push_back(endpoint);
}

double LegacyHorizontalAlignment::row_length(const Row& row) const {
    switch (row.type) {
    case 1:
        return row.parameter1;
    case 2:
        return row.parameter2;
    case 3:
    case 4:
        return std::abs(row.parameter2) > 0.00001
            ? row.parameter1 * row.parameter1 / row.parameter2
            : 0.0;
    default:
        return 0.0;
    }
}

LegacyHorizontalAlignment::State LegacyHorizontalAlignment::evaluate_row(
    const Row& row,
    double length_m) const {
    State result;
    result.point = row.point;
    result.heading_rad = row.heading_rad;
    result.radius_m = row.radius_m;
    const double turn = row.turn;
    if (row.type == 1) {
        result.point = add(row.point, scale(direction(row.heading_rad), length_m));
        result.radius_m = 1.0e40;
    } else if (row.type == 2) {
        const double radius = row.parameter1;
        const double angle = length_m / radius;
        const double end_heading = row.heading_rad + turn * angle;
        const double tangent = radius * std::tan(angle * 0.5);
        const Point2D midpoint = add(
            row.point, scale(direction(row.heading_rad), tangent));
        result.point = add(midpoint, scale(direction(end_heading), tangent));
        result.heading_rad = end_heading;
        result.radius_m = radius;
    } else if (row.type == 3) {
        const double parameter_a = row.parameter1;
        const double radius = row.parameter2;
        const double total_length = parameter_a * parameter_a / radius;
        const double angle = length_m * length_m * 0.5 /
            radius / total_length;
        const double x = legacy_spiral_x(parameter_a, length_m);
        const double y = legacy_spiral_y(parameter_a, length_m);
        const double tangent1 = x - y / std::tan(angle);
        const double tangent2 = y / std::sin(angle);
        const double end_heading = row.heading_rad + angle * turn;
        const Point2D midpoint = add(
            row.point, scale(direction(row.heading_rad), tangent1));
        result.point = add(midpoint, scale(direction(end_heading), tangent2));
        result.heading_rad = end_heading;
        result.radius_m = length_m > 0.0
            ? parameter_a * parameter_a / length_m
            : 1.0e40;
    } else if (row.type == 4) {
        const double parameter_a = row.parameter1;
        const double radius = row.parameter2;
        const double total_length = parameter_a * parameter_a / radius;
        const double remaining = total_length - length_m;
        const double full_x = legacy_spiral_x(parameter_a, total_length);
        const double full_y = legacy_spiral_y(parameter_a, total_length);
        const double angle = remaining * remaining * 0.5 /
            radius / total_length;
        const double x = legacy_spiral_x(parameter_a, remaining);
        const double y = legacy_spiral_y(parameter_a, remaining);
        const double terminal_heading =
            row.heading_rad + turn * total_length * 0.5 / radius;
        result.heading_rad = terminal_heading - turn * angle;
        result.radius_m = remaining > 0.0
            ? parameter_a * parameter_a / remaining
            : 1.0e40;
        result.point.x = row.point.x +
            (x - full_x) * std::cos(terminal_heading + pi) +
            (y - full_y) * std::cos(
                terminal_heading + pi - turn * 0.5 * pi);
        result.point.y = row.point.y +
            (x - full_x) * std::sin(terminal_heading + pi) +
            (y - full_y) * std::sin(
                terminal_heading + pi - turn * 0.5 * pi);
    }
    if (result.heading_rad < 0.0) {
        result.heading_rad += 2.0 * pi;
    }
    if (result.heading_rad > 2.0 * pi) {
        result.heading_rad -= 2.0 * pi;
    }
    return result;
}

LegacyHorizontalAlignment::State LegacyHorizontalAlignment::evaluate_station(
    double station_m) const {
    if (station_m < rows_.front().station_m + 0.0001) {
        return {rows_.front().point, rows_.front().heading_rad,
                rows_.front().radius_m};
    }
    if (station_m > rows_.back().station_m - 0.0001) {
        return {rows_.back().point, rows_.back().heading_rad,
                rows_.back().radius_m};
    }
    for (std::size_t index = 1; index + 1 < rows_.size(); ++index) {
        const Row& row = rows_[index];
        double length = row_length(row);
        if (index + 2 == rows_.size()) {
            length += 0.1;
        }
        if (station_m < row.station_m + length + 0.001 &&
            station_m > row.station_m - 0.001) {
            double local = station_m - row.station_m;
            if (local <= 0.0) {
                local = 1.0e-12;
            }
            return evaluate_row(row, local);
        }
    }
    return {};
}

double LegacyHorizontalAlignment::circle_station(
    const Row& row,
    const Point2D& point) const {
    const double radius = row.parameter1;
    const double chord = legacy_distance(point, row.point);
    double station = row.station_m +
        radius * 2.0 * std::asin(0.5 * chord / radius);
    const Point2D center = add(
        row.point,
        scale(direction(row.heading_rad + row.turn * 0.5 * pi), radius));
    const double first_heading = legacy_heading(center, row.point);
    const double point_heading = legacy_heading(center, point);
    if (legacy_turn(point_heading, first_heading) * row.turn < 0.0) {
        station = row.station_m + radius * 2.0 * pi -
            radius * 2.0 * std::asin(0.5 * chord / radius);
    }
    return station;
}

LegacyHorizontalProjection LegacyHorizontalAlignment::project(
    const Point2D& point) const {
    LegacyHorizontalProjection result;
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return result;
    }

    std::vector<std::size_t> candidates;
    Point2D boundary_end = add(
        rows_.front().point,
        scale(direction(rows_.front().heading_rad + pi), 1.0));
    for (std::size_t index = 1; index < rows_.size(); ++index) {
        const Point2D boundary_start = boundary_end;
        boundary_end = rows_[index].point;
        const Row& boundary = rows_[index];
        double turn = 1.0;
        if (boundary.type > 1 && boundary.type < 7) {
            turn = boundary.turn;
        }
        const double first_heading = index > 1
            ? rows_[index - 1].heading_rad
            : rows_.front().heading_rad;
        const double second_heading = boundary.heading_rad;
        if (index + 1 == rows_.size()) {
            boundary_end = add(boundary_end, direction(first_heading));
        }
        const Point2D first_normal = add(
            boundary_start,
            scale(direction(first_heading + turn * 0.5 * pi), 1000.0));
        const Point2D second_normal = add(
            boundary_end,
            scale(direction(second_heading + turn * 0.5 * pi), 1000.0));
        const double first_side = legacy_side(
            first_normal, boundary_start, point);
        const double second_side = legacy_side(
            second_normal, boundary_end, point);
        if (first_side * second_side < 0.0001 ||
            std::abs(first_side) < 0.00001 ||
            std::abs(second_side) < 0.00001) {
            candidates.push_back(index);
        }
    }
    if (candidates.empty()) {
        result.point = rows_.front().point;
        result.station_m = -1.0;
        result.distance_m = legacy_distance(result.point, point);
        result.element_type = 1;
        result.valid = true;
        return result;
    }

    double best_distance = std::numeric_limits<double>::infinity();
    for (const std::size_t boundary_index : candidates) {
        if (boundary_index == 0 || boundary_index >= rows_.size()) {
            continue;
        }
        const Row& row = rows_[boundary_index - 1];
        const Point2D start = boundary_index == 1
            ? add(rows_.front().point,
                  scale(direction(rows_.front().heading_rad + pi), 1.0))
            : row.point;
        const Point2D end = rows_[boundary_index].point;
        const double start_heading = boundary_index > 1
            ? row.heading_rad
            : rows_.front().heading_rad;
        double station = -1.0;
        if (row.type < 1) {
            continue;
        }
        if (row.type == 1) {
            const double from_start = legacy_distance(start, point);
            const double point_heading = legacy_heading(start, point);
            station = row.station_m + from_start *
                std::cos(legacy_turn(point_heading, start_heading));
        } else if (row.type == 2) {
            const double radius = row.parameter1;
            const Point2D center = add(
                row.point,
                scale(direction(row.heading_rad + row.turn * 0.5 * pi), radius));
            const double middle_length =
                0.5 * (rows_[boundary_index].station_m - row.station_m);
            const Point2D middle_test = add(
                start, scale(direction(start_heading), middle_length));
            double radial_heading = legacy_heading(center, point);
            const double first_side = legacy_side(start, center, middle_test);
            const double second_side = legacy_side(start, center, point);
            if (first_side * second_side < -0.001 ||
                std::abs(first_side) < 0.00001 ||
                std::abs(second_side) < 0.00001) {
                if (std::abs(second_side) < 0.001) {
                    station = row.station_m;
                } else {
                    radial_heading += pi;
                }
            }
            if (station < 0.0) {
                const Point2D projected = add(
                    center, scale(direction(radial_heading), radius));
                station = circle_station(row, projected);
            }
        } else {
            const double first_heading = legacy_heading(point, start);
            const double second_heading = legacy_heading(point, end);
            (void)second_heading;
            const double first_turn = legacy_turn(first_heading, start_heading);
            if (std::abs(first_turn - 0.5 * pi) < 0.00001) {
                const State state = evaluate_station(row.station_m);
                result.point = state.point;
                result.station_m = row.station_m;
                result.distance_m = legacy_distance(state.point, point);
                result.element_type = row.type;
                result.valid = true;
                return result;
            } else {
                const int side = std::abs(first_turn) > 0.5 * pi ? 1 : -1;
                double lower = row.station_m;
                double upper = rows_[boundary_index].station_m;
                do {
                    station = 0.5 * (lower + upper);
                    const State state = evaluate_station(station);
                    const double point_heading = legacy_heading(point, state.point);
                    const double turn = legacy_turn(
                        point_heading, state.heading_rad);
                    if ((side == 1 && std::abs(turn) > 0.5 * pi) ||
                        (side == -1 && std::abs(turn) < 0.5 * pi)) {
                        lower = station;
                    } else {
                        upper = station;
                    }
                } while (upper > -0.001 && lower > -0.001 &&
                         std::abs(upper - lower) > 0.00001);
            }
        }

        if (!std::isfinite(station)) {
            continue;
        }
        const State state = evaluate_station(station);
        const double candidate_distance = legacy_distance(state.point, point);
        if (candidate_distance < best_distance) {
            best_distance = candidate_distance;
            result.point = state.point;
            result.station_m = station;
            result.distance_m = candidate_distance;
            result.element_type = row.type;
            result.valid = true;
        }
    }
    if (result.valid && result.distance_m > 10000.0) {
        result.point = rows_.front().point;
        result.station_m = -1.0;
        result.distance_m = legacy_distance(result.point, point);
        result.element_type = 1;
    }
    return result;
}

} // namespace alignment_reconstruct::detail
