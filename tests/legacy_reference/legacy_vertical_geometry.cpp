#include "legacy_vertical_geometry.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legacy_reference {
namespace {

std::vector<double> grades(
    const std::vector<alignment_reconstruct::VerticalControlPoint>& points) {
    std::vector<double> result;
    for (std::size_t index = 0; index + 1 < points.size(); ++index) {
        result.push_back((points[index + 1].elevation_m - points[index].elevation_m) /
                         (points[index + 1].station_m - points[index].station_m));
    }
    return result;
}

double circular_elevation(
    const std::vector<alignment_reconstruct::VerticalControlPoint>& points,
    const std::vector<double>& segment_grades,
    std::size_t index,
    double station_m) {
    const double grade_change = segment_grades[index] - segment_grades[index - 1];
    const int curve_sign = grade_change > 0.0 ? 1 : -1;
    const double radius = points[index].vertical_curve_radius_m;
    const double incoming_angle = std::atan(segment_grades[index - 1]);
    const double outgoing_angle = std::atan(segment_grades[index]);
    const double tangent = radius * std::tan(0.5 * std::abs(incoming_angle - outgoing_angle));
    const double start_station = points[index].station_m - tangent * std::cos(incoming_angle);
    const double start_elevation = points[index].elevation_m - tangent * std::sin(incoming_angle);
    const double center_station = start_station -
        static_cast<double>(curve_sign) * radius * std::sin(incoming_angle);
    const double center_elevation = start_elevation +
        static_cast<double>(curve_sign) * radius * std::cos(incoming_angle);
    const double vertical = std::sqrt(std::max(
        0.0, radius * radius - (station_m - center_station) * (station_m - center_station)));
    return center_elevation - static_cast<double>(curve_sign) * vertical;
}

} // namespace

double vertical_elevation(
    const std::vector<alignment_reconstruct::VerticalControlPoint>& control_points,
    double station_m) {
    if (control_points.size() < 2) {
        throw std::invalid_argument("legacy vertical reference needs two points");
    }
    const auto segment_grades = grades(control_points);
    station_m = std::max(control_points.front().station_m,
                         std::min(station_m, control_points.back().station_m));
    for (std::size_t index = 1; index + 1 < control_points.size(); ++index) {
        const double radius = control_points[index].vertical_curve_radius_m;
        if (radius < 100.0) {
            continue;
        }
        const double incoming_angle = std::atan(segment_grades[index - 1]);
        const double outgoing_angle = std::atan(segment_grades[index]);
        const double tangent = radius * std::tan(0.5 * std::abs(incoming_angle - outgoing_angle));
        const double start = control_points[index].station_m - tangent * std::cos(incoming_angle);
        const double end = control_points[index].station_m + tangent * std::cos(outgoing_angle);
        if (station_m >= start && station_m <= end) {
            return circular_elevation(control_points, segment_grades, index, station_m);
        }
    }
    std::size_t segment = 0;
    while (segment + 1 < control_points.size() - 1 &&
           station_m > control_points[segment + 1].station_m) {
        ++segment;
    }
    return control_points[segment].elevation_m +
        segment_grades[segment] * (station_m - control_points[segment].station_m);
}

} // namespace legacy_reference
