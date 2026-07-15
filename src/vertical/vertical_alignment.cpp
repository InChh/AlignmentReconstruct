#include "vertical/vertical_alignment.h"

#include "geometry/vector_math.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace alignment_reconstruct::detail {
namespace {

constexpr double station_tolerance_m = 1.0e-8;

double linear_elevation(
    const VerticalControlPoint& point,
    double grade,
    double station_m) {
    return point.elevation_m + grade * (station_m - point.station_m);
}

} // namespace

VerticalAlignment::VerticalAlignment(
    const std::vector<VerticalControlPoint>& control_points)
    : control_points_(control_points) {
    if (control_points_.size() < 2) {
        throw std::invalid_argument("At least two vertical control points are required");
    }
    for (std::size_t index = 0; index < control_points_.size(); ++index) {
        const auto& point = control_points_[index];
        if (!std::isfinite(point.station_m) || !std::isfinite(point.elevation_m) ||
            !std::isfinite(point.vertical_curve_radius_m)) {
            throw std::invalid_argument("Vertical control point contains non-finite data");
        }
        if (point.vertical_curve_radius_m < 0.0 ||
            (point.vertical_curve_radius_m > 0.0 && point.vertical_curve_radius_m < 100.0)) {
            throw std::invalid_argument("Vertical radius must be zero or a physical radius >= 100 m");
        }
        if (index > 0 &&
            point.station_m <= control_points_[index - 1].station_m + station_tolerance_m) {
            throw std::invalid_argument("Vertical stations must be strictly increasing");
        }
    }

    grades_.reserve(control_points_.size() - 1);
    for (std::size_t index = 0; index + 1 < control_points_.size(); ++index) {
        const auto& first = control_points_[index];
        const auto& second = control_points_[index + 1];
        grades_.push_back(
            (second.elevation_m - first.elevation_m) /
            (second.station_m - first.station_m));
    }

    for (std::size_t index = 1; index + 1 < control_points_.size(); ++index) {
        const double radius = control_points_[index].vertical_curve_radius_m;
        if (radius <= 0.0) {
            continue;
        }
        const double incoming_angle = std::atan(grades_[index - 1]);
        const double outgoing_angle = std::atan(grades_[index]);
        const double signed_deflection = outgoing_angle - incoming_angle;
        if (std::abs(signed_deflection) <= 1.0e-15) {
            continue;
        }
        const double tangent_length = radius * std::tan(0.5 * std::abs(signed_deflection));
        const auto& intersection = control_points_[index];
        CircularCurve curve;
        curve.control_index = index;
        curve.start_station_m = intersection.station_m -
            tangent_length * std::cos(incoming_angle);
        curve.end_station_m = intersection.station_m +
            tangent_length * std::cos(outgoing_angle);
        const double start_elevation = intersection.elevation_m -
            tangent_length * std::sin(incoming_angle);
        curve.radius_m = radius;
        curve.turn_sign = signed_deflection > 0.0 ? 1 : -1;
        curve.center_station_m = curve.start_station_m -
            static_cast<double>(curve.turn_sign) * radius * std::sin(incoming_angle);
        curve.center_elevation_m = start_elevation +
            static_cast<double>(curve.turn_sign) * radius * std::cos(incoming_angle);
        if (curve.start_station_m < control_points_[index - 1].station_m - station_tolerance_m ||
            curve.end_station_m > control_points_[index + 1].station_m + station_tolerance_m) {
            throw std::invalid_argument("Vertical curve extends beyond adjacent control points");
        }
        if (!curves_.empty() &&
            curve.start_station_m < curves_.back().end_station_m - station_tolerance_m) {
            throw std::invalid_argument("Vertical curves overlap");
        }
        curves_.push_back(curve);
    }
}

VerticalAlignmentState VerticalAlignment::evaluate(double station_m) const {
    if (!std::isfinite(station_m)) {
        throw std::invalid_argument("Vertical evaluation station is non-finite");
    }
    const double station = clamp(station_m, start_station_m(), end_station_m());
    for (const auto& curve : curves_) {
        if (station >= curve.start_station_m - station_tolerance_m &&
            station <= curve.end_station_m + station_tolerance_m) {
            const double horizontal = station - curve.center_station_m;
            const double radicand = std::max(
                0.0, curve.radius_m * curve.radius_m - horizontal * horizontal);
            const double vertical = std::sqrt(radicand);
            VerticalAlignmentState state;
            state.elevation_m = curve.center_elevation_m -
                static_cast<double>(curve.turn_sign) * vertical;
            state.grade = static_cast<double>(curve.turn_sign) * horizontal /
                std::max(vertical, 1.0e-15);
            state.on_vertical_curve = true;
            return state;
        }
    }

    const auto upper = std::upper_bound(
        control_points_.begin(), control_points_.end(), station,
        [](double value, const VerticalControlPoint& point) {
            return value < point.station_m;
        });
    std::size_t segment_index = 0;
    if (upper == control_points_.begin()) {
        segment_index = 0;
    } else if (upper == control_points_.end()) {
        segment_index = grades_.size() - 1;
    } else {
        segment_index = static_cast<std::size_t>(upper - control_points_.begin() - 1);
    }
    return {
        linear_elevation(control_points_[segment_index], grades_[segment_index], station),
        grades_[segment_index],
        false
    };
}

double VerticalAlignment::start_station_m() const noexcept {
    return control_points_.front().station_m;
}

double VerticalAlignment::end_station_m() const noexcept {
    return control_points_.back().station_m;
}

} // namespace alignment_reconstruct::detail
