#include "horizontal/horizontal_element.h"

#include "geometry/vector_math.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace alignment_reconstruct::detail {
namespace {

Point2D integrate_euler(
    double start_heading_rad,
    double start_curvature_inverse_m,
    double curvature_rate_inverse_m2,
    double length_m) {
    if (length_m == 0.0) {
        return {};
    }
    if (std::abs(curvature_rate_inverse_m2) <= 1.0e-18) {
        if (std::abs(start_curvature_inverse_m) <= 1.0e-15) {
            return scale(direction(start_heading_rad), length_m);
        }
        const double end_heading = start_heading_rad + start_curvature_inverse_m * length_m;
        return {
            (std::sin(end_heading) - std::sin(start_heading_rad)) / start_curvature_inverse_m,
            (-std::cos(end_heading) + std::cos(start_heading_rad)) / start_curvature_inverse_m
        };
    }

    constexpr std::array<double, 8> nodes{
        0.09501250983763744, 0.28160355077925891,
        0.45801677765722739, 0.61787624440264375,
        0.75540440835500303, 0.86563120238783174,
        0.94457502307323258, 0.98940093499164993
    };
    constexpr std::array<double, 8> weights{
        0.18945061045506850, 0.18260341504492359,
        0.16915651939500254, 0.14959598881657673,
        0.12462897125553387, 0.09515851168249278,
        0.06225352393864789, 0.02715245941175409
    };

    const double heading_change =
        start_curvature_inverse_m * length_m +
        0.5 * curvature_rate_inverse_m2 * length_m * length_m;
    const int segment_count =
        std::max(1, static_cast<int>(std::ceil(std::abs(heading_change) / 0.25)));
    const double segment_length = length_m / static_cast<double>(segment_count);
    Point2D result{};
    for (int segment = 0; segment < segment_count; ++segment) {
        const double lower = static_cast<double>(segment) * segment_length;
        const double upper = lower + segment_length;
        const double midpoint = 0.5 * (lower + upper);
        const double half = 0.5 * (upper - lower);
        double integral_x = 0.0;
        double integral_y = 0.0;
        for (std::size_t index = 0; index < nodes.size(); ++index) {
            for (const double sign : {-1.0, 1.0}) {
                const double station = midpoint + sign * half * nodes[index];
                const double angle = start_heading_rad +
                    start_curvature_inverse_m * station +
                    0.5 * curvature_rate_inverse_m2 * station * station;
                integral_x += weights[index] * std::cos(angle);
                integral_y += weights[index] * std::sin(angle);
            }
        }
        result.x += half * integral_x;
        result.y += half * integral_y;
    }
    return result;
}

} // namespace

HorizontalElement::HorizontalElement(
    HorizontalElementKind kind,
    double start_station_m,
    Point2D start_point,
    double start_heading_rad,
    double length_m,
    double start_curvature_inverse_m,
    double end_curvature_inverse_m)
    : kind_(kind),
      start_station_m_(start_station_m),
      start_point_(start_point),
      start_heading_rad_(normalize_heading(start_heading_rad)),
      length_m_(length_m),
      start_curvature_inverse_m_(start_curvature_inverse_m),
      end_curvature_inverse_m_(end_curvature_inverse_m) {
    if (!std::isfinite(start_station_m_) ||
        !std::isfinite(start_point_.x) || !std::isfinite(start_point_.y) ||
        !std::isfinite(start_heading_rad_) || !std::isfinite(length_m_) ||
        !std::isfinite(start_curvature_inverse_m_) ||
        !std::isfinite(end_curvature_inverse_m_) || length_m_ < 0.0) {
        throw std::invalid_argument("Invalid horizontal element");
    }
}

HorizontalElementState HorizontalElement::evaluate(double distance_from_start_m) const {
    const double length = clamp(distance_from_start_m, 0.0, length_m_);
    const double curvature_rate = length_m_ > 0.0
        ? (end_curvature_inverse_m_ - start_curvature_inverse_m_) / length_m_
        : 0.0;
    HorizontalElementState state;
    state.point = add(
        start_point_,
        integrate_euler(start_heading_rad_, start_curvature_inverse_m_, curvature_rate, length));
    state.heading_rad = normalize_heading(
        start_heading_rad_ + start_curvature_inverse_m_ * length +
        0.5 * curvature_rate * length * length);
    state.curvature_inverse_m = start_curvature_inverse_m_ + curvature_rate * length;
    return state;
}

HorizontalElementKind HorizontalElement::kind() const noexcept { return kind_; }
double HorizontalElement::start_station_m() const noexcept { return start_station_m_; }
double HorizontalElement::end_station_m() const noexcept { return start_station_m_ + length_m_; }
double HorizontalElement::length_m() const noexcept { return length_m_; }
const Point2D& HorizontalElement::start_point() const noexcept { return start_point_; }
double HorizontalElement::start_heading_rad() const noexcept { return start_heading_rad_; }
double HorizontalElement::start_curvature_inverse_m() const noexcept { return start_curvature_inverse_m_; }
double HorizontalElement::end_curvature_inverse_m() const noexcept { return end_curvature_inverse_m_; }

} // namespace alignment_reconstruct::detail
