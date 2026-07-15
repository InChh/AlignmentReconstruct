#include "geometry/vector_math.h"

#include <algorithm>
#include <cmath>

namespace alignment_reconstruct::detail {

Point2D add(const Point2D& left, const Point2D& right) noexcept {
    return {left.x + right.x, left.y + right.y};
}

Point2D subtract(const Point2D& left, const Point2D& right) noexcept {
    return {left.x - right.x, left.y - right.y};
}

Point2D scale(const Point2D& value, double factor) noexcept {
    return {value.x * factor, value.y * factor};
}

double dot(const Point2D& left, const Point2D& right) noexcept {
    return left.x * right.x + left.y * right.y;
}

double cross(const Point2D& left, const Point2D& right) noexcept {
    return left.x * right.y - left.y * right.x;
}

double norm(const Point2D& value) noexcept {
    return std::hypot(value.x, value.y);
}

double distance(const Point2D& left, const Point2D& right) noexcept {
    return norm(subtract(left, right));
}

Point2D direction(double heading_rad) noexcept {
    return {std::cos(heading_rad), std::sin(heading_rad)};
}

double heading(const Point2D& from, const Point2D& to) noexcept {
    return normalize_heading(std::atan2(to.y - from.y, to.x - from.x));
}

double normalize_heading(double angle_rad) noexcept {
    const double full_turn = 2.0 * pi;
    angle_rad = std::fmod(angle_rad, full_turn);
    if (angle_rad < 0.0) {
        angle_rad += full_turn;
    }
    return angle_rad;
}

double normalize_signed_angle(double angle_rad) noexcept {
    angle_rad = normalize_heading(angle_rad);
    if (angle_rad > pi) {
        angle_rad -= 2.0 * pi;
    }
    return angle_rad;
}

double clamp(double value, double lower, double upper) noexcept {
    return std::max(lower, std::min(value, upper));
}

} // namespace alignment_reconstruct::detail
