#pragma once

#include "alignment_reconstruct/types.h"

namespace alignment_reconstruct::detail {

inline constexpr double pi = 3.141592653589793238462643383279502884;

Point2D add(const Point2D& left, const Point2D& right) noexcept;
Point2D subtract(const Point2D& left, const Point2D& right) noexcept;
Point2D scale(const Point2D& value, double factor) noexcept;
double dot(const Point2D& left, const Point2D& right) noexcept;
double cross(const Point2D& left, const Point2D& right) noexcept;
double norm(const Point2D& value) noexcept;
double distance(const Point2D& left, const Point2D& right) noexcept;
Point2D direction(double heading_rad) noexcept;
double heading(const Point2D& from, const Point2D& to) noexcept;
double normalize_heading(double angle_rad) noexcept;
double normalize_signed_angle(double angle_rad) noexcept;
double clamp(double value, double lower, double upper) noexcept;

} // namespace alignment_reconstruct::detail
