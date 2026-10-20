#include "geometry/legacy_line_fit.h"

#include "geometry/vector_math.h"

#include <cmath>

namespace alignment_reconstruct::detail {
namespace {

constexpr double legacy_precision = 1.0e-6;

Point2D foot_point(
    double a,
    double b,
    double c,
    const Point2D& point) {
    return {
        point.x - a * (a * point.x + b * point.y + c) /
            (a * a + b * b),
        point.y - b * (a * point.x + b * point.y + c) /
            (a * a + b * b)
    };
}

double legacy_side(
    const Point2D& point,
    const Point2D& line_start,
    const Point2D& line_end) {
    const double center_x = (point.x + line_start.x + line_end.x) / 3.0;
    const double center_y = (point.y + line_start.y + line_end.y) / 3.0;
    const double x1 = point.x - center_x;
    const double x2 = line_start.x - center_x;
    const double x3 = line_end.x - center_x;
    const double y1 = point.y - center_y;
    const double y2 = line_start.y - center_y;
    const double y3 = line_end.y - center_y;
    const double value = x1 * (y2 - y3) - x2 * (y1 - y3) +
        x3 * (y1 - y2);
    return value < 0.0 ? -1.0 : 1.0;
}

} // namespace

LegacyLineFitResult fit_legacy_reconstruction_line(
    const std::vector<Point2D>& points) {
    LegacyLineFitResult result;
    if (points.size() < 2) {
        return result;
    }

    Point2D center{};
    for (const Point2D& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            return result;
        }
        center.x += point.x;
        center.y += point.y;
    }
    const double count = static_cast<double>(points.size());
    center.x /= count;
    center.y /= count;

    std::vector<Point2D> centered;
    centered.reserve(points.size());
    for (const Point2D& point : points) {
        centered.push_back(subtract(point, center));
    }

    double x = 0.0;
    double y = 0.0;
    double xx = 0.0;
    double xy = 0.0;
    double yy = 0.0;
    for (const Point2D& point : centered) {
        x += point.x;
        y += point.y;
        xx += point.x * point.x;
        xy += point.y * point.x;
        yy += point.y * point.y;
    }
    if (xx + yy <= 1.0e-24) {
        return result;
    }

    const double v = 2.0 * xy / (xx - yy);
    double theta = 0.0;
    if (xy >= 0.0 && v >= 0.0) {
        theta = std::atan(v) / 2.0;
    } else if (xy >= 0.0 && v < 0.0) {
        theta = (std::atan(v) + pi) / 2.0;
    } else if (xy <= 0.0 && v >= 0.0) {
        theta = (std::atan(v) + pi) / 2.0;
    } else {
        theta = std::atan(v) / 2.0 + pi;
    }

    // Zero cross covariance has an exact axis eigendirection. In particular,
    // zero slope is horizontal; it must not enter the legacy vertical branch.
    if (xy == 0.0) {
        if (xx >= yy) {
            result.start = {points.front().x, center.y};
            result.end = {points.back().x, center.y};
        } else {
            result.start = {center.x, points.front().y};
            result.end = {center.x, points.back().y};
        }
    } else {
        // Preserve the active reference arithmetic away from the exact axes.
        // Do not snap a near-vertical tangent to vertical: that changes its
        // direction and makes the result depend on the coordinate rotation.
        const double k = std::tan(theta);
        const double b = (y - k * x) / count;
        result.start = add(foot_point(k, -1.0, b, centered.front()), center);
        result.end = add(foot_point(k, -1.0, b, centered.back()), center);
    }

    const Point2D line_vector = subtract(result.end, result.start);
    const double line_length = norm(line_vector);
    if (!std::isfinite(line_length) || line_length <= 1.0e-12) {
        return result;
    }
    result.line.centroid = result.start;
    result.line.unit_direction = scale(line_vector, 1.0 / line_length);
    result.line.valid = true;
    result.valid = true;
    return result;
}

Point2D project_to_legacy_line(
    const Point2D& point,
    const LegacyLineFitResult& line) {
    if (std::abs(line.start.y - line.end.y) < legacy_precision) {
        return {point.x, (line.start.y + line.end.y) / 2.0};
    }
    if (std::abs(line.start.x - line.end.x) < legacy_precision) {
        return {(line.start.x + line.end.x) / 2.0, point.y};
    }

    const double k = (line.end.y - line.start.y) /
        (line.end.x - line.start.x);
    const double b = line.start.y - line.start.x * k;
    const double x = (point.y + point.x / k - b) / (k + 1.0 / k);
    return {x, k * x + b};
}

double legacy_signed_distance_to_line(
    const Point2D& point,
    const LegacyLineFitResult& line) {
    const Point2D foot = project_to_legacy_line(point, line);
    return distance(foot, point) * legacy_side(point, line.start, line.end);
}

bool intersect_legacy_lines(
    const LegacyLineFitResult& first,
    const LegacyLineFitResult& second,
    Point2D& intersection) {
    const double a11 = first.end.y - first.start.y;
    const double a12 = first.start.x - first.end.x;
    const double a21 = second.end.y - second.start.y;
    const double a22 = second.start.x - second.end.x;
    const double b1 = first.start.x * first.end.y -
        first.end.x * first.start.y;
    const double b2 = second.start.x * second.end.y -
        second.end.x * second.start.y;
    const double determinant = a11 * a22 - a21 * a12;
    if (std::abs(determinant) < 1.0e-6) {
        return false;
    }
    intersection.x = (b1 * a22 - b2 * a12) / determinant;
    intersection.y = (b2 * a11 - b1 * a21) / determinant;
    return true;
}

} // namespace alignment_reconstruct::detail
