#include "geometry/line_fit.h"

#include "geometry/vector_math.h"

#include <cmath>

namespace alignment_reconstruct::detail {

LineFitResult fit_orthogonal_line(const std::vector<Point2D>& points) {
    LineFitResult result;
    if (points.size() < 2) {
        return result;
    }

    for (const auto& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            return result;
        }
        result.centroid.x += point.x;
        result.centroid.y += point.y;
    }
    const double count = static_cast<double>(points.size());
    result.centroid.x /= count;
    result.centroid.y /= count;

    double xx = 0.0;
    double xy = 0.0;
    double yy = 0.0;
    for (const auto& point : points) {
        const Point2D delta = subtract(point, result.centroid);
        xx += delta.x * delta.x;
        xy += delta.x * delta.y;
        yy += delta.y * delta.y;
    }
    if (xx + yy <= 1.0e-24) {
        return result;
    }

    const double angle = 0.5 * std::atan2(2.0 * xy, xx - yy);
    result.unit_direction = direction(angle);
    if (dot(result.unit_direction, subtract(points.back(), points.front())) < 0.0) {
        result.unit_direction = scale(result.unit_direction, -1.0);
    }

    double squared_error = 0.0;
    for (const auto& point : points) {
        const double orthogonal = cross(result.unit_direction, subtract(point, result.centroid));
        squared_error += orthogonal * orthogonal;
    }
    result.rms_orthogonal_error_m = std::sqrt(squared_error / count);
    result.valid = true;
    return result;
}

} // namespace alignment_reconstruct::detail
