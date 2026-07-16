#include "geometry/circle_fit.h"

#include "geometry/vector_math.h"

#include <cmath>

namespace alignment_reconstruct::detail {

CircleFitResult fit_circle_to_range(
    const std::vector<Point2D>& points,
    std::size_t first_index,
    std::size_t last_index) {
    CircleFitResult result;
    if (first_index > last_index || last_index >= points.size() ||
        last_index - first_index + 1 < 3) {
        return result;
    }

    const std::size_t count = last_index - first_index + 1;
    for (std::size_t index = first_index; index <= last_index; ++index) {
        result.center = add(result.center, points[index]);
    }
    result.center.x /= static_cast<double>(count);
    result.center.y /= static_cast<double>(count);

    double xx = 0.0;
    double yy = 0.0;
    double xy = 0.0;
    double x_radius_squared = 0.0;
    double y_radius_squared = 0.0;
    double radius_squared_sum = 0.0;
    for (std::size_t index = first_index; index <= last_index; ++index) {
        const Point2D delta = subtract(points[index], result.center);
        const double radius_squared = dot(delta, delta);
        xx += delta.x * delta.x;
        yy += delta.y * delta.y;
        xy += delta.x * delta.y;
        x_radius_squared += delta.x * radius_squared;
        y_radius_squared += delta.y * radius_squared;
        radius_squared_sum += radius_squared;
    }
    const double determinant = xx * yy - xy * xy;
    if (std::abs(determinant) <= 1.0e-18) {
        return result;
    }
    const Point2D local_center{
        (yy * x_radius_squared / 2.0 - xy * y_radius_squared / 2.0) /
            determinant,
        (xx * y_radius_squared / 2.0 - xy * x_radius_squared / 2.0) /
            determinant
    };
    result.center = add(result.center, local_center);
    result.radius_m = std::sqrt(
        dot(local_center, local_center) +
        radius_squared_sum / static_cast<double>(count));
    result.valid = std::isfinite(result.radius_m) && result.radius_m > 0.0;
    return result;
}

} // namespace alignment_reconstruct::detail
