#pragma once

#include "geometry/line_fit.h"

#include <vector>

namespace alignment_reconstruct::detail {

struct LegacyLineFitResult {
    LineFitResult line{};
    Point2D start{};
    Point2D end{};
    bool valid = false;
};

LegacyLineFitResult fit_legacy_reconstruction_line(
    const std::vector<Point2D>& points);

Point2D project_to_legacy_line(
    const Point2D& point,
    const LegacyLineFitResult& line);

double legacy_signed_distance_to_line(
    const Point2D& point,
    const LegacyLineFitResult& line);

bool intersect_legacy_lines(
    const LegacyLineFitResult& first,
    const LegacyLineFitResult& second,
    Point2D& intersection);

} // namespace alignment_reconstruct::detail
