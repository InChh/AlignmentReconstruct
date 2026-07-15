#pragma once

#include "alignment_reconstruct/types.h"

#include <vector>

namespace alignment_reconstruct::detail {

struct LineFitResult {
    Point2D centroid{};
    Point2D unit_direction{};
    double rms_orthogonal_error_m = 0.0;
    bool valid = false;
};

LineFitResult fit_orthogonal_line(const std::vector<Point2D>& points);

} // namespace alignment_reconstruct::detail
