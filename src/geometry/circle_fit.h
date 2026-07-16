#pragma once

#include "alignment_reconstruct/types.h"

#include <cstddef>
#include <vector>

namespace alignment_reconstruct::detail {

struct CircleFitResult {
    Point2D center{};
    double radius_m = 0.0;
    bool valid = false;
};

CircleFitResult fit_circle_to_range(
    const std::vector<Point2D>& points,
    std::size_t first_index,
    std::size_t last_index);

} // namespace alignment_reconstruct::detail
