#pragma once

#include "alignment_reconstruct/types.h"
#include "geometry/line_fit.h"

#include <cstddef>
#include <vector>

namespace alignment_reconstruct::detail {

struct HorizontalCurveParameters {
    double radius_m = 0.0;
    double entry_spiral_length_m = 0.0;
    double exit_spiral_length_m = 0.0;
    double mean_squared_offset_m2 = 0.0;
};

HorizontalCurveParameters fit_horizontal_curve(
    const std::vector<Point2D>& points,
    std::size_t curve_first_index,
    std::size_t curve_last_index,
    std::size_t objective_first_index,
    std::size_t objective_last_index,
    const LineFitResult& incoming_line,
    const LineFitResult& outgoing_line,
    const Point2D& incoming_start,
    const Point2D& incoming_end,
    const Point2D& outgoing_start,
    const Point2D& outgoing_end,
    const Point2D& intersection,
    const ReconstructionOptions& options);

} // namespace alignment_reconstruct::detail
