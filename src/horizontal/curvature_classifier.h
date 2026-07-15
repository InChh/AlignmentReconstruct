#pragma once

#include "alignment_reconstruct/types.h"

#include <cstddef>
#include <vector>

namespace alignment_reconstruct::detail {

struct ClassificationRun {
    bool curved = false;
    std::size_t first_index = 0;
    std::size_t last_index = 0;
};

struct HorizontalSampleAnalysis {
    std::vector<double> cumulative_distance_m;
    std::vector<double> tangent_heading_rad;
    std::vector<double> curvature_inverse_m;
    std::vector<bool> curved;
    std::vector<ClassificationRun> runs;
};

HorizontalSampleAnalysis classify_horizontal_samples(
    const std::vector<Point2D>& ordered_points,
    const ReconstructionOptions& options);

} // namespace alignment_reconstruct::detail
