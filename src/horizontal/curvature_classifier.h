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

enum class HorizontalSampleKind {
    straight = 0,
    circular = 1,
    spiral = 2
};

struct HorizontalStraightSegment {
    std::size_t first_index = 0;
    std::size_t last_index = 0;
    std::size_t following_curve_sample_count = 0;
    std::size_t circular_first_index = 0;
    std::size_t circular_last_index = 0;
    std::size_t circular_sample_count = 0;
};

struct HorizontalSampleAnalysis {
    std::vector<double> cumulative_distance_m;
    std::vector<double> tangent_heading_rad;
    std::vector<double> curvature_inverse_m;
    std::vector<HorizontalCurvatureSample> curvature_graph;
    std::vector<bool> curved;
    std::vector<ClassificationRun> runs;
    std::vector<HorizontalSampleKind> sample_kind;
    std::vector<HorizontalStraightSegment> straight_segments;
    bool curves_erased_by_dynamic = false;
};

bool has_horizontal_curvature_support(
    const std::vector<double>& cumulative_distance_m,
    double window_m);

HorizontalSampleAnalysis classify_horizontal_samples(
    const std::vector<Point2D>& ordered_points,
    const ReconstructionOptions& options,
    const std::vector<HorizontalCurvatureSample>& curvature_graph = {});

} // namespace alignment_reconstruct::detail
