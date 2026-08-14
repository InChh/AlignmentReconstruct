#pragma once

#include "alignment_reconstruct/types.h"
#include "horizontal/alignment_optimizer.h"

#include <vector>

namespace alignment_reconstruct::detail {

HorizontalCurveParameters apply_horizontal_output_options(
    HorizontalCurveParameters parameters,
    const ReconstructionOptions& options);

struct HorizontalReconstruction {
    std::vector<HorizontalControlPoint> controls;
    std::vector<HorizontalCurvatureSample> curvature_graph;
};

class HorizontalReconstructor {
public:
    explicit HorizontalReconstructor(ReconstructionOptions options);

    std::vector<HorizontalControlPoint> reconstruct(
        const std::vector<Point2D>& ordered_points) const;

    HorizontalReconstruction reconstruct_with_curvature_graph(
        const std::vector<Point2D>& ordered_points,
        const std::vector<HorizontalCurvatureSample>& curvature_graph = {}) const;

private:
    ReconstructionOptions options_;
};

} // namespace alignment_reconstruct::detail
