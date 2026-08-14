#pragma once

#include "alignment_reconstruct/api.h"
#include "alignment_reconstruct/types.h"

namespace alignment_reconstruct {

ALIGNMENT_RECONSTRUCT_API ReconstructionResult reconstruct(
    const std::vector<Point3D>& ordered_points,
    const ReconstructionOptions& options = {},
    const std::vector<HorizontalCurvatureSample>& curvature_graph = {});

} // namespace alignment_reconstruct
