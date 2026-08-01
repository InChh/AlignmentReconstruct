#pragma once

#include "alignment_reconstruct/api.h"
#include "alignment_reconstruct/types.h"

namespace alignment_reconstruct {

ALIGNMENT_RECONSTRUCT_API AdjustmentCalculationResult calculate_adjustments(
    const std::vector<Point3D>& points,
    const AlignmentDefinition& alignment);

} // namespace alignment_reconstruct
