#pragma once

#include "alignment_reconstruct/types.h"

namespace alignment_reconstruct {

AdjustmentCalculationResult calculate_adjustments(
    const std::vector<Point3D>& points,
    const AlignmentDefinition& alignment);

} // namespace alignment_reconstruct
