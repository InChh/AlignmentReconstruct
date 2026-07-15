#pragma once

#include "alignment_reconstruct/types.h"

namespace alignment_reconstruct {

ReconstructionResult reconstruct(
    const std::vector<Point3D>& ordered_points,
    const ReconstructionOptions& options = {});

} // namespace alignment_reconstruct
