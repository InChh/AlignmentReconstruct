#pragma once

#include "alignment_reconstruct/api.h"
#include "alignment_reconstruct/types.h"

namespace alignment_reconstruct {

ALIGNMENT_RECONSTRUCT_API PointCloudFitResult fit_point_cloud(
    const std::vector<Point3D>& unordered_points,
    double interval_m);

} // namespace alignment_reconstruct
