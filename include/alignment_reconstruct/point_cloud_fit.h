#pragma once

#include "alignment_reconstruct/types.h"

namespace alignment_reconstruct {

PointCloudFitResult fit_point_cloud(
    const std::vector<Point3D>& unordered_points,
    double interval_m);

} // namespace alignment_reconstruct
