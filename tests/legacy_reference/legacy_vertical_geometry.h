#pragma once

#include "alignment_reconstruct/types.h"

#include <vector>

namespace legacy_reference {

double vertical_elevation(
    const std::vector<alignment_reconstruct::VerticalControlPoint>& control_points,
    double station_m);

} // namespace legacy_reference
