#pragma once

#include "alignment_reconstruct/types.h"

namespace legacy_reference {

struct State {
    alignment_reconstruct::Point2D point{};
    double heading_rad = 0.0;
};

double spiral_x(double parameter_a_m, double length_m);
double spiral_y(double parameter_a_m, double length_m);

State line(alignment_reconstruct::Point2D start, double start_heading_rad, double length_m);
State circle(alignment_reconstruct::Point2D start, double start_heading_rad,
             double radius_m, int turn_sign, double length_m);
State entry_spiral(alignment_reconstruct::Point2D start, double start_heading_rad,
                   double radius_m, double total_length_m, int turn_sign, double length_m);
State exit_spiral(alignment_reconstruct::Point2D start, double start_heading_rad,
                  double radius_m, double total_length_m, int turn_sign, double length_m);

} // namespace legacy_reference
