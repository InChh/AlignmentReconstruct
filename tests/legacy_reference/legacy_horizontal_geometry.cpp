#include "legacy_horizontal_geometry.h"

#include <cmath>

namespace legacy_reference {
namespace {

constexpr double pi = 3.141592653589793238462643383279502884;

long factorial(int value) {
    long result = 1;
    for (int index = 1; index <= value; ++index) {
        result *= index;
    }
    return result;
}

} // namespace

double spiral_x(double parameter_a_m, double length_m) {
    if (parameter_a_m <= 0.0) {
        return 0.0;
    }
    const double c = 2.0 * parameter_a_m * parameter_a_m;
    double sum = 0.0;
    for (int index = 0; index < 10; ++index) {
        sum += std::pow(-1.0, index + 2) * std::pow(length_m, 4 * index + 1) /
            static_cast<double>(4 * index + 1) / std::pow(c, 2 * index) /
            static_cast<double>(factorial(2 * index));
    }
    return sum;
}

double spiral_y(double parameter_a_m, double length_m) {
    if (parameter_a_m <= 0.0) {
        return 0.0;
    }
    const double t = length_m * length_m / (2.0 * parameter_a_m * parameter_a_m);
    double sum = 0.0;
    for (int index = 1; index <= 10; ++index) {
        const double exponent = 0.5 * static_cast<double>(4 * index - 1);
        sum += std::pow(-1.0, index + 1) * std::pow(t, exponent) /
            exponent / static_cast<double>(factorial(2 * index - 1));
    }
    return sum * parameter_a_m / std::sqrt(2.0);
}

State line(alignment_reconstruct::Point2D start, double start_heading_rad, double length_m) {
    return {{start.x + length_m * std::cos(start_heading_rad),
             start.y + length_m * std::sin(start_heading_rad)}, start_heading_rad};
}

State circle(alignment_reconstruct::Point2D start, double start_heading_rad,
             double radius_m, int turn_sign, double length_m) {
    const double angle = length_m / radius_m;
    const double end_heading = start_heading_rad + static_cast<double>(turn_sign) * angle;
    const double tangent = radius_m * std::tan(0.5 * angle);
    const alignment_reconstruct::Point2D midpoint{
        start.x + tangent * std::cos(start_heading_rad),
        start.y + tangent * std::sin(start_heading_rad)};
    return {{midpoint.x + tangent * std::cos(end_heading),
             midpoint.y + tangent * std::sin(end_heading)}, end_heading};
}

State entry_spiral(alignment_reconstruct::Point2D start, double start_heading_rad,
                   double radius_m, double total_length_m, int turn_sign, double length_m) {
    const double parameter = std::sqrt(radius_m * total_length_m);
    const double local_x = spiral_x(parameter, length_m);
    const double local_y = spiral_y(parameter, length_m);
    const double side_heading = start_heading_rad + static_cast<double>(turn_sign) * 0.5 * pi;
    const double deflection = length_m * length_m / (2.0 * radius_m * total_length_m);
    return {{start.x + local_x * std::cos(start_heading_rad) + local_y * std::cos(side_heading),
             start.y + local_x * std::sin(start_heading_rad) + local_y * std::sin(side_heading)},
            start_heading_rad + static_cast<double>(turn_sign) * deflection};
}

State exit_spiral(alignment_reconstruct::Point2D start, double start_heading_rad,
                  double radius_m, double total_length_m, int turn_sign, double length_m) {
    const double parameter = std::sqrt(radius_m * total_length_m);
    const double remaining = total_length_m - length_m;
    const double full_x = spiral_x(parameter, total_length_m);
    const double full_y = spiral_y(parameter, total_length_m);
    const double local_x = spiral_x(parameter, remaining);
    const double local_y = spiral_y(parameter, remaining);
    const double terminal_heading = start_heading_rad +
        static_cast<double>(turn_sign) * total_length_m / (2.0 * radius_m);
    const double reverse_heading = terminal_heading + pi;
    const double reverse_side = reverse_heading - static_cast<double>(turn_sign) * 0.5 * pi;
    const double remaining_deflection = remaining * remaining /
        (2.0 * radius_m * total_length_m);
    return {{start.x + (local_x - full_x) * std::cos(reverse_heading) +
                         (local_y - full_y) * std::cos(reverse_side),
             start.y + (local_x - full_x) * std::sin(reverse_heading) +
                         (local_y - full_y) * std::sin(reverse_side)},
            terminal_heading - static_cast<double>(turn_sign) * remaining_deflection};
}

} // namespace legacy_reference
