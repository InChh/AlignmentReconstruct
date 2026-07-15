#include "alignment_reconstruct/adjustment.h"
#include "alignment_reconstruct/reconstruction.h"

#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    using namespace alignment_reconstruct;

    std::vector<Point3D> measured_centerline;
    for (int index = 0; index <= 100; ++index) {
        const double station = 20.0 * static_cast<double>(index);
        measured_centerline.push_back({station, 0.0, 100.0 + 0.005 * station});
    }

    const ReconstructionResult reconstruction = reconstruct(measured_centerline);
    if (!reconstruction.ok()) {
        std::cerr << "Reconstruction failed: "
                  << reconstruction.diagnostics.front().message << '\n';
        return 1;
    }

    AlignmentDefinition alignment;
    alignment.horizontal = reconstruction.horizontal;
    alignment.vertical = reconstruction.vertical;

    const std::vector<Point3D> points_to_adjust{
        {500.0, 1.5, 102.48},
        {1500.0, -0.8, 107.53}
    };
    const AdjustmentCalculationResult adjustments =
        calculate_adjustments(points_to_adjust, alignment);
    if (!adjustments.ok()) {
        std::cerr << "Adjustment calculation failed: "
                  << adjustments.diagnostics.front().message << '\n';
        return 2;
    }

    std::cout << std::fixed << std::setprecision(3)
              << "horizontal controls: " << reconstruction.horizontal.size() << '\n'
              << "vertical controls: " << reconstruction.vertical.size() << '\n';
    for (const auto& point : adjustments.points) {
        std::cout << "point " << point.point_index
                  << ": station=" << point.station_m
                  << " m, offset=" << point.horizontal_offset_mm()
                  << " mm, lift/drop=" << point.lift_drop_mm() << " mm\n";
    }
    return 0;
}
