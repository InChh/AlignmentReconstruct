#include "alignment_reconstruct/adjustment.h"
#include "alignment_reconstruct/point_cloud_fit.h"
#include "alignment_reconstruct/reconstruction.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    using namespace alignment_reconstruct;

    constexpr double heading_rad = 0.2;
    const double direction_x = std::cos(heading_rad);
    const double direction_y = std::sin(heading_rad);
    std::vector<Point3D> unordered_point_cloud;
    for (int index = 0; index <= 2000; ++index) {
        const double station = static_cast<double>(index);
        const double x = station * direction_x;
        const double y = station * direction_y;
        const double z = 100.0 + 0.005 * station;
        unordered_point_cloud.push_back({x, y - 0.4, z - 0.2});
        unordered_point_cloud.push_back({x, y, z});
        unordered_point_cloud.push_back({x, y + 0.4, z + 0.2});
    }
    std::reverse(unordered_point_cloud.begin(), unordered_point_cloud.end());

    const PointCloudFitResult point_cloud_fit =
        fit_point_cloud(unordered_point_cloud, 20.0);
    if (!point_cloud_fit.ok()) {
        std::cerr << "Point-cloud fitting failed: "
                  << point_cloud_fit.diagnostics.front().message << '\n';
        return 1;
    }

    const ReconstructionResult reconstruction =
        reconstruct(point_cloud_fit.centerline_points);
    if (!reconstruction.ok()) {
        std::cerr << "Reconstruction failed: "
                  << reconstruction.diagnostics.front().message << '\n';
        return 2;
    }

    AlignmentDefinition alignment;
    alignment.horizontal = reconstruction.horizontal;
    alignment.vertical = reconstruction.vertical;

    const auto offset_point = [=](double station_m, double offset_m, double z) {
        return Point3D{
            station_m * direction_x - offset_m * direction_y,
            station_m * direction_y + offset_m * direction_x,
            z
        };
    };
    const std::vector<Point3D> points_to_adjust{
        offset_point(500.0, 1.5, 102.48),
        offset_point(1500.0, -0.8, 107.53)
    };
    const AdjustmentCalculationResult adjustments =
        calculate_adjustments(points_to_adjust, alignment);
    if (!adjustments.ok()) {
        std::cerr << "Adjustment calculation failed: "
                  << adjustments.diagnostics.front().message << '\n';
        return 3;
    }

    std::cout << std::fixed << std::setprecision(3)
              << "centerline samples: "
              << point_cloud_fit.centerline_points.size() << '\n'
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
