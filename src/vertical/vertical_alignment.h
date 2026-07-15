#pragma once

#include "alignment_reconstruct/types.h"

#include <vector>

namespace alignment_reconstruct::detail {

struct VerticalAlignmentState {
    double elevation_m = 0.0;
    double grade = 0.0;
    bool on_vertical_curve = false;
};

class VerticalAlignment {
public:
    explicit VerticalAlignment(const std::vector<VerticalControlPoint>& control_points);

    VerticalAlignmentState evaluate(double station_m) const;
    double start_station_m() const noexcept;
    double end_station_m() const noexcept;

private:
    struct CircularCurve {
        std::size_t control_index = 0;
        double start_station_m = 0.0;
        double end_station_m = 0.0;
        double center_station_m = 0.0;
        double center_elevation_m = 0.0;
        double radius_m = 0.0;
        int turn_sign = 0;
    };

    std::vector<VerticalControlPoint> control_points_;
    std::vector<double> grades_;
    std::vector<CircularCurve> curves_;
};

} // namespace alignment_reconstruct::detail
