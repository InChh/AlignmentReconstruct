#pragma once

#include "alignment_reconstruct/types.h"

#include <cstddef>
#include <vector>

namespace alignment_reconstruct::detail {

struct LegacyHorizontalProjection {
    Point2D point{};
    double station_m = 0.0;
    double distance_m = 0.0;
    int element_type = 0;
    bool valid = false;
};

class LegacyHorizontalAlignment {
public:
    LegacyHorizontalAlignment(
        Point2D start,
        Point2D intersection,
        Point2D end,
        double radius_m,
        double entry_spiral_length_m,
        double exit_spiral_length_m);

    LegacyHorizontalProjection project(const Point2D& point) const;

private:
    struct Row {
        int type = 0;
        double parameter1 = 0.0;
        double parameter2 = 0.0;
        double parameter3 = 0.0;
        double turn = 0.0;
        double heading_rad = 0.0;
        double station_m = 0.0;
        double radius_m = 0.0;
        Point2D point{};
    };

    struct State {
        Point2D point{};
        double heading_rad = 0.0;
        double radius_m = 0.0;
    };

    State evaluate_row(const Row& row, double length_m) const;
    State evaluate_station(double station_m) const;
    double row_length(const Row& row) const;
    double circle_station(const Row& row, const Point2D& point) const;

    std::vector<Row> rows_;
};

} // namespace alignment_reconstruct::detail
