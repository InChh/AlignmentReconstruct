#pragma once

#include "alignment_reconstruct/types.h"

namespace alignment_reconstruct::detail {

struct HorizontalElementState {
    Point2D point{};
    double heading_rad = 0.0;
    double curvature_inverse_m = 0.0;
};

class HorizontalElement {
public:
    HorizontalElement(
        HorizontalElementKind kind,
        double start_station_m,
        Point2D start_point,
        double start_heading_rad,
        double length_m,
        double start_curvature_inverse_m,
        double end_curvature_inverse_m);

    HorizontalElementState evaluate(double distance_from_start_m) const;

    HorizontalElementKind kind() const noexcept;
    double start_station_m() const noexcept;
    double end_station_m() const noexcept;
    double length_m() const noexcept;
    const Point2D& start_point() const noexcept;
    double start_heading_rad() const noexcept;
    double start_curvature_inverse_m() const noexcept;
    double end_curvature_inverse_m() const noexcept;

private:
    HorizontalElementKind kind_;
    double start_station_m_;
    Point2D start_point_;
    double start_heading_rad_;
    double length_m_;
    double start_curvature_inverse_m_;
    double end_curvature_inverse_m_;
};

} // namespace alignment_reconstruct::detail
