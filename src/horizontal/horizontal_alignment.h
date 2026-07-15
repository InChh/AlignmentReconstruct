#pragma once

#include "alignment_reconstruct/types.h"
#include "horizontal/horizontal_element.h"

#include <vector>

namespace alignment_reconstruct::detail {

struct HorizontalProjection {
    bool valid = false;
    double station_m = 0.0;
    HorizontalElementState state{};
    double signed_offset_m = 0.0;
    double distance_m = 0.0;
    HorizontalElementKind element_kind = HorizontalElementKind::straight;
};

class HorizontalAlignment {
public:
    HorizontalAlignment(
        const std::vector<HorizontalControlPoint>& control_points,
        double start_station_m);

    HorizontalElementState evaluate(double station_m) const;
    HorizontalProjection project(const Point2D& point) const;

    double start_station_m() const noexcept;
    double end_station_m() const noexcept;
    const std::vector<HorizontalElement>& elements() const noexcept;

private:
    std::vector<HorizontalElement> elements_;
    double start_station_m_ = 0.0;
    double end_station_m_ = 0.0;
};

} // namespace alignment_reconstruct::detail
