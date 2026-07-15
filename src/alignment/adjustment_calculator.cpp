#include "alignment/adjustment_calculator.h"

#include <cmath>

namespace alignment_reconstruct::detail {

AlignmentAdjustmentCalculator::AlignmentAdjustmentCalculator(
    const HorizontalAlignment& horizontal,
    const VerticalAlignment& vertical)
    : horizontal_(horizontal), vertical_(vertical) {}

AdjustmentCalculationResult AlignmentAdjustmentCalculator::calculate(
    const std::vector<Point3D>& points) const {
    AdjustmentCalculationResult result;
    result.points.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto& point = points[index];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            result.status = StatusCode::invalid_input;
            result.diagnostics.push_back({
                DiagnosticSeverity::error,
                DiagnosticCode::non_finite_value,
                index,
                "Measured point contains a non-finite coordinate"
            });
            result.points.clear();
            return result;
        }
        const auto projection = horizontal_.project({point.x, point.y});
        if (!projection.valid) {
            result.status = StatusCode::numerical_failure;
            result.diagnostics.push_back({
                DiagnosticSeverity::error,
                DiagnosticCode::projection_failed,
                index,
                "Horizontal projection failed"
            });
            result.points.clear();
            return result;
        }
        const auto vertical = vertical_.evaluate(projection.station_m);
        PointAdjustment adjustment;
        adjustment.point_index = index;
        adjustment.station_m = projection.station_m;
        adjustment.projected_point = projection.state.point;
        adjustment.design_elevation_m = vertical.elevation_m;
        adjustment.horizontal_offset_m = projection.signed_offset_m;
        adjustment.lift_drop_m = vertical.elevation_m - point.z;
        adjustment.horizontal_element = projection.element_kind;
        result.points.push_back(adjustment);
    }
    return result;
}

} // namespace alignment_reconstruct::detail
