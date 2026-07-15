#include "alignment_reconstruct/types.h"

namespace alignment_reconstruct {

bool ReconstructionResult::ok() const noexcept {
    return status == StatusCode::success;
}

double PointAdjustment::horizontal_offset_mm() const noexcept {
    return horizontal_offset_m * 1000.0;
}

double PointAdjustment::lift_drop_mm() const noexcept {
    return lift_drop_m * 1000.0;
}

bool AdjustmentCalculationResult::ok() const noexcept {
    return status == StatusCode::success;
}

} // namespace alignment_reconstruct
