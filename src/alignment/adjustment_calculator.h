#pragma once

#include "alignment_reconstruct/types.h"

#include "horizontal/horizontal_alignment.h"
#include "vertical/vertical_alignment.h"

#include <vector>

namespace alignment_reconstruct::detail {

class AlignmentAdjustmentCalculator {
public:
    AlignmentAdjustmentCalculator(
        const HorizontalAlignment& horizontal,
        const VerticalAlignment& vertical);

    AdjustmentCalculationResult calculate(const std::vector<Point3D>& points) const;

private:
    const HorizontalAlignment& horizontal_;
    const VerticalAlignment& vertical_;
};

} // namespace alignment_reconstruct::detail
