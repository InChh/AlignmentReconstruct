#pragma once

#include "alignment_reconstruct/types.h"

#include <vector>

namespace alignment_reconstruct::detail {

class HorizontalReconstructor {
public:
    explicit HorizontalReconstructor(ReconstructionOptions options);

    std::vector<HorizontalControlPoint> reconstruct(
        const std::vector<Point2D>& ordered_points) const;

private:
    ReconstructionOptions options_;
};

} // namespace alignment_reconstruct::detail
