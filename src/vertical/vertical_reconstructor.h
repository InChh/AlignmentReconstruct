#pragma once

#include "alignment_reconstruct/types.h"
#include "vertical/profile_classifier.h"

#include <vector>

namespace alignment_reconstruct::detail {

struct HorizontalSpiralInterval {
    double start_station_m = 0.0;
    double end_station_m = 0.0;
};

class VerticalReconstructor {
public:
    explicit VerticalReconstructor(
        ReconstructionOptions options,
        std::vector<HorizontalSpiralInterval> horizontal_spirals = {});

    std::vector<VerticalControlPoint> reconstruct(
        const std::vector<ProfileSample>& ordered_samples) const;

private:
    ReconstructionOptions options_;
    std::vector<HorizontalSpiralInterval> horizontal_spirals_;
};

} // namespace alignment_reconstruct::detail
