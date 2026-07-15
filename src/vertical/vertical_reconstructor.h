#pragma once

#include "alignment_reconstruct/types.h"
#include "vertical/profile_classifier.h"

#include <vector>

namespace alignment_reconstruct::detail {

class VerticalReconstructor {
public:
    explicit VerticalReconstructor(ReconstructionOptions options);

    std::vector<VerticalControlPoint> reconstruct(
        const std::vector<ProfileSample>& ordered_samples) const;

private:
    ReconstructionOptions options_;
};

} // namespace alignment_reconstruct::detail
