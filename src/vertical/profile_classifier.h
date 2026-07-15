#pragma once

#include <cstddef>
#include <vector>

namespace alignment_reconstruct::detail {

struct ProfileSample {
    double station_m = 0.0;
    double elevation_m = 0.0;
};

struct ProfileClassificationRun {
    bool curved = false;
    std::size_t first_index = 0;
    std::size_t last_index = 0;
};

struct ProfileSampleAnalysis {
    std::vector<double> second_divided_difference;
    std::vector<bool> curved;
    std::vector<ProfileClassificationRun> runs;
};

ProfileSampleAnalysis classify_profile_samples(
    const std::vector<ProfileSample>& samples,
    double threshold);

} // namespace alignment_reconstruct::detail
