#include "vertical/profile_classifier.h"

#include <cmath>
#include <stdexcept>

namespace alignment_reconstruct::detail {
namespace {

std::vector<ProfileClassificationRun> form_runs(const std::vector<bool>& curved) {
    std::vector<ProfileClassificationRun> runs;
    if (curved.empty()) {
        return runs;
    }
    std::size_t first = 0;
    for (std::size_t index = 1; index <= curved.size(); ++index) {
        if (index == curved.size() || curved[index] != curved[first]) {
            runs.push_back({curved[first], first, index - 1});
            first = index;
        }
    }
    return runs;
}

} // namespace

ProfileSampleAnalysis classify_profile_samples(
    const std::vector<ProfileSample>& samples,
    double threshold) {
    if (samples.size() < 2 || !std::isfinite(threshold) || threshold < 0.0) {
        throw std::invalid_argument("Invalid vertical profile classification input");
    }
    ProfileSampleAnalysis analysis;
    analysis.second_divided_difference.assign(samples.size(), 0.0);
    analysis.curved.assign(samples.size(), false);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        if (!std::isfinite(samples[index].station_m) ||
            !std::isfinite(samples[index].elevation_m)) {
            throw std::invalid_argument("Vertical sample contains non-finite data");
        }
        if (index > 0 && samples[index].station_m <= samples[index - 1].station_m) {
            throw std::invalid_argument("Vertical sample stations must be strictly increasing");
        }
    }
    for (std::size_t index = 1; index + 1 < samples.size(); ++index) {
        const double first_length = samples[index].station_m - samples[index - 1].station_m;
        const double second_length = samples[index + 1].station_m - samples[index].station_m;
        const double numerator =
            first_length * samples[index + 1].elevation_m -
            (first_length + second_length) * samples[index].elevation_m +
            second_length * samples[index - 1].elevation_m;
        const double denominator =
            first_length * second_length * (first_length + second_length);
        analysis.second_divided_difference[index] = numerator / denominator;
    }
    for (std::size_t index = 0; index < samples.size(); ++index) {
        analysis.curved[index] =
            std::abs(analysis.second_divided_difference[index]) >= threshold;
    }
    analysis.curved.front() = false;
    analysis.curved.back() = false;
    analysis.runs = form_runs(analysis.curved);
    return analysis;
}

} // namespace alignment_reconstruct::detail
