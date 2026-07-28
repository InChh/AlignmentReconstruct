#include "vertical/profile_classifier.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace alignment_reconstruct::detail {
namespace {

constexpr double classification_window_m = 15.0;

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
    if (samples.size() == 2) {
        analysis.runs = form_runs(analysis.curved);
        return analysis;
    }

    std::size_t first_calculated = 1;
    while (first_calculated + 1 < samples.size() &&
           samples[first_calculated].station_m -
                   samples.front().station_m <=
               classification_window_m) {
        ++first_calculated;
    }
    std::size_t last_calculated = samples.size() - 2;
    while (last_calculated > 0 &&
           samples.back().station_m -
                   samples[last_calculated].station_m <=
               classification_window_m) {
        --last_calculated;
    }

    if (first_calculated <= last_calculated) {
        for (std::size_t index = first_calculated;
             index <= last_calculated; ++index) {
            std::size_t previous = index;
            while (previous > 0) {
                --previous;
                if (samples[index].station_m -
                        samples[previous].station_m >
                    classification_window_m) {
                    break;
                }
            }

            std::size_t next = index + 1;
            while (next < samples.size() &&
                   samples[next].station_m -
                           samples[index].station_m <=
                       classification_window_m) {
                ++next;
            }
            if (next == samples.size()) {
                break;
            }

            const double first_length =
                samples[index].station_m -
                samples[previous].station_m;
            const double second_length =
                samples[next].station_m -
                samples[index].station_m;
            const double numerator =
                (second_length + first_length) *
                    samples[index].elevation_m -
                second_length * samples[previous].elevation_m -
                first_length * samples[next].elevation_m;
            const double denominator =
                first_length * second_length *
                (first_length + second_length);
            analysis.second_divided_difference[index] =
                std::abs(numerator * 100.0 / denominator);
        }
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
