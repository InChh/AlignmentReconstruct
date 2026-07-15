#include "test_framework.h"

#include "legacy_reference/legacy_vertical_geometry.h"
#include "vertical/profile_classifier.h"
#include "vertical/vertical_reconstructor.h"

#include <vector>

using namespace alignment_reconstruct;

AR_TEST(profile_second_divided_difference_supports_nonuniform_stations) {
    const std::vector<detail::ProfileSample> samples{
        {0.0, 0.0}, {10.0, 100.0}, {30.0, 900.0}, {60.0, 3600.0}
    };
    const auto analysis = detail::classify_profile_samples(samples, 0.5);
    AR_EXPECT_NEAR(analysis.second_divided_difference[1], 1.0, 1.0e-12);
    AR_EXPECT_NEAR(analysis.second_divided_difference[2], 1.0, 1.0e-12);
    AR_EXPECT(analysis.curved[1]);
    AR_EXPECT(analysis.curved[2]);
}

AR_TEST(vertical_reconstruction_recovers_grades_intersections_and_radii) {
    const std::vector<VerticalControlPoint> expected{
        {0.0, 100.0, 0.0},
        {1200.0, 112.0, 12000.0},
        {2600.0, 105.0, 9000.0},
        {4000.0, 115.0, 0.0}
    };
    std::vector<detail::ProfileSample> samples;
    for (double station = 0.0; station <= 4000.0; station += 10.0) {
        samples.push_back({
            station,
            legacy_reference::vertical_elevation(expected, station)
        });
    }
    ReconstructionOptions options;
    options.vertical_second_difference_threshold_candidates = {1.0e-6};
    options.minimum_grade_length_m = 400.0;
    options.grade_rounding_per_mille = 0.0;
    const detail::VerticalReconstructor reconstructor(options);
    const auto actual = reconstructor.reconstruct(samples);
    AR_EXPECT(actual.size() == expected.size());
    for (std::size_t index = 0; index < actual.size(); ++index) {
        AR_EXPECT_NEAR(actual[index].station_m, expected[index].station_m, 10.0);
        AR_EXPECT_NEAR(actual[index].elevation_m, expected[index].elevation_m, 0.05);
    }
    AR_EXPECT_NEAR(actual[1].vertical_curve_radius_m, 12000.0, 200.0);
    AR_EXPECT_NEAR(actual[2].vertical_curve_radius_m, 9000.0, 200.0);
}

AR_TEST(vertical_reconstruction_preserves_a_sharp_zero_radius_grade_break) {
    std::vector<detail::ProfileSample> samples;
    for (int index = 0; index <= 200; ++index) {
        const double station = 10.0 * static_cast<double>(index);
        const double elevation = station <= 1000.0
            ? 100.0 + 0.01 * station
            : 110.0 - 0.005 * (station - 1000.0);
        samples.push_back({station, elevation});
    }
    ReconstructionOptions options;
    options.vertical_second_difference_threshold_candidates = {1.0e-6};
    options.grade_rounding_per_mille = 0.0;
    const detail::VerticalReconstructor reconstructor(options);
    const auto actual = reconstructor.reconstruct(samples);
    AR_EXPECT(actual.size() == 3);
    AR_EXPECT_NEAR(actual[1].station_m, 1000.0, 1.0e-9);
    AR_EXPECT_NEAR(actual[1].elevation_m, 110.0, 1.0e-9);
    AR_EXPECT_NEAR(actual[1].vertical_curve_radius_m, 0.0, 0.0);
}
