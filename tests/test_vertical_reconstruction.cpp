#include "test_framework.h"

#include "legacy_reference/legacy_vertical_geometry.h"
#include "vertical/profile_classifier.h"
#include "vertical/vertical_reconstructor.h"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace alignment_reconstruct;

namespace {

std::string environment_value(const char* name) {
    char* raw_value = nullptr;
    std::size_t value_size = 0;
    if (_dupenv_s(&raw_value, &value_size, name) != 0 ||
        raw_value == nullptr) {
        return {};
    }
    const std::string value(raw_value);
    std::free(raw_value);
    return value;
}

} // namespace

AR_TEST(profile_second_divided_difference_matches_rebuild_window_and_scale) {
    const std::vector<detail::ProfileSample> samples{
        {0.0, 0.0},
        {10.0, 100.0},
        {30.0, 900.0},
        {60.0, 3600.0},
        {90.0, 8100.0}
    };
    const auto analysis = detail::classify_profile_samples(samples, 50.0);
    AR_EXPECT_NEAR(analysis.second_divided_difference[1], 0.0, 0.0);
    AR_EXPECT_NEAR(analysis.second_divided_difference[2], 100.0, 1.0e-12);
    AR_EXPECT_NEAR(analysis.second_divided_difference[3], 100.0, 1.0e-12);
    AR_EXPECT(!analysis.curved[1]);
    AR_EXPECT(analysis.curved[2]);
    AR_EXPECT(analysis.curved[3]);
}

AR_TEST(empty_vertical_threshold_candidates_use_the_rebuild_default) {
    const std::vector<detail::ProfileSample> samples{
        {0.0, 100.0},
        {100.0, 100.4},
        {200.0, 100.8},
        {300.0, 101.2},
        {400.0, 101.6}
    };
    ReconstructionOptions empty_options;
    empty_options.vertical_second_difference_threshold_candidates.clear();
    ReconstructionOptions explicit_options;
    explicit_options.vertical_second_difference_threshold_candidates = {
        0.001
    };
    const auto fallback =
        detail::VerticalReconstructor(empty_options).reconstruct(samples);
    const auto explicit_default =
        detail::VerticalReconstructor(explicit_options).reconstruct(samples);
    AR_EXPECT(fallback.size() == explicit_default.size());
    for (std::size_t index = 0; index < fallback.size(); ++index) {
        AR_EXPECT_NEAR(
            fallback[index].station_m,
            explicit_default[index].station_m,
            0.0);
        AR_EXPECT_NEAR(
            fallback[index].elevation_m,
            explicit_default[index].elevation_m,
            0.0);
        AR_EXPECT_NEAR(
            fallback[index].vertical_curve_radius_m,
            explicit_default[index].vertical_curve_radius_m,
            0.0);
    }
}

AR_TEST(rebuild_path_rounds_every_internal_slope_station) {
    std::vector<detail::ProfileSample> samples;
    constexpr double intersection_station_m = 1003.25;
    for (int index = 0; index <= 20; ++index) {
        const double station_m = 50.0 * static_cast<double>(index);
        samples.push_back({
            station_m,
            100.0 + 0.004 * station_m
        });
    }
    samples.push_back({
        intersection_station_m,
        100.0 + 0.004 * intersection_station_m
    });
    for (int index = 21; index <= 40; ++index) {
        const double station_m = 50.0 * static_cast<double>(index);
        samples.push_back({
            station_m,
            100.0 + 0.004 * intersection_station_m -
                0.006 * (station_m - intersection_station_m)
        });
    }

    ReconstructionOptions options;
    options.vertical_second_difference_threshold_candidates = {1.0e-8};
    options.minimum_grade_length_m = 0.0;
    options.grade_rounding_per_mille = 0.0;
    options.station_rounding_m = 10.0;
    const auto actual =
        detail::VerticalReconstructor(options).reconstruct(samples);

    AR_EXPECT(actual.size() == 3);
    AR_EXPECT_NEAR(
        actual[1].station_m,
        1000.0,
        1.0e-8);
}

AR_TEST(full_coordinate_output_refreshes_the_last_elevation) {
    std::vector<detail::ProfileSample> samples;
    constexpr double intersection_station_m = 1003.25;
    constexpr double incoming_grade = 0.004;
    constexpr double outgoing_grade = 0.006006;
    for (int index = 0; index <= 20; ++index) {
        const double station_m = 50.0 * static_cast<double>(index);
        samples.push_back({
            station_m,
            100.0 + incoming_grade * station_m
        });
    }
    samples.push_back({
        intersection_station_m,
        100.0 + incoming_grade * intersection_station_m
    });
    for (int index = 21; index <= 40; ++index) {
        const double station_m = 50.0 * static_cast<double>(index);
        samples.push_back({
            station_m,
            100.0 + incoming_grade * intersection_station_m +
                outgoing_grade *
                    (station_m - intersection_station_m)
        });
    }

    ReconstructionOptions options;
    options.vertical_second_difference_threshold_candidates = {1.0e-8};
    options.minimum_grade_length_m = 0.0;
    options.grade_rounding_per_mille = 0.01;
    options.station_rounding_m = 0.0;
    options.maximum_grade_difference_per_mille = 0.0;
    const auto actual =
        detail::VerticalReconstructor(options).reconstruct(samples);

    AR_EXPECT(actual.size() == 3);
    const double actual_incoming_grade_per_mille =
        1000.0 *
        (actual[1].elevation_m - actual[0].elevation_m) /
        (actual[1].station_m - actual[0].station_m);
    const double actual_outgoing_grade_per_mille =
        1000.0 *
        (actual[2].elevation_m - actual[1].elevation_m) /
        (actual[2].station_m - actual[1].station_m);
    AR_EXPECT_NEAR(actual_incoming_grade_per_mille, 4.0, 1.0e-10);
    AR_EXPECT_NEAR(actual_outgoing_grade_per_mille, 6.01, 1.0e-10);
}

AR_TEST(vertical_reconstruction_matches_active_legacy_grade_and_radius_fit) {
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
    options.maximum_up_grade = 0.0;
    options.maximum_down_grade = 0.0;
    options.maximum_grade_difference_per_mille = 0.0;
    options.minimum_vertical_curve_grade_difference_per_mille = 0.0;
    options.minimum_vertical_curve_radius_m = 1000.0;
    const detail::VerticalReconstructor reconstructor(options);
    const auto actual = reconstructor.reconstruct(samples);
    AR_EXPECT(actual.size() == expected.size());
    for (std::size_t index = 0; index < actual.size(); ++index) {
        AR_EXPECT_NEAR(actual[index].station_m, expected[index].station_m, 10.0);
        AR_EXPECT_NEAR(actual[index].elevation_m, expected[index].elevation_m, 0.05);
    }
    AR_EXPECT_NEAR(actual[1].vertical_curve_radius_m, 6400.0, 0.0);
    AR_EXPECT_NEAR(actual[2].vertical_curve_radius_m, 4700.0, 0.0);
}

AR_TEST(vertical_reconstruction_assigns_the_legacy_minimum_radius_to_a_grade_break) {
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
    options.maximum_up_grade = 0.0;
    options.maximum_down_grade = 0.0;
    options.maximum_grade_difference_per_mille = 0.0;
    options.minimum_vertical_curve_grade_difference_per_mille = 0.0;
    options.minimum_vertical_curve_radius_m = 1000.0;
    const detail::VerticalReconstructor reconstructor(options);
    const auto actual = reconstructor.reconstruct(samples);
    AR_EXPECT(actual.size() == 3);
    AR_EXPECT_NEAR(actual[1].station_m, 1000.0, 1.0e-9);
    AR_EXPECT_NEAR(actual[1].elevation_m, 110.0, 1.0e-9);
    AR_EXPECT_NEAR(actual[1].vertical_curve_radius_m, 1000.0, 0.0);
}

AR_TEST(vertical_reconstruction_moves_a_vertical_curve_clear_of_a_horizontal_spiral) {
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
    options.maximum_up_grade = 0.0;
    options.maximum_down_grade = 0.0;
    options.maximum_grade_difference_per_mille = 0.0;
    options.minimum_vertical_curve_grade_difference_per_mille = 0.0;
    options.minimum_vertical_curve_radius_m = 1000.0;
    const detail::VerticalReconstructor reconstructor(
        options, {{995.0, 1015.0}});
    const auto actual = reconstructor.reconstruct(samples);
    AR_EXPECT(actual.size() == 3);
    AR_EXPECT_NEAR(actual[1].station_m, 980.0, 1.0e-9);
    AR_EXPECT_NEAR(actual[1].elevation_m, 110.0, 1.0e-9);
}

AR_TEST(vertical_reconstruction_keeps_noisy_raw_line_intersections_feasible) {
    std::vector<detail::ProfileSample> samples;
    for (int index = 0; index <= 360; ++index) {
        const double station =
            20.0 * static_cast<double>(index) +
            (index % 7 == 0 ? 3.0 : 0.0);
        double elevation = 100.0;
        if (station <= 2200.0) {
            elevation += 0.004 * station;
        } else if (station <= 4700.0) {
            elevation += 0.004 * 2200.0 -
                0.006 * (station - 2200.0);
        } else {
            elevation += 0.004 * 2200.0 - 0.006 * 2500.0 +
                0.009 * (station - 4700.0);
        }
        elevation +=
            static_cast<double>((index * 37) % 13 - 6) * 0.0012;
        samples.push_back({station, elevation});
    }

    ReconstructionOptions options;
    options.vertical_second_difference_threshold_candidates = {5.0e-6};
    options.maximum_up_grade = 0.0;
    options.maximum_down_grade = 0.0;
    options.maximum_grade_difference_per_mille = 0.0;
    options.minimum_vertical_curve_grade_difference_per_mille = 0.0;
    options.minimum_vertical_curve_radius_m = 1000.0;
    const auto actual =
        detail::VerticalReconstructor(options).reconstruct(samples);
    AR_EXPECT(actual.size() == 4);
    AR_EXPECT_NEAR(actual[1].station_m, 2200.0, 1.0);
    AR_EXPECT_NEAR(actual[2].station_m, 4700.0, 1.0);
    AR_EXPECT_NEAR(actual[1].elevation_m, 108.8, 0.02);
    AR_EXPECT_NEAR(actual[2].elevation_m, 93.8, 0.02);
}

AR_TEST(vertical_reconstruction_runs_an_external_profile_fixture) {
    const std::string input_path = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_PROFILE_CSV");
    if (input_path.empty()) {
        test_support::skip(
            "External vertical profile fixture was not requested");
    }
    std::ifstream input(input_path);
    AR_EXPECT(input.good());
    std::string row;
    std::getline(input, row);
    std::vector<detail::ProfileSample> samples;
    while (std::getline(input, row)) {
        std::istringstream fields(row);
        std::string ignored_index;
        std::string station;
        std::string elevation;
        std::getline(fields, ignored_index, ',');
        std::getline(fields, station, ',');
        std::getline(fields, elevation, ',');
        samples.push_back({std::stod(station), std::stod(elevation)});
    }

    ReconstructionOptions options;
    const std::string minimum_length = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_MINIMUM_GRADE_LENGTH");
    if (!minimum_length.empty()) {
        options.minimum_grade_length_m = std::stod(minimum_length);
    }
    const std::string threshold = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_VERTICAL_THRESHOLD");
    if (!threshold.empty()) {
        options.vertical_second_difference_threshold_candidates = {
            std::stod(threshold)
        };
    }
    const std::string minimum_radius = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_MINIMUM_VERTICAL_RADIUS");
    if (!minimum_radius.empty()) {
        options.minimum_vertical_curve_radius_m =
            std::stod(minimum_radius);
    }
    const std::string maximum_radius = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_MAXIMUM_VERTICAL_RADIUS");
    if (!maximum_radius.empty()) {
        options.maximum_vertical_curve_radius_m =
            std::stod(maximum_radius);
    }
    const std::string minimum_curve_grade_difference = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_MINIMUM_VERTICAL_GRADE_DIFFERENCE");
    if (!minimum_curve_grade_difference.empty()) {
        options.minimum_vertical_curve_grade_difference_per_mille =
            std::stod(minimum_curve_grade_difference);
    }
    const std::string maximum_grade_difference = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_MAXIMUM_GRADE_DIFFERENCE");
    if (!maximum_grade_difference.empty()) {
        options.maximum_grade_difference_per_mille =
            std::stod(maximum_grade_difference);
    }
    const std::string maximum_up_grade = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_MAXIMUM_UP_GRADE");
    if (!maximum_up_grade.empty()) {
        options.maximum_up_grade = std::stod(maximum_up_grade);
    }
    const std::string maximum_down_grade = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_MAXIMUM_DOWN_GRADE");
    if (!maximum_down_grade.empty()) {
        options.maximum_down_grade = std::stod(maximum_down_grade);
    }
    const std::string station_rounding = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_STATION_ROUNDING");
    if (!station_rounding.empty()) {
        options.station_rounding_m = std::stod(station_rounding);
    }
    const std::string grade_rounding = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_GRADE_ROUNDING");
    if (!grade_rounding.empty()) {
        options.grade_rounding_per_mille = std::stod(grade_rounding);
    }
    const auto controls =
        detail::VerticalReconstructor(options).reconstruct(samples);
    AR_EXPECT(controls.size() >= 2);

    const std::string output_path = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_VERTICAL_OUTPUT");
    if (!output_path.empty()) {
        std::ofstream output(output_path);
        AR_EXPECT(output.good());
        output
            << "control_index,station_m,elevation_m,"
               "vertical_curve_radius_m\n";
        output << std::setprecision(17);
        for (std::size_t index = 0; index < controls.size(); ++index) {
            output << index << ','
                   << controls[index].station_m << ','
                   << controls[index].elevation_m << ','
                   << controls[index].vertical_curve_radius_m << '\n';
        }
    }
    std::cout << "[METRIC] external vertical profile: samples="
              << samples.size() << ", controls=" << controls.size() << '\n';
}
