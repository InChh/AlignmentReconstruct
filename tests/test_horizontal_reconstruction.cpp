#include "fixture_reader.h"
#include "test_framework.h"

#include "geometry/vector_math.h"
#include "horizontal/curvature_classifier.h"
#include "horizontal/horizontal_alignment.h"
#include "horizontal/horizontal_reconstructor.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

using namespace alignment_reconstruct;

AR_TEST(horizontal_reconstruction_recovers_a_deterministic_spiral_curve) {
    const std::vector<HorizontalControlPoint> expected{
        {HorizontalControlPointKind::start, {0.0, 0.0}, 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::intersection, {1000.0, 0.0}, 500.0, 80.0, 60.0},
        {HorizontalControlPointKind::end, {1500.0, 800.0}, 0.0, 0.0, 0.0}
    };
    const detail::HorizontalAlignment alignment(expected, 0.0);
    std::vector<Point2D> samples;
    for (double station = 0.0; station < alignment.end_station_m(); station += 20.0) {
        const auto state = alignment.evaluate(station);
        const double noise = 0.003 * std::sin(0.013 * station);
        samples.push_back({
            state.point.x - noise * std::sin(state.heading_rad),
            state.point.y + noise * std::cos(state.heading_rad)
        });
    }
    samples.push_back(alignment.evaluate(alignment.end_station_m()).point);

    const detail::HorizontalReconstructor reconstructor({});
    const auto actual = reconstructor.reconstruct(samples);
    AR_EXPECT(actual.size() == 3);
    AR_EXPECT_NEAR(actual[1].position.x, expected[1].position.x, 0.25);
    AR_EXPECT_NEAR(actual[1].position.y, expected[1].position.y, 0.25);
    AR_EXPECT_NEAR(actual[1].radius_m, expected[1].radius_m, 5.0);
    AR_EXPECT_NEAR(actual[1].entry_spiral_length_m, expected[1].entry_spiral_length_m, 10.0);
    AR_EXPECT_NEAR(actual[1].exit_spiral_length_m, expected[1].exit_spiral_length_m, 10.0);
}

AR_TEST(horizontal_reconstruction_keeps_a_straight_route_as_two_controls) {
    std::vector<Point2D> samples;
    for (int index = 0; index <= 100; ++index) {
        const double distance = 10.0 * static_cast<double>(index);
        samples.push_back({5000.0 + distance, 300.0 + 0.25 * distance});
    }
    const detail::HorizontalReconstructor reconstructor({});
    const auto actual = reconstructor.reconstruct(samples);
    AR_EXPECT(actual.size() == 2);
    AR_EXPECT(actual.front().kind == HorizontalControlPointKind::start);
    AR_EXPECT(actual.back().kind == HorizontalControlPointKind::end);
    AR_EXPECT_NEAR(actual.front().position.x, samples.front().x, 1.0e-9);
    AR_EXPECT_NEAR(actual.back().position.y, samples.back().y, 1.0e-9);
}

AR_TEST(horizontal_classification_rechecks_the_first_merged_pair_safely) {
    const std::vector<HorizontalControlPoint> controls{
        {HorizontalControlPointKind::start, {0.0, 0.0}, 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::intersection,
         {1200.0, 0.0}, 600.0, 80.0, 70.0},
        {HorizontalControlPointKind::end,
         {1800.0, 900.0}, 0.0, 0.0, 0.0}
    };
    const detail::HorizontalAlignment alignment(controls, 0.0);
    std::vector<Point2D> samples;
    for (double station = 0.0; station < alignment.end_station_m(); station += 10.0) {
        samples.push_back(alignment.evaluate(station).point);
    }
    samples.push_back(alignment.evaluate(alignment.end_station_m()).point);

    const auto analysis = detail::classify_horizontal_samples(samples, {});
    AR_EXPECT(analysis.straight_segments.size() == 1);
    AR_EXPECT(analysis.straight_segments.front().first_index == 0);
    AR_EXPECT(analysis.straight_segments.front().last_index + 1 == samples.size());
}

AR_TEST(field_horizontal_reconstruction_recovers_all_mdb_control_points) {
    const auto directory = test_support::fixture_directory();
    const auto points = test_support::read_point_fixture(
        directory / "k1413_k1462_points.csv");
    const auto expected_rows = test_support::read_horizontal_fixture(
        directory / "k1413_k1462_horizontal.csv");
    const detail::HorizontalReconstructor reconstructor({});
    const auto actual = reconstructor.reconstruct(points);
    AR_EXPECT(actual.size() == expected_rows.size());

    double maximum_coordinate_error_m = 0.0;
    double maximum_radius_error_m = 0.0;
    double maximum_entry_spiral_error_m = 0.0;
    double maximum_exit_spiral_error_m = 0.0;
    std::size_t coordinate_index = 0;
    std::size_t radius_index = 0;
    std::size_t entry_spiral_index = 0;
    std::size_t exit_spiral_index = 0;
    for (std::size_t index = 0; index < actual.size(); ++index) {
        const auto& expected = expected_rows[index].control_point;
        AR_EXPECT(actual[index].kind == expected.kind);
        const double coordinate_error = detail::distance(
            actual[index].position, expected.position);
        const double radius_error = std::abs(
            actual[index].radius_m - expected.radius_m);
        const double entry_spiral_error = std::abs(
            actual[index].entry_spiral_length_m - expected.entry_spiral_length_m);
        const double exit_spiral_error = std::abs(
            actual[index].exit_spiral_length_m - expected.exit_spiral_length_m);
        if (coordinate_error > maximum_coordinate_error_m) {
            maximum_coordinate_error_m = coordinate_error;
            coordinate_index = index;
        }
        if (radius_error > maximum_radius_error_m) {
            maximum_radius_error_m = radius_error;
            radius_index = index;
        }
        if (entry_spiral_error > maximum_entry_spiral_error_m) {
            maximum_entry_spiral_error_m = entry_spiral_error;
            entry_spiral_index = index;
        }
        if (exit_spiral_error > maximum_exit_spiral_error_m) {
            maximum_exit_spiral_error_m = exit_spiral_error;
            exit_spiral_index = index;
        }
    }

    std::cout << "[METRIC] field horizontal reconstruction: max_coordinate_m="
              << maximum_coordinate_error_m << "@" << coordinate_index
              << ", max_radius_m=" << maximum_radius_error_m << "@" << radius_index
              << ", max_entry_spiral_m=" << maximum_entry_spiral_error_m
              << "@" << entry_spiral_index
              << ", max_exit_spiral_m=" << maximum_exit_spiral_error_m
              << "@" << exit_spiral_index << '\n';
    constexpr double parity_tolerance_m = 0.001;
    if (maximum_coordinate_error_m > parity_tolerance_m ||
        maximum_radius_error_m > parity_tolerance_m ||
        maximum_entry_spiral_error_m > parity_tolerance_m ||
        maximum_exit_spiral_error_m > parity_tolerance_m) {
        std::ostringstream detail_message;
        detail_message << "coordinate abs_error=" << maximum_coordinate_error_m
                       << " at control " << coordinate_index
                       << "; radius abs_error=" << maximum_radius_error_m
                       << " at control " << radius_index
                       << "; entry spiral abs_error=" << maximum_entry_spiral_error_m
                       << " at control " << entry_spiral_index
                       << "; exit spiral abs_error=" << maximum_exit_spiral_error_m
                       << " at control " << exit_spiral_index
                       << "; tolerance=" << parity_tolerance_m;
        test_support::fail(
            "field horizontal reconstruction parity",
            __FILE__,
            __LINE__,
            detail_message.str());
    }
}
