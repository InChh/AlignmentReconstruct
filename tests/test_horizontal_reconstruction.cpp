#include "fixture_reader.h"
#include "test_framework.h"

#include "geometry/vector_math.h"
#include "horizontal/horizontal_alignment.h"
#include "horizontal/horizontal_reconstructor.h"

#include <algorithm>
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

    ReconstructionOptions options;
    options.horizontal_optimization_max_iterations = 160;
    const detail::HorizontalReconstructor reconstructor(options);
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

AR_TEST(field_horizontal_reconstruction_recovers_all_mdb_control_points) {
    const auto directory = test_support::fixture_directory();
    const auto points = test_support::read_point_fixture(
        directory / "k1413_k1462_points.csv");
    const auto expected_rows = test_support::read_horizontal_fixture(
        directory / "k1413_k1462_horizontal.csv");
    ReconstructionOptions options;
    options.horizontal_optimization_max_iterations = 200;
    const detail::HorizontalReconstructor reconstructor(options);
    const auto actual = reconstructor.reconstruct(points);
    AR_EXPECT(actual.size() == expected_rows.size());

    double maximum_coordinate_error_m = 0.0;
    double maximum_radius_error_m = 0.0;
    double maximum_spiral_error_m = 0.0;
    std::size_t coordinate_index = 0;
    std::size_t radius_index = 0;
    std::size_t spiral_index = 0;
    Point2D coordinate_actual{};
    Point2D coordinate_expected{};
    double radius_actual = 0.0;
    double radius_expected = 0.0;
    double spiral_actual = 0.0;
    double spiral_expected = 0.0;
    for (std::size_t index = 0; index < actual.size(); ++index) {
        const auto& expected = expected_rows[index].control_point;
        const double coordinate_error = detail::distance(actual[index].position, expected.position);
        const double radius_error = std::abs(actual[index].radius_m - expected.radius_m);
        const double spiral_error = std::max(
            std::abs(actual[index].entry_spiral_length_m - expected.entry_spiral_length_m),
            std::abs(actual[index].exit_spiral_length_m - expected.exit_spiral_length_m));
        if (coordinate_error > maximum_coordinate_error_m) {
            maximum_coordinate_error_m = coordinate_error;
            coordinate_index = index;
            coordinate_actual = actual[index].position;
            coordinate_expected = expected.position;
        }
        if (radius_error > maximum_radius_error_m) {
            maximum_radius_error_m = radius_error;
            radius_index = index;
            radius_actual = actual[index].radius_m;
            radius_expected = expected.radius_m;
        }
        if (spiral_error > maximum_spiral_error_m) {
            maximum_spiral_error_m = spiral_error;
            spiral_index = index;
            const double entry_error = std::abs(
                actual[index].entry_spiral_length_m - expected.entry_spiral_length_m);
            if (entry_error >= std::abs(
                    actual[index].exit_spiral_length_m - expected.exit_spiral_length_m)) {
                spiral_actual = actual[index].entry_spiral_length_m;
                spiral_expected = expected.entry_spiral_length_m;
            } else {
                spiral_actual = actual[index].exit_spiral_length_m;
                spiral_expected = expected.exit_spiral_length_m;
            }
        }
    }
    std::cout << "[METRIC] field horizontal reconstruction: max_coordinate_m="
              << maximum_coordinate_error_m << "@" << coordinate_index
              << ", max_radius_m=" << maximum_radius_error_m << "@" << radius_index
              << ", max_spiral_m=" << maximum_spiral_error_m << "@" << spiral_index << '\n';
    if (maximum_coordinate_error_m > 0.5 || maximum_radius_error_m > 150.0 ||
        maximum_spiral_error_m > 40.0) {
        std::ostringstream detail_message;
        detail_message << "fixture=k1413_k1462_horizontal.csv; coordinate actual=("
                       << coordinate_actual.x << ',' << coordinate_actual.y << "), expected=("
                       << coordinate_expected.x << ',' << coordinate_expected.y
                       << "), abs_error=" << maximum_coordinate_error_m
                       << ", tolerance=0.5 at " << coordinate_index
                       << "; radius actual=" << radius_actual << ", expected=" << radius_expected
                       << ", abs_error=" << maximum_radius_error_m
                       << ", tolerance=150 at " << radius_index
                       << "; spiral actual=" << spiral_actual << ", expected=" << spiral_expected
                       << ", abs_error=" << maximum_spiral_error_m
                       << ", tolerance=40 at " << spiral_index;
        test_support::fail(
            "field horizontal reconstruction parity", __FILE__, __LINE__, detail_message.str());
    }
}
