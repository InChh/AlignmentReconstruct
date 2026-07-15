#include "fixture_reader.h"
#include "test_framework.h"

#include "horizontal/horizontal_alignment.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>

using alignment_reconstruct::HorizontalControlPoint;
using alignment_reconstruct::HorizontalControlPointKind;
using alignment_reconstruct::Point2D;
using alignment_reconstruct::detail::HorizontalAlignment;

AR_TEST(horizontal_alignment_builds_two_point_line_and_right_positive_offsets) {
    const std::vector<HorizontalControlPoint> controls{
        {HorizontalControlPointKind::start, {0.0, 0.0}, 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::end, {100.0, 0.0}, 0.0, 0.0, 0.0}
    };
    const HorizontalAlignment alignment(controls, 1200.0);
    AR_EXPECT_NEAR(alignment.start_station_m(), 1200.0, 0.0);
    AR_EXPECT_NEAR(alignment.end_station_m(), 1300.0, 1.0e-12);

    const auto right = alignment.project({35.0, 4.5});
    const auto left = alignment.project({35.0, -2.0});
    AR_EXPECT(right.valid);
    AR_EXPECT(left.valid);
    AR_EXPECT_NEAR(right.station_m, 1235.0, 1.0e-10);
    AR_EXPECT_NEAR(right.signed_offset_m, 4.5, 1.0e-10);
    AR_EXPECT_NEAR(left.signed_offset_m, -2.0, 1.0e-10);

    const auto before_start = alignment.project({-10.0, 2.0});
    const auto after_end = alignment.project({110.0, -3.0});
    AR_EXPECT_NEAR(before_start.station_m, 1200.0, 0.0);
    AR_EXPECT_NEAR(after_end.station_m, 1300.0, 0.0);
    AR_EXPECT_NEAR(before_start.signed_offset_m, 2.0, 1.0e-12);
    AR_EXPECT_NEAR(after_end.signed_offset_m, -3.0, 1.0e-12);
}

AR_TEST(horizontal_alignment_supports_unequal_spirals_and_closes_on_tangents) {
    const std::vector<HorizontalControlPoint> controls{
        {HorizontalControlPointKind::start, {0.0, 0.0}, 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::intersection, {1000.0, 0.0}, 500.0, 60.0, 90.0},
        {HorizontalControlPointKind::end, {1400.0, 600.0}, 0.0, 0.0, 0.0}
    };
    const HorizontalAlignment alignment(controls, 0.0);
    const auto start = alignment.evaluate(0.0);
    const auto end = alignment.evaluate(alignment.end_station_m());
    AR_EXPECT_NEAR(start.point.x, 0.0, 1.0e-12);
    AR_EXPECT_NEAR(start.point.y, 0.0, 1.0e-12);
    AR_EXPECT_NEAR(end.point.x, 1400.0, 1.0e-7);
    AR_EXPECT_NEAR(end.point.y, 600.0, 1.0e-7);
    AR_EXPECT(alignment.elements().size() == 5);
}

AR_TEST(field_horizontal_alignment_matches_mdb_stations_and_offsets) {
    const auto directory = test_support::fixture_directory();
    const auto horizontal_rows = test_support::read_horizontal_fixture(
        directory / "k1413_k1462_horizontal.csv");
    const auto expected_rows = test_support::read_offset_fixture(
        directory / "k1413_k1462_offsets.csv");

    std::vector<HorizontalControlPoint> controls;
    controls.reserve(horizontal_rows.size());
    for (const auto& row : horizontal_rows) {
        controls.push_back(row.control_point);
    }
    const HorizontalAlignment alignment(controls, 0.0);

    double maximum_station_error_m = 0.0;
    double maximum_offset_error_m = 0.0;
    double squared_offset_error_sum = 0.0;
    std::size_t maximum_station_index = 0;
    std::size_t maximum_offset_index = 0;
    for (const auto& expected : expected_rows) {
        const auto actual = alignment.project(expected.point);
        AR_EXPECT(actual.valid);
        const double station_error = std::abs(actual.station_m - expected.station_m);
        const double offset_error = std::abs(actual.signed_offset_m - expected.horizontal_offset_m);
        if (station_error > maximum_station_error_m) {
            maximum_station_error_m = station_error;
            maximum_station_index = expected.point_index;
        }
        if (offset_error > maximum_offset_error_m) {
            maximum_offset_error_m = offset_error;
            maximum_offset_index = expected.point_index;
        }
        squared_offset_error_sum += offset_error * offset_error;
    }

    const double rms_offset_error_m = std::sqrt(
        squared_offset_error_sum / static_cast<double>(expected_rows.size()));
    std::cout << "[METRIC] field alignment parity: max_station_m="
              << maximum_station_error_m << "@" << maximum_station_index
              << ", max_offset_m=" << maximum_offset_error_m << "@" << maximum_offset_index
              << ", rms_offset_m=" << rms_offset_error_m << '\n';
    if (maximum_station_error_m > 0.02 || maximum_offset_error_m > 0.002 ||
        rms_offset_error_m > 0.0005) {
        std::ostringstream detail;
        detail << "max station error=" << maximum_station_error_m
               << " at " << maximum_station_index
               << ", max offset error=" << maximum_offset_error_m
               << " at " << maximum_offset_index
               << ", rms offset error=" << rms_offset_error_m;
        test_support::fail("field horizontal parity", __FILE__, __LINE__, detail.str());
    }
}
