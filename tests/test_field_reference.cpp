#include "fixture_reader.h"
#include "test_framework.h"

#include "alignment_reconstruct/adjustment.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

using namespace alignment_reconstruct;

AR_TEST(public_adjustment_api_matches_all_mdb_horizontal_results) {
    const auto directory = test_support::fixture_directory();
    const auto horizontal_rows = test_support::read_horizontal_fixture(
        directory / "k1413_k1462_horizontal.csv");
    const auto expected_rows = test_support::read_offset_fixture(
        directory / "k1413_k1462_offsets.csv");

    AlignmentDefinition alignment;
    for (const auto& row : horizontal_rows) {
        alignment.horizontal.push_back(row.control_point);
    }
    alignment.vertical = {{0.0, 0.0, 0.0}, {60000.0, 0.0, 0.0}};
    std::vector<Point3D> points;
    points.reserve(expected_rows.size());
    for (const auto& row : expected_rows) {
        points.push_back({row.point.x, row.point.y, 0.0});
    }

    const auto result = calculate_adjustments(points, alignment);
    AR_EXPECT(result.ok());
    AR_EXPECT(result.points.size() == expected_rows.size());
    double maximum_station_error_m = 0.0;
    double maximum_offset_error_m = 0.0;
    double squared_offset_error_sum = 0.0;
    std::size_t station_index = 0;
    std::size_t offset_index = 0;
    double station_actual = 0.0;
    double station_expected = 0.0;
    double offset_actual = 0.0;
    double offset_expected = 0.0;
    for (std::size_t index = 0; index < result.points.size(); ++index) {
        const double station_error = std::abs(
            result.points[index].station_m - expected_rows[index].station_m);
        const double offset_error = std::abs(
            result.points[index].horizontal_offset_m -
            expected_rows[index].horizontal_offset_m);
        if (station_error > maximum_station_error_m) {
            maximum_station_error_m = station_error;
            station_index = index;
            station_actual = result.points[index].station_m;
            station_expected = expected_rows[index].station_m;
        }
        if (offset_error > maximum_offset_error_m) {
            maximum_offset_error_m = offset_error;
            offset_index = index;
            offset_actual = result.points[index].horizontal_offset_m;
            offset_expected = expected_rows[index].horizontal_offset_m;
        }
        squared_offset_error_sum += offset_error * offset_error;
        AR_EXPECT_NEAR(result.points[index].lift_drop_m, 0.0, 0.0);
    }
    const double rms_offset_error_m = std::sqrt(
        squared_offset_error_sum / static_cast<double>(result.points.size()));
    std::cout << "[METRIC] public field adjustment: max_station_m="
              << maximum_station_error_m << "@" << station_index
              << ", max_offset_m=" << maximum_offset_error_m << "@" << offset_index
              << ", rms_offset_m=" << rms_offset_error_m << '\n';
    if (maximum_station_error_m > 0.02 || maximum_offset_error_m > 0.002 ||
        rms_offset_error_m > 0.0005) {
        std::ostringstream detail;
        detail << "fixture=k1413_k1462_offsets.csv, station actual=" << station_actual
               << ", expected=" << station_expected
               << ", abs_error=" << maximum_station_error_m << ", tolerance=0.02 at "
               << station_index << "; offset actual=" << offset_actual
               << ", expected=" << offset_expected
               << ", abs_error=" << maximum_offset_error_m << ", tolerance=0.002 at "
               << offset_index
               << ", rms offset=" << rms_offset_error_m;
        test_support::fail("public field adjustment parity", __FILE__, __LINE__, detail.str());
    }
}
