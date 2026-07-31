#include "test_framework.h"

#include "alignment_reconstruct/point_cloud_fit.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace alignment_reconstruct;

namespace {

std::vector<Point3D> straight_point_cloud() {
    std::vector<Point3D> points;
    points.reserve(303);
    for (std::size_t index = 0; index <= 100; ++index) {
        const double x = static_cast<double>(index);
        const double center_y = 0.1 * x;
        const double center_z = 100.0 + 0.01 * x;
        points.push_back({x, center_y - 0.4, center_z - 0.2});
        points.push_back({x, center_y, center_z});
        points.push_back({x, center_y + 0.4, center_z + 0.2});
    }
    return points;
}

std::string environment_value(const char* name) {
    char* value = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&value, &length, name) != 0 || value == nullptr) {
        return {};
    }
    const std::string result(value);
    std::free(value);
    return result;
}

std::vector<Point3D> read_comma_triples(
    const std::string& path,
    double coordinate_scale) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("Could not open external point-cloud fixture");
    }

    std::vector<Point3D> points;
    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream fields(line);
        Point3D point;
        char first_comma = '\0';
        char second_comma = '\0';
        if (!(fields >> point.x >> first_comma >>
              point.y >> second_comma >> point.z) ||
            first_comma != ',' ||
            second_comma != ',') {
            throw std::runtime_error(
                "External point-cloud fixture contains an invalid XYZ row");
        }
        fields >> std::ws;
        if (!fields.eof()) {
            throw std::runtime_error(
                "External point-cloud fixture contains extra row fields");
        }
        point.x *= coordinate_scale;
        point.y *= coordinate_scale;
        point.z *= coordinate_scale;
        points.push_back(point);
    }
    return points;
}

void expect_same_points(
    const std::vector<Point3D>& actual,
    const std::vector<Point3D>& expected,
    double tolerance_m) {
    AR_EXPECT(actual.size() == expected.size());
    for (std::size_t index = 0; index < actual.size(); ++index) {
        AR_EXPECT_NEAR(actual[index].x, expected[index].x, tolerance_m);
        AR_EXPECT_NEAR(actual[index].y, expected[index].y, tolerance_m);
        AR_EXPECT_NEAR(actual[index].z, expected[index].z, tolerance_m);
    }
}

} // namespace

AR_TEST(point_cloud_fit_samples_a_straight_centerline_by_plan_length) {
    std::vector<Point3D> points = straight_point_cloud();
    std::reverse(points.begin(), points.end());

    const auto result = fit_point_cloud(points, 10.0);

    AR_EXPECT(result.ok());
    AR_EXPECT(result.diagnostics.empty());
    AR_EXPECT(result.centerline_points.size() == 12);
    AR_EXPECT_NEAR(result.centerline_points.front().x, 0.0, 1.0e-10);
    AR_EXPECT_NEAR(result.centerline_points.front().y, 0.0, 1.0e-10);
    AR_EXPECT_NEAR(result.centerline_points.front().z, 100.0, 1.0e-10);
    AR_EXPECT_NEAR(result.centerline_points.back().x, 100.0, 1.0e-10);
    AR_EXPECT_NEAR(result.centerline_points.back().y, 10.0, 1.0e-10);
    AR_EXPECT_NEAR(result.centerline_points.back().z, 101.0, 1.0e-10);

    const double plan_scale = std::sqrt(1.01);
    for (std::size_t index = 0; index <= 10; ++index) {
        const double station = static_cast<double>(index) * 10.0;
        const double expected_x = station / plan_scale;
        AR_EXPECT_NEAR(
            result.centerline_points[index].x, expected_x, 1.0e-9);
        AR_EXPECT_NEAR(
            result.centerline_points[index].y, 0.1 * expected_x, 1.0e-9);
        AR_EXPECT_NEAR(
            result.centerline_points[index].z,
            100.0 + 0.01 * expected_x,
            1.0e-9);
        if (index > 0) {
            const double dx =
                result.centerline_points[index].x -
                result.centerline_points[index - 1].x;
            const double dy =
                result.centerline_points[index].y -
                result.centerline_points[index - 1].y;
            AR_EXPECT_NEAR(std::hypot(dx, dy), 10.0, 1.0e-9);
        }
    }
}

AR_TEST(point_cloud_fit_is_independent_of_input_row_order) {
    const std::vector<Point3D> forward = straight_point_cloud();
    std::vector<Point3D> reverse = forward;
    std::reverse(reverse.begin(), reverse.end());

    const auto forward_result = fit_point_cloud(forward, 7.5);
    const auto reverse_result = fit_point_cloud(reverse, 7.5);

    AR_EXPECT(forward_result.ok());
    AR_EXPECT(reverse_result.ok());
    expect_same_points(
        forward_result.centerline_points,
        reverse_result.centerline_points,
        0.0);
}

AR_TEST(point_cloud_fit_keeps_both_endpoints_when_interval_exceeds_length) {
    const auto result = fit_point_cloud(straight_point_cloud(), 1000.0);

    AR_EXPECT(result.ok());
    AR_EXPECT(result.centerline_points.size() == 2);
    AR_EXPECT_NEAR(result.centerline_points.front().x, 0.0, 1.0e-10);
    AR_EXPECT_NEAR(result.centerline_points.back().x, 100.0, 1.0e-10);
}

AR_TEST(point_cloud_fit_rejects_too_few_points) {
    const auto result = fit_point_cloud(
        std::vector<Point3D>(9, Point3D{}),
        10.0);

    AR_EXPECT(result.status == StatusCode::invalid_input);
    AR_EXPECT(result.centerline_points.empty());
    AR_EXPECT(result.diagnostics.size() == 1);
    AR_EXPECT(
        result.diagnostics[0].code == DiagnosticCode::too_few_points);
}

AR_TEST(point_cloud_fit_rejects_invalid_sample_intervals) {
    const auto points = straight_point_cloud();
    const auto zero = fit_point_cloud(points, 0.0);
    const auto infinite = fit_point_cloud(
        points, std::numeric_limits<double>::infinity());

    AR_EXPECT(zero.status == StatusCode::invalid_input);
    AR_EXPECT(infinite.status == StatusCode::invalid_input);
    AR_EXPECT(
        zero.diagnostics[0].code ==
        DiagnosticCode::invalid_sample_interval);
    AR_EXPECT(
        infinite.diagnostics[0].code ==
        DiagnosticCode::invalid_sample_interval);
}

AR_TEST(point_cloud_fit_reports_the_non_finite_input_index) {
    std::vector<Point3D> points = straight_point_cloud();
    points[17].z = std::numeric_limits<double>::quiet_NaN();

    const auto result = fit_point_cloud(points, 10.0);

    AR_EXPECT(result.status == StatusCode::invalid_input);
    AR_EXPECT(result.diagnostics.size() == 1);
    AR_EXPECT(
        result.diagnostics[0].code == DiagnosticCode::non_finite_value);
    AR_EXPECT(result.diagnostics[0].input_index == 17);
}

AR_TEST(point_cloud_fit_rejects_bins_with_fewer_than_three_points) {
    std::vector<Point3D> points;
    for (std::size_t index = 0; index < 25; ++index) {
        const double x = static_cast<double>(index);
        points.push_back({x, 0.0, 0.0});
        points.push_back({x, 1.0, 1.0});
    }

    const auto result = fit_point_cloud(points, 10.0);

    AR_EXPECT(result.status == StatusCode::invalid_input);
    AR_EXPECT(result.diagnostics.size() == 1);
    AR_EXPECT(
        result.diagnostics[0].code ==
        DiagnosticCode::insufficient_spatial_support);
}

AR_TEST(point_cloud_fit_rejects_fewer_than_twenty_spatial_bins) {
    std::vector<Point3D> points;
    for (std::size_t index = 0; index < 19; ++index) {
        const double x = static_cast<double>(index);
        points.push_back({x, -1.0, -1.0});
        points.push_back({x, 0.0, 0.0});
        points.push_back({x, 1.0, 1.0});
    }

    const auto result = fit_point_cloud(points, 10.0);

    AR_EXPECT(result.status == StatusCode::invalid_input);
    AR_EXPECT(result.diagnostics.size() == 1);
    AR_EXPECT(
        result.diagnostics[0].code ==
        DiagnosticCode::insufficient_spatial_support);
}

AR_TEST(point_cloud_fit_matches_the_optional_python_reference) {
    const std::string point_cloud_path = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_POINT_CLOUD_MM_CSV");
    const std::string centerline_path = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_POINT_CLOUD_CENTERLINE_MM_CSV");
    if (point_cloud_path.empty() || centerline_path.empty()) {
        test_support::skip(
            "external point-cloud and Python reference environment variables are not set");
    }

    const std::vector<Point3D> points =
        read_comma_triples(point_cloud_path, 0.001);
    const std::vector<Point3D> expected =
        read_comma_triples(centerline_path, 0.001);
    const auto result = fit_point_cloud(points, 10.0);
    std::vector<Point3D> reversed_points = points;
    std::reverse(reversed_points.begin(), reversed_points.end());
    const auto reversed_result = fit_point_cloud(reversed_points, 10.0);

    AR_EXPECT(result.ok());
    AR_EXPECT(reversed_result.ok());
    AR_EXPECT(expected.size() == 262);
    expect_same_points(result.centerline_points, expected, 1.0e-7);
    expect_same_points(
        result.centerline_points,
        reversed_result.centerline_points,
        0.0);
}
