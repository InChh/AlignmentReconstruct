#include "test_framework.h"

#include "alignment_reconstruct/point_cloud_fit.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
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

std::vector<Point3D> axis_aligned_point_cloud() {
    std::vector<Point3D> points;
    points.reserve(303);
    for (std::size_t index = 0; index <= 100; ++index) {
        const double x = static_cast<double>(index);
        const double center_z = 100.0 + 0.01 * x;
        points.push_back({x, -0.4, center_z - 0.2});
        points.push_back({x, 0.0, center_z});
        points.push_back({x, 0.4, center_z + 0.2});
    }
    return points;
}

Point3D rotate_plan(const Point3D& point, double angle_rad) {
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    return {
        cosine * point.x - sine * point.y,
        sine * point.x + cosine * point.y,
        point.z
    };
}

std::vector<Point3D> rotate_plan(
    const std::vector<Point3D>& points,
    double angle_rad) {
    std::vector<Point3D> rotated;
    rotated.reserve(points.size());
    for (const Point3D& point : points) {
        rotated.push_back(rotate_plan(point, angle_rad));
    }
    return rotated;
}

std::vector<Point3D> swap_plan_coordinates(
    const std::vector<Point3D>& points) {
    std::vector<Point3D> swapped = points;
    for (Point3D& point : swapped) {
        std::swap(point.x, point.y);
    }
    return swapped;
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

struct ReferenceError {
    double maximum_plan_m = 0.0;
    double rms_plan_m = 0.0;
    double maximum_height_m = 0.0;
    double rms_height_m = 0.0;
};

ReferenceError reference_error(
    const std::vector<Point3D>& fitted,
    const std::vector<Point3D>& reference) {
    AR_EXPECT(!fitted.empty());
    AR_EXPECT(reference.size() >= 2);
    ReferenceError result;
    for (const auto& point : fitted) {
        double minimum_distance_squared = std::numeric_limits<double>::infinity();
        double expected_z = 0.0;
        for (std::size_t index = 1; index < reference.size(); ++index) {
            const auto& left = reference[index-1];
            const auto& right = reference[index];
            const double dx = right.x-left.x;
            const double dy = right.y-left.y;
            const double fraction = std::clamp(
                ((point.x-left.x)*dx+(point.y-left.y)*dy)/(dx*dx+dy*dy),0.0,1.0);
            const double ex = point.x-left.x-fraction*dx;
            const double ey = point.y-left.y-fraction*dy;
            const double distance_squared = ex*ex+ey*ey;
            if (distance_squared < minimum_distance_squared) {
                minimum_distance_squared = distance_squared;
                expected_z = left.z+fraction*(right.z-left.z);
            }
        }
        const double height_error = point.z-expected_z;
        result.maximum_plan_m = std::max(result.maximum_plan_m,std::sqrt(minimum_distance_squared));
        result.maximum_height_m = std::max(result.maximum_height_m,std::abs(height_error));
        result.rms_plan_m += minimum_distance_squared;
        result.rms_height_m += height_error*height_error;
    }
    result.rms_plan_m = std::sqrt(result.rms_plan_m/static_cast<double>(fitted.size()));
    result.rms_height_m = std::sqrt(result.rms_height_m/static_cast<double>(fitted.size()));
    return result;
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

AR_TEST(point_cloud_fit_automatically_orients_a_y_direction_point_cloud) {
    const double right_angle_rad = std::acos(-1.0) * 0.5;
    const std::vector<Point3D> points =
        rotate_plan(axis_aligned_point_cloud(), right_angle_rad);
    std::vector<Point3D> reversed_points = points;
    std::reverse(reversed_points.begin(), reversed_points.end());

    const auto result = fit_point_cloud(points, 10.0);
    const auto reversed_result = fit_point_cloud(reversed_points, 10.0);

    AR_EXPECT(result.ok());
    AR_EXPECT(reversed_result.ok());
    AR_EXPECT(result.centerline_points.size() == 11);
    for (std::size_t index = 0; index <= 10; ++index) {
        const double station_m = static_cast<double>(index) * 10.0;
        const Point3D expected = rotate_plan(
            {station_m, 0.0, 100.0 + 0.01 * station_m},
            right_angle_rad);
        AR_EXPECT_NEAR(
            result.centerline_points[index].x, expected.x, 1.0e-9);
        AR_EXPECT_NEAR(
            result.centerline_points[index].y, expected.y, 1.0e-9);
        AR_EXPECT_NEAR(
            result.centerline_points[index].z, expected.z, 1.0e-9);
    }
    expect_same_points(
        result.centerline_points,
        reversed_result.centerline_points,
        0.0);
}

AR_TEST(point_cloud_fit_automatically_orients_a_diagonal_point_cloud) {
    const double diagonal_angle_rad = std::acos(-1.0) * 0.25;
    const std::vector<Point3D> points =
        rotate_plan(axis_aligned_point_cloud(), diagonal_angle_rad);
    std::vector<Point3D> permuted = points;
    std::rotate(permuted.begin(), permuted.begin() + 137, permuted.end());
    std::reverse(permuted.begin(), permuted.begin() + 83);

    const auto result = fit_point_cloud(points, 10.0);
    const auto permuted_result = fit_point_cloud(permuted, 10.0);

    AR_EXPECT(result.ok());
    AR_EXPECT(permuted_result.ok());
    AR_EXPECT(result.centerline_points.size() == 11);
    for (std::size_t index = 0; index <= 10; ++index) {
        const double station_m = static_cast<double>(index) * 10.0;
        const Point3D expected = rotate_plan(
            {station_m, 0.0, 100.0 + 0.01 * station_m},
            diagonal_angle_rad);
        AR_EXPECT_NEAR(
            result.centerline_points[index].x, expected.x, 1.0e-9);
        AR_EXPECT_NEAR(
            result.centerline_points[index].y, expected.y, 1.0e-9);
        AR_EXPECT_NEAR(
            result.centerline_points[index].z, expected.z, 1.0e-9);
        if (index > 0) {
            const Point3D& previous = result.centerline_points[index - 1];
            AR_EXPECT_NEAR(
                std::hypot(
                    result.centerline_points[index].x - previous.x,
                    result.centerline_points[index].y - previous.y),
                10.0,
                1.0e-9);
        }
    }
    expect_same_points(
        result.centerline_points,
        permuted_result.centerline_points,
        0.0);
}

AR_TEST(point_cloud_fit_rejects_a_cloud_without_a_stable_plan_direction) {
    std::vector<Point3D> points;
    for (std::size_t x = 0; x < 25; ++x) {
        for (std::size_t y = 0; y < 25; ++y) {
            points.push_back({
                static_cast<double>(x),
                static_cast<double>(y),
                100.0
            });
        }
    }

    const auto result = fit_point_cloud(points, 10.0);

    AR_EXPECT(result.status == StatusCode::invalid_input);
    AR_EXPECT(result.centerline_points.empty());
    AR_EXPECT(result.diagnostics.size() == 1);
    AR_EXPECT(
        result.diagnostics[0].code ==
        DiagnosticCode::insufficient_spatial_support);
    AR_EXPECT(result.diagnostics[0].input_index == no_input_index);
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

AR_TEST(point_cloud_fit_accepts_two_points_per_longitudinal_bin) {
    std::vector<Point3D> points;
    for (std::size_t index = 0; index < 25; ++index) {
        const double x = static_cast<double>(index);
        points.push_back({x, 0.0, 0.0});
        points.push_back({x, 1.0, 1.0});
    }

    const auto result = fit_point_cloud(points, 10.0);

    AR_EXPECT(result.ok());
    AR_EXPECT(result.centerline_points.size() == 4);
    for (const Point3D& point : result.centerline_points) {
        AR_EXPECT_NEAR(point.y, 0.5, 1.0e-9);
        AR_EXPECT_NEAR(point.z, 0.5, 1.0e-9);
    }
    AR_EXPECT_NEAR(result.centerline_points.front().x, 0.0, 1.0e-9);
    AR_EXPECT_NEAR(result.centerline_points.back().x, 24.0, 1.0e-9);
}

AR_TEST(point_cloud_fit_accepts_a_short_supported_cloud) {
    std::vector<Point3D> points;
    for (std::size_t index = 0; index < 19; ++index) {
        const double x = static_cast<double>(index);
        points.push_back({x, -1.0, -1.0});
        points.push_back({x, 0.0, 0.0});
        points.push_back({x, 1.0, 1.0});
    }

    const auto result = fit_point_cloud(points, 10.0);

    AR_EXPECT(result.ok());
    AR_EXPECT(result.centerline_points.size() == 3);
    AR_EXPECT_NEAR(result.centerline_points.front().x, 0.0, 1.0e-9);
    AR_EXPECT_NEAR(result.centerline_points.back().x, 18.0, 1.0e-9);
}

AR_TEST(point_cloud_fit_handles_a_sparse_fitted_centerline) {
    std::vector<Point3D> points;
    for (std::size_t index = 0; index <= 80; ++index) {
        const double x = 5.0 * static_cast<double>(index);
        points.push_back({x, 0.0002 * x * x, 100.0 + 0.01 * x});
    }
    const auto result = fit_point_cloud(points, 5.0);
    AR_EXPECT(result.ok());
    AR_EXPECT(result.centerline_points.size() > 80);
    for (const Point3D& point : result.centerline_points) {
        AR_EXPECT_NEAR(point.y, 0.0002 * point.x * point.x, 0.002);
        AR_EXPECT_NEAR(point.z, 100.0 + 0.01 * point.x, 1.0e-8);
    }
    AR_EXPECT_NEAR(result.centerline_points.front().x, 0.0, 1.0e-9);
    AR_EXPECT_NEAR(result.centerline_points.back().x, 400.0, 1.0e-9);
    std::vector<Point3D> repeated;
    for (const auto& point : points) {
        repeated.insert(repeated.end(), 3, point);
    }
    const auto repeated_result = fit_point_cloud(repeated, 5.0);
    AR_EXPECT(repeated_result.ok());
    expect_same_points(result.centerline_points, repeated_result.centerline_points, 1.0e-8);
    std::rotate(points.begin(), points.begin() + 31, points.end());
    std::reverse(points.begin(), points.end());
    const auto permuted = fit_point_cloud(points, 5.0);
    AR_EXPECT(permuted.ok());
    expect_same_points(result.centerline_points, permuted.centerline_points, 0.0);
}

AR_TEST(point_cloud_fit_handles_sparse_wide_cross_sections_without_row_order) {
    std::vector<Point3D> points;
    for (std::size_t index = 0; index <= 80; ++index) {
        const double x = 5.0 * static_cast<double>(index);
        for (std::size_t cross = 0; cross < 10; ++cross) {
            const double offset = (static_cast<double>(cross) - 4.5) * 4.0;
            points.push_back(rotate_plan({x, offset, 100.0 + 0.01*x}, 0.63));
        }
    }
    std::reverse(points.begin(), points.end());
    const auto result = fit_point_cloud(points, 5.0);
    AR_EXPECT(result.ok());
    AR_EXPECT(result.centerline_points.size() == 81);
    for (std::size_t index = 0; index < result.centerline_points.size(); ++index) {
        const Point3D expected = rotate_plan(
            {5.0*static_cast<double>(index), 0.0, 100.0 + 0.05*static_cast<double>(index)}, 0.63);
        AR_EXPECT_NEAR(result.centerline_points[index].x, expected.x, 1.0e-8);
        AR_EXPECT_NEAR(result.centerline_points[index].y, expected.y, 1.0e-8);
        AR_EXPECT_NEAR(result.centerline_points[index].z, expected.z, 1.0e-8);
    }
}

AR_TEST(point_cloud_fit_handles_large_sparse_longitudinal_spacing) {
    std::vector<Point3D> points;
    for (std::size_t index = 0; index <= 24; ++index) {
        const double x = 10000.0*static_cast<double>(index);
        points.push_back({x, 0.0, 100.0 + 0.001*x});
    }
    const auto result = fit_point_cloud(points, 10000.0);
    AR_EXPECT(result.ok());
    AR_EXPECT(result.centerline_points.size() == points.size());
    expect_same_points(result.centerline_points, points, 1.0e-7);
}

AR_TEST(point_cloud_fit_handles_a_curved_wide_sparse_cloud) {
    std::vector<Point3D> points;
    for (std::size_t index = 0; index <= 150; ++index) {
        const double x = 10.0*static_cast<double>(index);
        const double y = 0.0002*x*(x-1500.0);
        const double slope = 0.0002*(2.0*x-1500.0);
        const double normal_scale = std::hypot(1.0, slope);
        for (std::size_t cross = 0; cross < 10; ++cross) {
            const double offset = (static_cast<double>(cross)-4.5)*4.0;
            points.push_back({x-offset*slope/normal_scale,
                y+offset/normal_scale, 100.0+0.01*x});
        }
    }
    const auto result = fit_point_cloud(points, 5.0);
    AR_EXPECT(result.ok());
    AR_EXPECT_NEAR(result.centerline_points.front().x, 0.0, 1.0);
    AR_EXPECT_NEAR(result.centerline_points.back().x, 1500.0, 1.0);
    for (const Point3D& point : result.centerline_points) {
        AR_EXPECT_NEAR(point.y, 0.0002*point.x*(point.x-1500.0), 0.5);
        AR_EXPECT_NEAR(point.z, 100.0+0.01*point.x, 0.15);
    }
    std::rotate(points.begin(),points.begin()+317,points.end());
    std::reverse(points.begin(),points.end());
    const auto permuted=fit_point_cloud(points,5.0);
    AR_EXPECT(permuted.ok());
    expect_same_points(result.centerline_points,permuted.centerline_points,0.0);
    for (const double angle : {0.63, std::acos(-1.0)*0.5, std::acos(-1.0)*0.25}) {
        const auto rotated = fit_point_cloud(rotate_plan(points,angle),5.0);
        AR_EXPECT(rotated.ok());
        for (const auto& point : rotated.centerline_points) {
            const Point3D local = rotate_plan(point,-angle);
            AR_EXPECT_NEAR(local.y,0.0002*local.x*(local.x-1500.0),0.5);
            AR_EXPECT_NEAR(local.z,100.0+0.01*local.x,0.15);
        }
    }
}

AR_TEST(point_cloud_fit_handles_sub_metre_extent_and_keeps_both_endpoints) {
    std::vector<Point3D> points;
    for (std::size_t index = 0; index < 10; ++index) {
        const double x = 0.0005*static_cast<double>(index);
        points.push_back({x, 0.0, 100.0+0.01*x});
    }
    const auto result=fit_point_cloud(points,1.0);
    AR_EXPECT(result.ok());
    AR_EXPECT(result.centerline_points.size()==2);
    AR_EXPECT_NEAR(result.centerline_points.front().x,points.front().x,1.0e-12);
    AR_EXPECT_NEAR(result.centerline_points.back().x,points.back().x,1.0e-12);
    AR_EXPECT_NEAR(result.centerline_points.back().z,points.back().z,1.0e-10);
}

AR_TEST(point_cloud_fit_reports_insufficient_distinct_longitudinal_positions) {
    std::vector<Point3D> points(10,Point3D{0.0,0.0,100.0});
    points.insert(points.end(),10,Point3D{100.0,0.0,100.0});
    const auto result=fit_point_cloud(points,5.0);
    AR_EXPECT(result.status==StatusCode::invalid_input);
    AR_EXPECT(result.centerline_points.empty());
    AR_EXPECT(result.diagnostics.size()==1);
    AR_EXPECT(result.diagnostics[0].code==DiagnosticCode::insufficient_spatial_support);
    AR_EXPECT(result.diagnostics[0].input_index==no_input_index);
}

AR_TEST(point_cloud_fit_handles_the_optional_sparse_and_wide_reference_pair) {
    const std::string narrow_path=environment_value("ALIGNMENT_RECONSTRUCT_EXTERNAL_SPARSE_CENTERLINE_CSV");
    const std::string wide_path=environment_value("ALIGNMENT_RECONSTRUCT_EXTERNAL_WIDE_CLOUD_CSV");
    if (narrow_path.empty() || wide_path.empty()) {
        test_support::skip("external sparse centerline and wide cloud pair was not requested");
    }
    const auto reference=read_comma_triples(narrow_path,1.0);
    const auto cloud=read_comma_triples(wide_path,1.0);
    for (const auto* input : {&reference, &cloud}) {
        const auto fitted=fit_point_cloud(*input,5.0);
        AR_EXPECT(fitted.ok());
        auto reversed=*input;
        std::reverse(reversed.begin(),reversed.end());
        const auto reverse_result=fit_point_cloud(reversed,5.0);
        AR_EXPECT(reverse_result.ok());
        expect_same_points(fitted.centerline_points,reverse_result.centerline_points,0.0);
        const auto error=reference_error(fitted.centerline_points,reference);
        const bool narrow=input==&reference;
        AR_EXPECT(error.maximum_plan_m < (narrow ? 0.005 : 4.0));
        AR_EXPECT(error.rms_plan_m < (narrow ? 0.001 : 1.5));
        AR_EXPECT(error.maximum_height_m < (narrow ? 0.0001 : 0.5));
        AR_EXPECT(error.rms_height_m < (narrow ? 0.00001 : 0.15));
        std::cout << "  input=" << input->size() << ", output=" << fitted.centerline_points.size()
            << ", max/RMS XY=" << error.maximum_plan_m << '/' << error.rms_plan_m
            << " m, max/RMS H=" << error.maximum_height_m << '/' << error.rms_height_m << " m\n";
    }
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
    const std::vector<Point3D> public_contract_points =
        swap_plan_coordinates(points);
    const std::vector<Point3D> public_contract_expected =
        swap_plan_coordinates(expected);
    const auto public_contract_result =
        fit_point_cloud(public_contract_points, 10.0);

    AR_EXPECT(result.ok());
    AR_EXPECT(reversed_result.ok());
    AR_EXPECT(public_contract_result.ok());
    AR_EXPECT(expected.size() == 262);
    expect_same_points(result.centerline_points, expected, 1.0e-7);
    expect_same_points(
        public_contract_result.centerline_points,
        public_contract_expected,
        1.0e-7);
    expect_same_points(
        result.centerline_points,
        reversed_result.centerline_points,
        0.0);
}
