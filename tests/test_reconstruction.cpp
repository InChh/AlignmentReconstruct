#include "test_framework.h"

#include "alignment_reconstruct/adjustment.h"
#include "alignment_reconstruct/reconstruction.h"
#include "geometry/vector_math.h"
#include "horizontal/horizontal_alignment.h"
#include "vertical/vertical_alignment.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace alignment_reconstruct;

namespace {

double signed_offset_to_directed_line(
    const Point2D& point,
    const Point2D& line_start,
    const Point2D& line_end) {
    const double delta_x = line_end.x - line_start.x;
    const double delta_y = line_end.y - line_start.y;
    const double length = std::hypot(delta_x, delta_y);
    return (delta_x * (point.y - line_start.y) -
            delta_y * (point.x - line_start.x)) / length;
}

Point2D rotate_point(const Point2D& point, double angle_rad) {
    return {
        point.x * std::cos(angle_rad) - point.y * std::sin(angle_rad),
        point.x * std::sin(angle_rad) + point.y * std::cos(angle_rad)
    };
}

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

AR_TEST(public_reconstruction_returns_complete_horizontal_and_vertical_schemes) {
    constexpr double rotation_rad = 0.3;
    AlignmentDefinition expected;
    expected.horizontal = {
        {HorizontalControlPointKind::start,
         rotate_point({0.0, 0.0}, rotation_rad), 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::intersection,
         rotate_point({1200.0, 0.0}, rotation_rad), 600.0, 80.0, 70.0},
        {HorizontalControlPointKind::end,
         rotate_point({1800.0, 900.0}, rotation_rad), 0.0, 0.0, 0.0}
    };
    const detail::HorizontalAlignment horizontal(expected.horizontal, 0.0);
    expected.vertical = {
        {0.0, 100.0, 0.0},
        {1000.0, 110.0, 10000.0},
        {horizontal.end_station_m(), 105.0, 0.0}
    };
    const detail::VerticalAlignment vertical(expected.vertical);

    std::vector<Point3D> samples;
    for (double station = 0.0; station < horizontal.end_station_m(); station += 10.0) {
        const auto h = horizontal.evaluate(station);
        samples.push_back({h.point.x, h.point.y, vertical.evaluate(station).elevation_m});
    }
    const auto h_end = horizontal.evaluate(horizontal.end_station_m());
    samples.push_back({h_end.point.x, h_end.point.y,
                       vertical.evaluate(horizontal.end_station_m()).elevation_m});

    ReconstructionOptions options;
    options.vertical_second_difference_threshold_candidates = {1.0e-6};
    options.grade_rounding_per_mille = 0.0;
    options.maximum_up_grade = 0.0;
    options.maximum_down_grade = 0.0;
    options.maximum_grade_difference_per_mille = 0.0;
    options.minimum_vertical_curve_grade_difference_per_mille = 0.0;
    const auto result = reconstruct(samples, options);
    AR_EXPECT(result.ok());
    AR_EXPECT(result.horizontal.size() == 3);
    AR_EXPECT(result.vertical.size() == 3);
    AR_EXPECT(result.projected_stations_m.size() == samples.size());

    AlignmentDefinition reconstructed;
    reconstructed.start_station_m = options.start_station_m;
    reconstructed.horizontal = result.horizontal;
    reconstructed.vertical = result.vertical;
    const auto adjustments = calculate_adjustments(samples, reconstructed);
    AR_EXPECT(adjustments.ok());
    double maximum_offset = 0.0;
    double maximum_lift_drop = 0.0;
    for (const auto& point : adjustments.points) {
        maximum_offset = std::max(maximum_offset, std::abs(point.horizontal_offset_m));
        maximum_lift_drop = std::max(maximum_lift_drop, std::abs(point.lift_drop_m));
    }
    AR_EXPECT(maximum_offset < 0.02);
    AR_EXPECT(maximum_lift_drop < 0.12);
}

AR_TEST(public_reconstruction_reports_order_and_finite_input_errors) {
    auto result = reconstruct({{0.0, 0.0, 0.0}});
    AR_EXPECT(!result.ok());
    AR_EXPECT(result.diagnostics.front().code == DiagnosticCode::too_few_points);

    result = reconstruct({{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}});
    AR_EXPECT(!result.ok());
    AR_EXPECT(result.diagnostics.front().code ==
              DiagnosticCode::coincident_consecutive_points);
}

AR_TEST(public_adjustment_after_reconstructed_intersection_move_matches_tangent_geometry) {
    constexpr double rotation_rad = 0.3;
    const std::vector<HorizontalControlPoint> seed_controls{
        {HorizontalControlPointKind::start,
         rotate_point({0.0, 0.0}, rotation_rad), 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::intersection,
         rotate_point({1200.0, 0.0}, rotation_rad), 600.0, 80.0, 70.0},
        {HorizontalControlPointKind::end,
         rotate_point({1800.0, 900.0}, rotation_rad), 0.0, 0.0, 0.0}
    };
    const detail::HorizontalAlignment seed_alignment(seed_controls, 0.0);

    std::vector<Point3D> reconstruction_samples;
    for (double station = 0.0; station < seed_alignment.end_station_m(); station += 10.0) {
        const auto state = seed_alignment.evaluate(station);
        reconstruction_samples.push_back({state.point.x, state.point.y, 100.0});
    }
    const auto seed_end = seed_alignment.evaluate(seed_alignment.end_station_m());
    reconstruction_samples.push_back({seed_end.point.x, seed_end.point.y, 100.0});

    ReconstructionOptions options;
    options.vertical_second_difference_threshold_candidates = {1.0e-6};
    options.grade_rounding_per_mille = 0.0;
    const auto reconstruction = reconstruct(reconstruction_samples, options);
    AR_EXPECT(reconstruction.ok());
    AR_EXPECT(reconstruction.horizontal.size() == 3);

    AlignmentDefinition definition;
    definition.start_station_m = options.start_station_m;
    definition.horizontal = reconstruction.horizontal;
    definition.vertical = {{0.0, 100.0, 0.0}, {10000.0, 100.0, 0.0}};

    const detail::HorizontalAlignment reconstructed_alignment(
        definition.horizontal, definition.start_station_m);
    const auto incoming_probe_state = reconstructed_alignment.evaluate(
        reconstructed_alignment.start_station_m() + 200.0);
    const auto outgoing_probe_state = reconstructed_alignment.evaluate(
        reconstructed_alignment.end_station_m() - 200.0);
    const std::vector<Point3D> measured_points{
        {incoming_probe_state.point.x, incoming_probe_state.point.y, 100.0},
        {outgoing_probe_state.point.x, outgoing_probe_state.point.y, 100.0}
    };

    const auto before_move = calculate_adjustments(measured_points, definition);
    AR_EXPECT(before_move.ok());
    AR_EXPECT_NEAR(before_move.points[0].horizontal_offset_m, 0.0, 1.0e-9);
    AR_EXPECT_NEAR(before_move.points[1].horizontal_offset_m, 0.0, 1.0e-9);

    definition.horizontal[1].position.y += 25.0;
    const auto all_points_after_move = calculate_adjustments(reconstruction_samples, definition);
    AR_EXPECT(all_points_after_move.ok());
    AR_EXPECT(all_points_after_move.points.size() == reconstruction_samples.size());
    double maximum_full_route_offset_m = 0.0;
    for (const auto& point : all_points_after_move.points) {
        maximum_full_route_offset_m = std::max(
            maximum_full_route_offset_m, std::abs(point.horizontal_offset_m));
    }
    AR_EXPECT(maximum_full_route_offset_m > 1.0);

    const auto after_move = calculate_adjustments(measured_points, definition);
    AR_EXPECT(after_move.ok());
    AR_EXPECT(after_move.points.size() == measured_points.size());

    const Point2D incoming_point{measured_points[0].x, measured_points[0].y};
    const Point2D outgoing_point{measured_points[1].x, measured_points[1].y};
    const double expected_incoming_offset = signed_offset_to_directed_line(
        incoming_point,
        definition.horizontal[0].position,
        definition.horizontal[1].position);
    const double expected_outgoing_offset = signed_offset_to_directed_line(
        outgoing_point,
        definition.horizontal[1].position,
        definition.horizontal[2].position);

    AR_EXPECT(after_move.points[0].horizontal_element == HorizontalElementKind::straight);
    AR_EXPECT(after_move.points[1].horizontal_element == HorizontalElementKind::straight);
    AR_EXPECT_NEAR(after_move.points[0].horizontal_offset_m,
                   expected_incoming_offset, 1.0e-8);
    AR_EXPECT_NEAR(after_move.points[1].horizontal_offset_m,
                   expected_outgoing_offset, 1.0e-8);
    AR_EXPECT_NEAR(after_move.points[0].horizontal_offset_mm(),
                   expected_incoming_offset * 1000.0, 1.0e-5);
    AR_EXPECT_NEAR(after_move.points[1].horizontal_offset_mm(),
                   expected_outgoing_offset * 1000.0, 1.0e-5);
    AR_EXPECT(std::abs(expected_incoming_offset) > 1.0);
    AR_EXPECT(std::abs(expected_outgoing_offset) > 1.0);
    std::cout << std::setprecision(12)
              << "[METRIC] reconstructed intersection move: delta_y_m=25"
              << ", incoming_actual_m=" << after_move.points[0].horizontal_offset_m
              << ", incoming_expected_m=" << expected_incoming_offset
              << ", outgoing_actual_m=" << after_move.points[1].horizontal_offset_m
              << ", outgoing_expected_m=" << expected_outgoing_offset
              << ", full_route_max_abs_m=" << maximum_full_route_offset_m << '\n';
}

AR_TEST(public_reconstruction_runs_an_external_xyz_fixture) {
    const std::string input_path = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_XYZ_CSV");
    if (input_path.empty()) {
        test_support::skip(
            "External XYZ reconstruction fixture was not requested");
    }
    std::ifstream input(input_path);
    AR_EXPECT(input.good());
    std::string row;
    std::getline(input, row);
    std::vector<Point3D> samples;
    while (std::getline(input, row)) {
        std::istringstream fields(row);
        std::string ignored_index;
        std::string x;
        std::string y;
        std::string z;
        std::getline(fields, ignored_index, ',');
        std::getline(fields, x, ',');
        std::getline(fields, y, ',');
        std::getline(fields, z, ',');
        samples.push_back({std::stod(x), std::stod(y), std::stod(z)});
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
    const auto result = reconstruct(samples, options);
    if (!result.ok()) {
        std::string message = "External reconstruction failed";
        if (!result.diagnostics.empty()) {
            message += ": " + result.diagnostics.front().message;
        }
        throw std::runtime_error(message);
    }

    const std::string reference_path = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_VERTICAL_REFERENCE_CSV");
    if (!reference_path.empty()) {
        struct ExpectedVerticalControl {
            double station_m = 0.0;
            double elevation_m = 0.0;
            double vertical_curve_radius_m = 0.0;
        };
        const auto parse_number = [](std::string field) {
            if (!field.empty() && field.back() == '\r') {
                field.pop_back();
            }
            if (field.size() >= 2 &&
                field.front() == '"' &&
                field.back() == '"') {
                field = field.substr(1, field.size() - 2);
            }
            return std::stod(field);
        };

        std::ifstream reference(reference_path);
        AR_EXPECT(reference.good());
        std::getline(reference, row);
        std::vector<ExpectedVerticalControl> expected;
        while (std::getline(reference, row)) {
            std::istringstream fields(row);
            std::string ignored_index;
            std::string station;
            std::string elevation;
            std::string radius;
            std::getline(fields, ignored_index, ',');
            std::getline(fields, station, ',');
            std::getline(fields, elevation, ',');
            std::getline(fields, radius, ',');
            expected.push_back({
                parse_number(station),
                parse_number(elevation),
                parse_number(radius)
            });
        }

        AR_EXPECT(result.vertical.size() == expected.size());
        double maximum_station_error_m = 0.0;
        double maximum_elevation_error_m = 0.0;
        double maximum_radius_error_m = 0.0;
        for (std::size_t index = 0; index < expected.size(); ++index) {
            maximum_station_error_m = std::max(
                maximum_station_error_m,
                std::abs(result.vertical[index].station_m -
                         expected[index].station_m));
            maximum_elevation_error_m = std::max(
                maximum_elevation_error_m,
                std::abs(result.vertical[index].elevation_m -
                         expected[index].elevation_m));
            maximum_radius_error_m = std::max(
                maximum_radius_error_m,
                std::abs(result.vertical[index].vertical_curve_radius_m -
                         expected[index].vertical_curve_radius_m));
        }
        std::cout << std::setprecision(12)
                  << "[METRIC] external vertical reference parity:"
                  << " controls=" << expected.size()
                  << ", station_max_abs_m=" << maximum_station_error_m
                  << ", elevation_max_abs_m=" << maximum_elevation_error_m
                  << ", radius_max_abs_m=" << maximum_radius_error_m << '\n';
        AR_EXPECT(maximum_station_error_m <= 1.0e-5);
        AR_EXPECT(maximum_elevation_error_m <= 1.0e-5);
        AR_EXPECT(maximum_radius_error_m <= 1.0e-9);
    }

    const std::string output_path = environment_value(
        "ALIGNMENT_RECONSTRUCT_EXTERNAL_RECONSTRUCTION_OUTPUT");
    if (!output_path.empty()) {
        std::ofstream output(output_path);
        AR_EXPECT(output.good());
        output
            << "control_index,station_m,elevation_m,"
               "vertical_curve_radius_m\n";
        output << std::setprecision(17);
        for (std::size_t index = 0; index < result.vertical.size(); ++index) {
            output << index << ','
                   << result.vertical[index].station_m << ','
                   << result.vertical[index].elevation_m << ','
                   << result.vertical[index].vertical_curve_radius_m << '\n';
        }
    }
    std::cout << "[METRIC] external XYZ reconstruction: samples="
              << samples.size()
              << ", horizontal_controls=" << result.horizontal.size()
              << ", vertical_controls=" << result.vertical.size() << '\n';
}
