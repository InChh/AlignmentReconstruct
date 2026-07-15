#include "fixture_reader.h"
#include "test_framework.h"

#include "alignment_reconstruct/adjustment.h"
#include "geometry/vector_math.h"
#include "horizontal/horizontal_alignment.h"
#include "vertical/vertical_alignment.h"

#include <limits>
#include <vector>

using namespace alignment_reconstruct;

namespace {

AlignmentDefinition curved_definition() {
    AlignmentDefinition definition;
    definition.start_station_m = 500.0;
    definition.horizontal = {
        {HorizontalControlPointKind::start, {0.0, 0.0}, 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::intersection, {1000.0, 0.0}, 600.0, 70.0, 90.0},
        {HorizontalControlPointKind::end, {1500.0, 700.0}, 0.0, 0.0, 0.0}
    };
    definition.vertical = {
        {500.0, 100.0, 0.0},
        {1300.0, 108.0, 10000.0},
        {3000.0, 103.0, 0.0}
    };
    return definition;
}

} // namespace

AR_TEST(adjustment_uses_one_horizontal_station_for_offset_and_lift_drop) {
    const AlignmentDefinition definition = curved_definition();
    const detail::HorizontalAlignment horizontal(
        definition.horizontal, definition.start_station_m);
    const detail::VerticalAlignment vertical(definition.vertical);

    std::vector<Point3D> measured;
    const std::vector<double> expected_offsets{2.0, -3.5, 0.0};
    const std::vector<double> expected_lift_drop{0.04, -0.025, 0.0};
    const std::vector<double> stations{700.0, 1320.0, 2050.0};
    for (std::size_t index = 0; index < stations.size(); ++index) {
        const auto horizontal_state = horizontal.evaluate(stations[index]);
        const auto vertical_state = vertical.evaluate(stations[index]);
        const Point2D right_normal{
            -std::sin(horizontal_state.heading_rad),
            std::cos(horizontal_state.heading_rad)
        };
        measured.push_back({
            horizontal_state.point.x + expected_offsets[index] * right_normal.x,
            horizontal_state.point.y + expected_offsets[index] * right_normal.y,
            vertical_state.elevation_m - expected_lift_drop[index]
        });
    }

    const auto result = calculate_adjustments(measured, definition);
    AR_EXPECT(result.ok());
    AR_EXPECT(result.points.size() == measured.size());
    for (std::size_t index = 0; index < result.points.size(); ++index) {
        const auto& point = result.points[index];
        AR_EXPECT(point.point_index == index);
        AR_EXPECT_NEAR(point.station_m, stations[index], 1.0e-7);
        AR_EXPECT_NEAR(point.horizontal_offset_m, expected_offsets[index], 1.0e-8);
        AR_EXPECT_NEAR(point.lift_drop_m, expected_lift_drop[index], 1.0e-9);
        AR_EXPECT_NEAR(point.design_elevation_m,
                       vertical.evaluate(point.station_m).elevation_m, 1.0e-12);
        AR_EXPECT_NEAR(point.horizontal_offset_mm(), expected_offsets[index] * 1000.0, 1.0e-5);
        AR_EXPECT_NEAR(point.lift_drop_mm(), expected_lift_drop[index] * 1000.0, 1.0e-6);
    }
}

AR_TEST(adjustment_horizontal_result_is_independent_of_measured_z) {
    const auto directory = test_support::fixture_directory();
    const auto horizontal_rows = test_support::read_horizontal_fixture(
        directory / "k1413_k1462_horizontal.csv");
    const auto offset_rows = test_support::read_offset_fixture(
        directory / "k1413_k1462_offsets.csv");
    AlignmentDefinition definition;
    for (const auto& row : horizontal_rows) {
        definition.horizontal.push_back(row.control_point);
    }
    definition.vertical = {{0.0, 10.0, 0.0}, {60000.0, 10.0, 0.0}};

    std::vector<Point3D> first;
    std::vector<Point3D> second;
    for (std::size_t index = 0; index < offset_rows.size(); index += 400) {
        const auto& row = offset_rows[index];
        first.push_back({row.point.x, row.point.y, 0.0});
        second.push_back({row.point.x, row.point.y, 123.456});
    }
    const auto& last = offset_rows.back();
    first.push_back({last.point.x, last.point.y, 0.0});
    second.push_back({last.point.x, last.point.y, 123.456});
    const auto first_result = calculate_adjustments(first, definition);
    const auto second_result = calculate_adjustments(second, definition);
    AR_EXPECT(first_result.ok());
    AR_EXPECT(second_result.ok());
    AR_EXPECT(first_result.points.size() == second_result.points.size());
    for (std::size_t index = 0; index < first_result.points.size(); ++index) {
        AR_EXPECT_NEAR(first_result.points[index].station_m,
                       second_result.points[index].station_m, 0.0);
        AR_EXPECT_NEAR(first_result.points[index].horizontal_offset_m,
                       second_result.points[index].horizontal_offset_m, 0.0);
    }
}

AR_TEST(adjustment_returns_diagnostics_for_invalid_inputs_and_vertical_coverage) {
    AlignmentDefinition definition = curved_definition();
    definition.vertical.back().station_m = 1800.0;
    auto result = calculate_adjustments({}, definition);
    AR_EXPECT(!result.ok());
    AR_EXPECT(result.status == StatusCode::invalid_input);
    AR_EXPECT(result.diagnostics.front().code == DiagnosticCode::vertical_range_mismatch);

    definition = curved_definition();
    result = calculate_adjustments({
        {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0}
    }, definition);
    AR_EXPECT(!result.ok());
    AR_EXPECT(result.diagnostics.front().code == DiagnosticCode::non_finite_value);
    AR_EXPECT(result.diagnostics.front().input_index == 0);

    AlignmentDefinition overlapping;
    overlapping.horizontal = {
        {HorizontalControlPointKind::start, {0.0, 0.0}, 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::end, {300.0, 0.0}, 0.0, 0.0, 0.0}
    };
    overlapping.vertical = {
        {0.0, 0.0, 0.0},
        {100.0, 10.0, 10000.0},
        {200.0, 0.0, 10000.0},
        {300.0, 10.0, 0.0}
    };
    result = calculate_adjustments({}, overlapping);
    AR_EXPECT(!result.ok());
    AR_EXPECT(result.status == StatusCode::infeasible_geometry);
    AR_EXPECT(result.diagnostics.front().code == DiagnosticCode::overlapping_vertical_curves);
}

AR_TEST(adjustment_definition_validation_uses_stable_diagnostic_codes) {
    AlignmentDefinition definition = curved_definition();
    definition.horizontal.resize(1);
    auto result = calculate_adjustments({}, definition);
    AR_EXPECT(result.diagnostics.front().code ==
              DiagnosticCode::too_few_horizontal_control_points);

    definition = curved_definition();
    definition.horizontal[1].radius_m = 0.0;
    result = calculate_adjustments({}, definition);
    AR_EXPECT(result.diagnostics.front().code == DiagnosticCode::invalid_horizontal_radius);

    definition = curved_definition();
    definition.horizontal[1].entry_spiral_length_m = -1.0;
    result = calculate_adjustments({}, definition);
    AR_EXPECT(result.diagnostics.front().code == DiagnosticCode::invalid_spiral_length);

    definition = curved_definition();
    definition.vertical.resize(1);
    result = calculate_adjustments({}, definition);
    AR_EXPECT(result.diagnostics.front().code ==
              DiagnosticCode::too_few_vertical_control_points);

    definition = curved_definition();
    definition.vertical[1].station_m = definition.vertical[0].station_m;
    result = calculate_adjustments({}, definition);
    AR_EXPECT(result.diagnostics.front().code == DiagnosticCode::non_monotonic_station);

    definition = curved_definition();
    definition.vertical[1].vertical_curve_radius_m = 50.0;
    result = calculate_adjustments({}, definition);
    AR_EXPECT(result.diagnostics.front().code == DiagnosticCode::invalid_vertical_radius);
}
