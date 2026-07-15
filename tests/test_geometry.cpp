#include "test_framework.h"

#include "geometry/line_fit.h"
#include "horizontal/horizontal_element.h"
#include "legacy_reference/legacy_horizontal_geometry.h"

#include <cmath>
#include <vector>

using alignment_reconstruct::HorizontalElementKind;
using alignment_reconstruct::Point2D;
using alignment_reconstruct::detail::HorizontalElement;

namespace {

void expect_state_near(
    const alignment_reconstruct::detail::HorizontalElementState& actual,
    const legacy_reference::State& expected,
    double coordinate_tolerance) {
    AR_EXPECT_NEAR(actual.point.x, expected.point.x, coordinate_tolerance);
    AR_EXPECT_NEAR(actual.point.y, expected.point.y, coordinate_tolerance);
    AR_EXPECT_NEAR(std::sin(actual.heading_rad), std::sin(expected.heading_rad), 1.0e-10);
    AR_EXPECT_NEAR(std::cos(actual.heading_rad), std::cos(expected.heading_rad), 1.0e-10);
}

} // namespace

AR_TEST(horizontal_elements_match_active_legacy_equations) {
    const Point2D start{1234.5, -678.25};
    const double heading = 0.73;

    const HorizontalElement line(
        HorizontalElementKind::straight, 0.0, start, heading, 125.0, 0.0, 0.0);
    expect_state_near(line.evaluate(83.25), legacy_reference::line(start, heading, 83.25), 1.0e-9);

    const HorizontalElement circle(
        HorizontalElementKind::circular, 0.0, start, heading, 180.0,
        1.0 / 600.0, 1.0 / 600.0);
    expect_state_near(
        circle.evaluate(147.0), legacy_reference::circle(start, heading, 600.0, 1, 147.0),
        1.0e-9);

    const double spiral_length = 80.0;
    const double radius = 500.0;
    const HorizontalElement entry(
        HorizontalElementKind::entry_spiral, 0.0, start, heading, spiral_length,
        0.0, -1.0 / radius);
    const HorizontalElement exit(
        HorizontalElementKind::exit_spiral, 0.0, start, heading, spiral_length,
        -1.0 / radius, 0.0);
    for (const double length : {0.0, 7.5, 40.0, 79.0, 80.0}) {
        expect_state_near(entry.evaluate(length),
                          legacy_reference::entry_spiral(
                              start, heading, radius, spiral_length, -1, length),
                          1.0e-7);
        expect_state_near(exit.evaluate(length),
                          legacy_reference::exit_spiral(
                              start, heading, radius, spiral_length, -1, length),
                          1.0e-7);
    }
}

AR_TEST(orthogonal_line_fit_preserves_input_direction_and_error) {
    const std::vector<Point2D> points{
        {1.0, 5.0}, {3.0, 9.0}, {5.0, 13.0}, {7.0, 17.0}
    };
    const auto fit = alignment_reconstruct::detail::fit_orthogonal_line(points);
    AR_EXPECT(fit.valid);
    AR_EXPECT(fit.unit_direction.x > 0.0);
    AR_EXPECT(fit.unit_direction.y > 0.0);
    AR_EXPECT_NEAR(fit.centroid.x, 4.0, 1.0e-12);
    AR_EXPECT_NEAR(fit.centroid.y, 11.0, 1.0e-12);
    AR_EXPECT_NEAR(fit.rms_orthogonal_error_m, 0.0, 1.0e-12);
}

AR_TEST(horizontal_element_connections_are_position_tangent_and_curvature_continuous) {
    const Point2D start{0.0, 0.0};
    const double radius = 800.0;
    const double spiral_length = 100.0;
    const HorizontalElement entry(
        HorizontalElementKind::entry_spiral, 10.0, start, 0.0, spiral_length,
        0.0, 1.0 / radius);
    const auto entry_end = entry.evaluate(spiral_length);
    const HorizontalElement circle(
        HorizontalElementKind::circular, entry.end_station_m(), entry_end.point,
        entry_end.heading_rad, 200.0, 1.0 / radius, 1.0 / radius);
    const auto circle_start = circle.evaluate(0.0);
    AR_EXPECT_NEAR(circle_start.point.x, entry_end.point.x, 1.0e-12);
    AR_EXPECT_NEAR(circle_start.point.y, entry_end.point.y, 1.0e-12);
    AR_EXPECT_NEAR(circle_start.heading_rad, entry_end.heading_rad, 1.0e-12);
    AR_EXPECT_NEAR(circle_start.curvature_inverse_m, entry_end.curvature_inverse_m, 1.0e-12);
}
