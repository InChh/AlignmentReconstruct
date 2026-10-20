#include "fixture_reader.h"
#include "test_framework.h"

#include "alignment_reconstruct/adjustment.h"
#include "alignment_reconstruct/reconstruction.h"
#include "geometry/vector_math.h"
#include "horizontal/alignment_optimizer.h"
#include "horizontal/curvature_classifier.h"
#include "horizontal/horizontal_alignment.h"
#include "horizontal/horizontal_reconstructor.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

using namespace alignment_reconstruct;

namespace {

Point2D rotate_noisy_reconstruction_point(const Point2D& point) {
    constexpr double angle_rad = 0.4;
    return {
        point.x * std::cos(angle_rad) - point.y * std::sin(angle_rad),
        point.x * std::sin(angle_rad) + point.y * std::cos(angle_rad)
    };
}

std::vector<HorizontalControlPoint> noisy_reconstruction_controls(double scale) {
    std::vector<HorizontalControlPoint> controls{
        {HorizontalControlPointKind::start, {0.0, 0.0}, 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::intersection, {800.0 * scale, 0.0},
         1000.0 * scale, 80.0 * scale, 60.0 * scale},
        {HorizontalControlPointKind::intersection, {1500.0 * scale, 800.0 * scale},
         1000.0 * scale, 80.0 * scale, 60.0 * scale},
        {HorizontalControlPointKind::end, {2300.0 * scale, 800.0 * scale},
         0.0, 0.0, 0.0}
    };
    for (auto& control : controls) {
        control.position = rotate_noisy_reconstruction_point(control.position);
    }
    return controls;
}

std::vector<Point2D> noisy_reconstruction_samples(
    const detail::HorizontalAlignment& source,
    double interval_m,
    double noise_length_scale_m,
    double amplitude_m = 0.1) {
    std::vector<Point2D> points;
    for (double station_m = 0.0; station_m < source.end_station_m();
         station_m += interval_m) {
        const auto state = source.evaluate(station_m);
        const double noise_m = amplitude_m * (
            std::sin(station_m / noise_length_scale_m) +
            0.4 * std::sin(station_m / (0.35 * noise_length_scale_m)));
        points.push_back({
            state.point.x - noise_m * std::sin(state.heading_rad),
            state.point.y + noise_m * std::cos(state.heading_rad)
        });
    }
    points.push_back(source.evaluate(source.end_station_m()).point);
    return points;
}

} // namespace

AR_TEST(horizontal_reconstruction_recovers_noisy_curves_after_dynamic_collapse) {
    struct TestCase {
        double scale;
        double interval_m;
        double noise_length_scale_m;
    };
    for (const TestCase& test_case : {
             TestCase{0.6, 5.0, 6.0}, TestCase{1.0, 5.0, 12.0},
             TestCase{1.5, 10.0, 23.0}}) {
        const auto controls = noisy_reconstruction_controls(test_case.scale);
        const detail::HorizontalAlignment source(controls, 0.0);
        const auto points = noisy_reconstruction_samples(
            source, test_case.interval_m, test_case.noise_length_scale_m);
        ReconstructionOptions options;
        options.curvature_threshold_inverse_m = 0.0001;
        const auto actual = detail::HorizontalReconstructor(options).reconstruct(points);
        AR_EXPECT(actual.size() == controls.size());
        for (std::size_t index = 1; index + 1 < actual.size(); ++index) {
            AR_EXPECT_NEAR(actual[index].radius_m, controls[index].radius_m,
                0.05 * controls[index].radius_m);
            AR_EXPECT(actual[index].entry_spiral_length_m > 0.0);
            AR_EXPECT(actual[index].exit_spiral_length_m > 0.0);
        }
        const detail::HorizontalAlignment rebuilt(actual, 0.0);
        double maximum_error_m = 0.0;
        double squared_error_m2 = 0.0;
        std::size_t count = 0;
        for (double station_m = 0.0; station_m <= source.end_station_m();
             station_m += 5.0) {
            const auto projected = rebuilt.project(source.evaluate(station_m).point);
            AR_EXPECT(projected.valid);
            maximum_error_m = std::max(maximum_error_m, projected.distance_m);
            squared_error_m2 += projected.distance_m * projected.distance_m;
            ++count;
        }
        const double rms_error_m = std::sqrt(squared_error_m2 / static_cast<double>(count));
        std::cout << "  noisy curves R=" << 1000.0 * test_case.scale
                  << ", interval=" << test_case.interval_m
                  << ", noise length scale=" << test_case.noise_length_scale_m
                  << ", reference max/RMS=" << maximum_error_m << '/' << rms_error_m << " m\n";
        AR_EXPECT(maximum_error_m < 0.5);
        AR_EXPECT(rms_error_m < 0.15);
    }
}

AR_TEST(horizontal_reconstruction_does_not_adopt_an_invalid_wider_window_candidate) {
    const detail::HorizontalAlignment source(noisy_reconstruction_controls(1.0), 0.0);
    const auto points = noisy_reconstruction_samples(source, 5.0, 12.0);
    ReconstructionOptions options;
    options.curvature_threshold_inverse_m = 0.0001;
    options.minimum_curve_sample_count = points.size() + 1;
    const auto actual = detail::HorizontalReconstructor(options).reconstruct(points);
    AR_EXPECT(actual.size() == 2);
}

AR_TEST(horizontal_reconstruction_recovers_noisy_curves_at_oblique_large_coordinates) {
    for (const double angle_deg : {23.0, 45.0}) {
        const double angle_rad = angle_deg * std::acos(-1.0) / 180.0 - 0.4;
        auto controls = noisy_reconstruction_controls(1.0);
        for (auto& control : controls) {
            const Point2D point = control.position;
            control.position = {
                3600000.0 + point.x * std::cos(angle_rad) - point.y * std::sin(angle_rad),
                560000.0 + point.x * std::sin(angle_rad) + point.y * std::cos(angle_rad)
            };
        }
        const detail::HorizontalAlignment source(controls, 0.0);
        const auto noisy_points = noisy_reconstruction_samples(source, 5.0, 12.0);
        std::vector<Point3D> points;
        for (const auto& point : noisy_points) {
            points.push_back({point.x, point.y, 100.0});
        }
        ReconstructionOptions options;
        options.curvature_threshold_inverse_m = 0.0001;
        const auto actual = reconstruct(points, options);
        AR_EXPECT(actual.ok());
        AR_EXPECT(actual.horizontal.size() == controls.size());

        std::vector<Point3D> reference;
        for (double station_m = 0.0; station_m <= source.end_station_m(); station_m += 5.0) {
            const auto point = source.evaluate(station_m).point;
            reference.push_back({point.x, point.y, 100.0});
        }
        const auto adjustments = calculate_adjustments(
            reference, {options.start_station_m, actual.horizontal, actual.vertical});
        AR_EXPECT(adjustments.ok());
        AR_EXPECT(adjustments.points.size() == reference.size());
        double maximum_error_m = 0.0;
        double squared_error_m2 = 0.0;
        for (std::size_t index = 0; index < adjustments.points.size(); ++index) {
            const auto& adjustment = adjustments.points[index];
            const double error_m = std::hypot(
                reference[index].x - adjustment.projected_point.x,
                reference[index].y - adjustment.projected_point.y);
            maximum_error_m = std::max(maximum_error_m, error_m);
            squared_error_m2 += error_m * error_m;
            AR_EXPECT_NEAR(adjustment.lift_drop_m, 0.0, 1.0e-8);
        }
        const double rms_error_m = std::sqrt(
            squared_error_m2 / static_cast<double>(adjustments.points.size()));
        std::cout << "  oblique noisy curves heading=" << angle_deg
                  << ", reference max/RMS=" << maximum_error_m << '/' << rms_error_m << " m\n";
        AR_EXPECT(maximum_error_m < 0.5);
        AR_EXPECT(rms_error_m < 0.15);
    }
}

AR_TEST(horizontal_reconstruction_bounds_geometric_error_for_noisy_short_spirals) {
    auto controls = noisy_reconstruction_controls(1.0);
    for (std::size_t index = 1; index + 1 < controls.size(); ++index) {
        controls[index].entry_spiral_length_m = 10.0;
        controls[index].exit_spiral_length_m = 10.0;
    }
    const detail::HorizontalAlignment source(controls, 0.0);
    const auto points = noisy_reconstruction_samples(source, 5.0, 23.0, 0.02);
    ReconstructionOptions options;
    options.curvature_threshold_inverse_m = 0.0001;
    const auto actual = detail::HorizontalReconstructor(options).reconstruct(points);
    AR_EXPECT(actual.size() == controls.size());
    for (std::size_t index = 1; index + 1 < actual.size(); ++index) {
        AR_EXPECT_NEAR(actual[index].radius_m, 1000.0, 5.0);
        AR_EXPECT_NEAR(actual[index].entry_spiral_length_m, 10.0, 20.0);
        AR_EXPECT_NEAR(actual[index].exit_spiral_length_m, 10.0, 20.0);
        std::cout << "  short spiral R/L1/L2=" << actual[index].radius_m << '/'
                  << actual[index].entry_spiral_length_m << '/'
                  << actual[index].exit_spiral_length_m << " m\n";
    }
    const detail::HorizontalAlignment rebuilt(actual, 0.0);
    double maximum_error_m = 0.0;
    double squared_error_m2 = 0.0;
    std::size_t count = 0;
    for (double station_m = 0.0; station_m <= source.end_station_m(); station_m += 5.0) {
        const auto projected = rebuilt.project(source.evaluate(station_m).point);
        AR_EXPECT(projected.valid);
        maximum_error_m = std::max(maximum_error_m, projected.distance_m);
        squared_error_m2 += projected.distance_m * projected.distance_m;
        ++count;
    }
    const double rms_error_m = std::sqrt(squared_error_m2 / static_cast<double>(count));
    std::cout << "  short spiral reference max/RMS=" << maximum_error_m << '/'
              << rms_error_m << " m\n";
    AR_EXPECT(maximum_error_m < 0.1);
    AR_EXPECT(rms_error_m < 0.03);
}

AR_TEST(horizontal_reconstruction_keeps_noisy_straight_and_short_routes_straight) {
    for (const double length_m : {55.0, 1000.0}) {
        std::vector<Point2D> points;
        for (double station_m = 0.0; station_m <= length_m; station_m += 5.0) {
            const double noise_m = 0.1 * (std::sin(station_m / 6.0) +
                0.4 * std::sin(station_m / 2.1));
            points.push_back(rotate_noisy_reconstruction_point({station_m, noise_m}));
        }
        ReconstructionOptions options;
        options.curvature_threshold_inverse_m = 0.0001;
        const auto actual = detail::HorizontalReconstructor(options).reconstruct(points);
        AR_EXPECT(actual.size() == 2);
        AR_EXPECT(actual.front().kind == HorizontalControlPointKind::start);
        AR_EXPECT(actual.back().kind == HorizontalControlPointKind::end);
    }
}

AR_TEST(horizontal_reconstruction_handles_tangent_heading_wraparound) {
    for (const double angle_deg : {0.0, 45.0, 90.0, 179.0, 225.0, 270.0, 359.0}) {
        auto controls = noisy_reconstruction_controls(1.0);
        const double angle_rad = angle_deg * std::acos(-1.0) / 180.0 - 0.4;
        for (auto& control : controls) {
            const auto p = control.position;
            control.position = {
                480000.0 + p.x * std::cos(angle_rad) - p.y * std::sin(angle_rad),
                4730000.0 + p.x * std::sin(angle_rad) + p.y * std::cos(angle_rad)
            };
        }
        const detail::HorizontalAlignment source(controls, 0.0);
        const auto samples = noisy_reconstruction_samples(source, 5.0, 23.0, 0.02);
        ReconstructionOptions options;
        options.curvature_threshold_inverse_m = 0.0001;
        const auto analysis = detail::classify_horizontal_samples(samples, options);
        double maximum_curvature = 0.0;
        for (const auto value : analysis.curvature_inverse_m) {
            maximum_curvature = std::max(maximum_curvature, std::abs(value));
        }
        std::cout << "  wraparound heading=" << angle_deg
                  << ", max curvature=" << maximum_curvature << "\n";
        AR_EXPECT(maximum_curvature < 0.002);
        const auto fitted = detail::HorizontalReconstructor(options).reconstruct(samples);
        AR_EXPECT(fitted.size() == controls.size());
        const detail::HorizontalAlignment rebuilt(fitted, 0.0);
        double maximum_error_m = 0.0;
        for (double station_m = 0.0; station_m <= source.end_station_m(); station_m += 5.0) {
            const auto projection = rebuilt.project(source.evaluate(station_m).point);
            AR_EXPECT(projection.valid);
            maximum_error_m = std::max(maximum_error_m, projection.distance_m);
        }
        std::cout << "  wraparound plan max=" << maximum_error_m << " m\n";
        AR_EXPECT(maximum_error_m < 0.3);
    }
}

AR_TEST(horizontal_reconstruction_supports_exact_and_near_axis_tangents) {
    for (const double angle_rad : {0.0, 0.5 * detail::pi, detail::pi,
             1.5 * detail::pi, 0.25 * detail::pi, 0.5 * detail::pi - 0.0005}) {
        std::vector<Point3D> points;
        for (double station_m = 0.0; station_m <= 500.0; station_m += 5.0) {
            points.push_back({480000.0 + station_m * std::cos(angle_rad),
                4730000.0 + station_m * std::sin(angle_rad), 100.0});
        }
        const auto result = reconstruct(points);
        AR_EXPECT(result.ok());
        AR_EXPECT(result.horizontal.size() == 2);
        const auto offsets = calculate_adjustments(
            points, {0.0, result.horizontal, result.vertical});
        AR_EXPECT(offsets.ok());
        for (const auto& point : offsets.points) {
            AR_EXPECT_NEAR(point.horizontal_offset_m, 0.0, 1.0e-7);
        }
    }
}

AR_TEST(horizontal_reconstruction_recovers_noise_split_singleton_tangents) {
    for (const auto& radius_and_spiral : {
             std::pair<double, double>{600.0, 10.0}, {750.0, 40.0}}) {
        auto controls = noisy_reconstruction_controls(1.0);
        for (std::size_t index = 1; index + 1 < controls.size(); ++index) {
            controls[index].radius_m = radius_and_spiral.first;
            controls[index].entry_spiral_length_m = radius_and_spiral.second;
            controls[index].exit_spiral_length_m = 0.8 * radius_and_spiral.second;
        }
        const detail::HorizontalAlignment source(controls, 0.0);
        const auto samples = noisy_reconstruction_samples(source, 5.0, 6.0, 0.05);
        std::vector<Point3D> points;
        for (const auto& point : samples) {
            points.push_back({point.x, point.y, 100.0});
        }
        const auto result = reconstruct(points);
        AR_EXPECT(result.ok());
        AR_EXPECT(result.horizontal.size() == controls.size());
        std::vector<Point3D> reference;
        for (double station_m = 0.0; station_m <= source.end_station_m(); station_m += 5.0) {
            const auto point = source.evaluate(station_m).point;
            reference.push_back({point.x, point.y, 100.0});
        }
        const auto offsets = calculate_adjustments(
            reference, {0.0, result.horizontal, result.vertical});
        AR_EXPECT(offsets.ok());
        double maximum_error_m = 0.0;
        for (const auto& point : offsets.points) {
            maximum_error_m = std::max(maximum_error_m, std::abs(point.horizontal_offset_m));
            AR_EXPECT_NEAR(point.lift_drop_m, 0.0, 1.0e-8);
        }
        std::cout << "  singleton recovery R=" << radius_and_spiral.first
                  << ", reference max=" << maximum_error_m << " m\n";
        AR_EXPECT(maximum_error_m < 0.2);
    }
}

AR_TEST(horizontal_output_options_preserve_parameters_when_disabled) {
    const detail::HorizontalCurveParameters input{123.5, 45.5, 52.5, 9.25};

    const auto actual = detail::apply_horizontal_output_options(input, {});

    AR_EXPECT_NEAR(actual.radius_m, input.radius_m, 0.0);
    AR_EXPECT_NEAR(actual.entry_spiral_length_m, input.entry_spiral_length_m, 0.0);
    AR_EXPECT_NEAR(actual.exit_spiral_length_m, input.exit_spiral_length_m, 0.0);
    AR_EXPECT_NEAR(actual.mean_squared_offset_m2, input.mean_squared_offset_m2, 0.0);
}

AR_TEST(horizontal_output_options_match_the_reference_rounding_steps) {
    struct RadiusCase {
        double step_m;
        double expected_m;
    };
    const std::vector<RadiusCase> radius_cases{
        {1.0, 176.0}, {10.0, 180.0}, {50.0, 200.0}, {100.0, 200.0}
    };
    for (const auto& test_case : radius_cases) {
        ReconstructionOptions options;
        options.horizontal_curve_radius_rounding_m = test_case.step_m;
        const auto actual = detail::apply_horizontal_output_options(
            {176.0, 47.0, 63.0, 2.0}, options);
        AR_EXPECT_NEAR(actual.radius_m, test_case.expected_m, 1.0e-12);
    }

    struct SpiralCase {
        double step_m;
        double expected_entry_m;
        double expected_exit_m;
    };
    const std::vector<SpiralCase> spiral_cases{
        {1.0, 47.0, 63.0}, {5.0, 45.0, 65.0}, {10.0, 50.0, 60.0}
    };
    for (const auto& test_case : spiral_cases) {
        ReconstructionOptions options;
        options.horizontal_spiral_length_rounding_m = test_case.step_m;
        const auto actual = detail::apply_horizontal_output_options(
            {176.0, 47.0, 63.0, 2.0}, options);
        AR_EXPECT_NEAR(
            actual.entry_spiral_length_m, test_case.expected_entry_m, 1.0e-12);
        AR_EXPECT_NEAR(
            actual.exit_spiral_length_m, test_case.expected_exit_m, 1.0e-12);
    }
}

AR_TEST(horizontal_output_options_keep_exact_half_steps_on_the_lower_multiple) {
    ReconstructionOptions options;
    options.horizontal_curve_radius_rounding_m = 10.0;

    const auto below_half = detail::apply_horizontal_output_options(
        {124.999, 0.0, 0.0, 1.0}, options);
    const auto exact_half = detail::apply_horizontal_output_options(
        {125.0, 0.0, 0.0, 1.0}, options);
    const auto above_half = detail::apply_horizontal_output_options(
        {125.001, 0.0, 0.0, 1.0}, options);

    AR_EXPECT_NEAR(below_half.radius_m, 120.0, 1.0e-12);
    AR_EXPECT_NEAR(exact_half.radius_m, 120.0, 1.0e-12);
    AR_EXPECT_NEAR(above_half.radius_m, 130.0, 1.0e-12);
}

AR_TEST(horizontal_output_options_equalize_before_rounding) {
    ReconstructionOptions equal_only;
    equal_only.equalize_horizontal_spiral_lengths = true;
    const auto equalized = detail::apply_horizontal_output_options(
        {500.0, 14.0, 18.0, 7.0}, equal_only);
    AR_EXPECT_NEAR(equalized.entry_spiral_length_m, 16.0, 0.0);
    AR_EXPECT_NEAR(equalized.exit_spiral_length_m, 16.0, 0.0);
    AR_EXPECT_NEAR(equalized.mean_squared_offset_m2, 7.0, 0.0);

    equal_only.horizontal_spiral_length_rounding_m = 10.0;
    const auto equalized_then_rounded = detail::apply_horizontal_output_options(
        {500.0, 14.0, 18.0, 7.0}, equal_only);
    AR_EXPECT_NEAR(equalized_then_rounded.entry_spiral_length_m, 20.0, 0.0);
    AR_EXPECT_NEAR(equalized_then_rounded.exit_spiral_length_m, 20.0, 0.0);
}

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

AR_TEST(horizontal_classification_preserves_a_curve_with_an_exact_axis_tangent) {
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
    // The exact horizontal entry tangent is valid. It used to be treated as
    // degenerate, causing this genuine curve to be merged into a straight.
    AR_EXPECT(analysis.straight_segments.size() == 2);
    AR_EXPECT(analysis.straight_segments.front().first_index == 0);
    AR_EXPECT(analysis.straight_segments.back().last_index + 1 == samples.size());
    const auto actual = detail::HorizontalReconstructor({}).reconstruct(samples);
    AR_EXPECT(actual.size() == controls.size());
    const detail::HorizontalAlignment rebuilt(actual, 0.0);
    for (const auto& sample : samples) {
        AR_EXPECT(rebuilt.project(sample).distance_m < 0.01);
    }
}

AR_TEST(field_horizontal_reconstruction_recovers_all_mdb_control_points) {
    const auto directory = test_support::fixture_directory();
    const auto points = test_support::read_point_fixture(
        directory / "k1413_k1462_points.csv");
    const auto expected_rows = test_support::read_horizontal_fixture(
        directory / "k1413_k1462_horizontal.csv");
    // This fixture's expected values were verified while 0.00008 was the
    // public default; keep that historical parameter explicit now that the
    // public default is 0.001.
    ReconstructionOptions options;
    options.curvature_threshold_inverse_m = 0.00008;
    const detail::HorizontalReconstructor reconstructor(options);
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
