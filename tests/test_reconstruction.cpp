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
#include <limits>
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

Point2D advance_point(
    const Point2D& point,
    double heading_rad,
    double distance_m) {
    return {
        point.x + distance_m * std::cos(heading_rad),
        point.y + distance_m * std::sin(heading_rad)
    };
}

std::vector<HorizontalControlPoint> make_five_curve_controls() {
    std::vector<HorizontalControlPoint> controls;
    controls.push_back({
        HorizontalControlPointKind::start,
        {0.0, 0.0}, 0.0, 0.0, 0.0
    });

    Point2D intersection{1500.0, 0.0};
    controls.push_back({
        HorizontalControlPointKind::intersection,
        intersection, 300.0, 80.0, 80.0
    });
    intersection = advance_point(intersection, 0.35, 1500.0);
    controls.push_back({
        HorizontalControlPointKind::intersection,
        intersection, 300.0, 80.0, 80.0
    });
    intersection = advance_point(intersection, 0.70, 1500.0);
    controls.push_back({
        HorizontalControlPointKind::intersection,
        intersection, 20000.0, 0.0, 0.0
    });
    intersection = advance_point(intersection, 0.71, 1500.0);
    controls.push_back({
        HorizontalControlPointKind::intersection,
        intersection, 300.0, 80.0, 80.0
    });
    intersection = advance_point(intersection, 1.06, 1500.0);
    controls.push_back({
        HorizontalControlPointKind::intersection,
        intersection, 300.0, 80.0, 80.0
    });
    controls.push_back({
        HorizontalControlPointKind::end,
        advance_point(intersection, 1.41, 1500.0), 0.0, 0.0, 0.0
    });
    return controls;
}

std::vector<Point3D> make_curved_reconstruction_samples() {
    constexpr double rotation_rad = 0.3;
    const std::vector<HorizontalControlPoint> controls{
        {HorizontalControlPointKind::start,
         rotate_point({0.0, 0.0}, rotation_rad), 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::intersection,
         rotate_point({1200.0, 0.0}, rotation_rad), 600.0, 80.0, 70.0},
        {HorizontalControlPointKind::end,
         rotate_point({1800.0, 900.0}, rotation_rad), 0.0, 0.0, 0.0}
    };
    const detail::HorizontalAlignment alignment(controls, 0.0);
    std::vector<Point3D> samples;
    for (double station_m = 0.0;
         station_m < alignment.end_station_m();
         station_m += 10.0) {
        const auto state = alignment.evaluate(station_m);
        samples.push_back({state.point.x, state.point.y, 100.0});
    }
    const auto end = alignment.evaluate(alignment.end_station_m());
    samples.push_back({end.point.x, end.point.y, 100.0});
    return samples;
}

ReconstructionOptions curvature_graph_test_options() {
    ReconstructionOptions options;
    options.vertical_second_difference_threshold_candidates = {1.0e-6};
    options.grade_rounding_per_mille = 0.0;
    return options;
}

constexpr double five_curve_sample_interval_m = 20.0;

std::vector<Point3D> make_five_curve_reconstruction_samples(
    const detail::HorizontalAlignment& source_alignment) {
    std::vector<Point3D> samples;
    for (double station_m = 0.0;
         station_m < source_alignment.end_station_m();
         station_m += five_curve_sample_interval_m) {
        const auto state = source_alignment.evaluate(station_m);
        const double lateral_noise_m = 0.1 * std::sin(0.013 * station_m);
        samples.push_back({
            state.point.x - lateral_noise_m * std::sin(state.heading_rad),
            state.point.y + lateral_noise_m * std::cos(state.heading_rad),
            100.0
        });
    }
    const auto end = source_alignment.evaluate(source_alignment.end_station_m());
    const double end_noise_m =
        0.1 * std::sin(0.013 * source_alignment.end_station_m());
    samples.push_back({
        end.point.x - end_noise_m * std::sin(end.heading_rad),
        end.point.y + end_noise_m * std::cos(end.heading_rad),
        100.0
    });
    return samples;
}

std::vector<double> make_five_curve_sample_stations(
    const detail::HorizontalAlignment& source_alignment) {
    std::vector<double> stations_m;
    for (double station_m = 0.0;
         station_m < source_alignment.end_station_m();
         station_m += five_curve_sample_interval_m) {
        stations_m.push_back(station_m);
    }
    stations_m.push_back(source_alignment.end_station_m());
    return stations_m;
}

std::size_t count_curve_runs(
    const std::vector<HorizontalCurvatureSample>& graph) {
    std::size_t count = 0;
    bool previous_was_curve = false;
    for (const HorizontalCurvatureSample& sample : graph) {
        const bool is_curve = sample.property == HorizontalSampleProperty::curve;
        if (is_curve && !previous_was_curve) {
            ++count;
        }
        previous_was_curve = is_curve;
    }
    return count;
}

std::size_t count_intersections(
    const std::vector<HorizontalControlPoint>& controls) {
    return static_cast<std::size_t>(std::count_if(
        controls.begin(), controls.end(),
        [](const HorizontalControlPoint& control) {
            return control.kind == HorizontalControlPointKind::intersection;
        }));
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

AR_TEST(public_reconstruction_returns_and_reuses_the_line_curve_curvature_graph) {
    const std::vector<Point3D> samples = make_curved_reconstruction_samples();
    const ReconstructionOptions options = curvature_graph_test_options();
    AR_EXPECT_NEAR(options.curvature_threshold_inverse_m, 0.001, 0.0);
    const ReconstructionResult automatic = reconstruct(samples, options);

    AR_EXPECT(automatic.ok());
    AR_EXPECT(automatic.curvature_graph.size() == samples.size());
    std::size_t straight_count = 0;
    std::size_t curve_count = 0;
    for (const HorizontalCurvatureSample& sample : automatic.curvature_graph) {
        AR_EXPECT(std::isfinite(sample.curvature_inverse_m));
        if (sample.property == HorizontalSampleProperty::straight) {
            ++straight_count;
        } else if (sample.property == HorizontalSampleProperty::curve) {
            ++curve_count;
        } else {
            AR_EXPECT(false);
        }
    }
    AR_EXPECT(straight_count > 0);
    AR_EXPECT(curve_count > 0);

    ReconstructionOptions threshold_bypass_options = options;
    threshold_bypass_options.curvature_threshold_inverse_m = 1.0;
    ReconstructionOptions window_bypass_options = options;
    window_bypass_options.curvature_window_m = 1.0e12;

    const ReconstructionResult threshold_control = reconstruct(
        samples, threshold_bypass_options);
    const ReconstructionResult window_control = reconstruct(
        samples, window_bypass_options);
    AR_EXPECT(threshold_control.ok());
    AR_EXPECT(window_control.ok());
    AR_EXPECT(threshold_control.horizontal.size() != automatic.horizontal.size());
    AR_EXPECT(window_control.horizontal.size() != automatic.horizontal.size());

    const std::vector<ReconstructionResult> reused_results{
        reconstruct(samples, threshold_bypass_options, automatic.curvature_graph),
        reconstruct(samples, window_bypass_options, automatic.curvature_graph)
    };
    for (const ReconstructionResult& reused : reused_results) {
        AR_EXPECT(reused.ok());
        AR_EXPECT(reused.curvature_graph.size() == automatic.curvature_graph.size());
        for (std::size_t index = 0; index < automatic.curvature_graph.size(); ++index) {
            AR_EXPECT_NEAR(
                reused.curvature_graph[index].curvature_inverse_m,
                automatic.curvature_graph[index].curvature_inverse_m,
                0.0);
            AR_EXPECT(
                reused.curvature_graph[index].property ==
                automatic.curvature_graph[index].property);
        }
        AR_EXPECT(reused.horizontal.size() == automatic.horizontal.size());
        for (std::size_t index = 0; index < automatic.horizontal.size(); ++index) {
            AR_EXPECT(reused.horizontal[index].kind == automatic.horizontal[index].kind);
            AR_EXPECT_NEAR(
                reused.horizontal[index].position.x,
                automatic.horizontal[index].position.x,
                1.0e-12);
            AR_EXPECT_NEAR(
                reused.horizontal[index].position.y,
                automatic.horizontal[index].position.y,
                1.0e-12);
            AR_EXPECT_NEAR(
                reused.horizontal[index].radius_m,
                automatic.horizontal[index].radius_m,
                1.0e-12);
            AR_EXPECT_NEAR(
                reused.horizontal[index].entry_spiral_length_m,
                automatic.horizontal[index].entry_spiral_length_m,
                1.0e-12);
            AR_EXPECT_NEAR(
                reused.horizontal[index].exit_spiral_length_m,
                automatic.horizontal[index].exit_spiral_length_m,
                1.0e-12);
        }
        AR_EXPECT(reused.projected_stations_m.size() == automatic.projected_stations_m.size());
        for (std::size_t index = 0; index < automatic.projected_stations_m.size(); ++index) {
            AR_EXPECT_NEAR(
                reused.projected_stations_m[index],
                automatic.projected_stations_m[index],
                1.0e-12);
        }
        AR_EXPECT(reused.vertical.size() == automatic.vertical.size());
        for (std::size_t index = 0; index < automatic.vertical.size(); ++index) {
            AR_EXPECT_NEAR(
                reused.vertical[index].station_m,
                automatic.vertical[index].station_m,
                1.0e-12);
            AR_EXPECT_NEAR(
                reused.vertical[index].elevation_m,
                automatic.vertical[index].elevation_m,
                1.0e-12);
            AR_EXPECT_NEAR(
                reused.vertical[index].vertical_curve_radius_m,
                automatic.vertical[index].vertical_curve_radius_m,
                1.0e-12);
        }
    }
}

AR_TEST(public_reconstruction_uses_an_edited_graph_to_add_a_fifth_curve) {
    const std::vector<HorizontalControlPoint> source_controls =
        make_five_curve_controls();
    const detail::HorizontalAlignment source_alignment(source_controls, 0.0);
    const ReconstructionOptions options = curvature_graph_test_options();
    AR_EXPECT_NEAR(options.curvature_threshold_inverse_m, 0.001, 0.0);

    double subtle_curve_start_station_m = 0.0;
    double subtle_curve_end_station_m = 0.0;
    double subtle_curve_curvature_inverse_m = 0.0;
    std::size_t subtle_curve_element_count = 0;
    for (const detail::HorizontalElement& element : source_alignment.elements()) {
        if (element.kind() != HorizontalElementKind::circular) {
            continue;
        }
        const double curvature_inverse_m =
            std::abs(element.start_curvature_inverse_m());
        if (curvature_inverse_m > 0.0 &&
            curvature_inverse_m < options.curvature_threshold_inverse_m) {
            subtle_curve_start_station_m = element.start_station_m();
            subtle_curve_end_station_m = element.end_station_m();
            subtle_curve_curvature_inverse_m = curvature_inverse_m;
            ++subtle_curve_element_count;
        }
    }
    AR_EXPECT(subtle_curve_element_count == 1);
    AR_EXPECT_NEAR(subtle_curve_curvature_inverse_m, 5.0e-5, 1.0e-12);

    const std::vector<double> sample_stations_m =
        make_five_curve_sample_stations(source_alignment);
    const std::vector<Point3D> samples =
        make_five_curve_reconstruction_samples(source_alignment);
    AR_EXPECT(sample_stations_m.size() == samples.size());

    const ReconstructionResult automatic = reconstruct(samples, options);
    AR_EXPECT(automatic.ok());
    AR_EXPECT(automatic.horizontal.size() == 6);
    AR_EXPECT(count_intersections(automatic.horizontal) == 4);
    AR_EXPECT(automatic.curvature_graph.size() == samples.size());
    AR_EXPECT(count_curve_runs(automatic.curvature_graph) == 4);

    std::vector<HorizontalCurvatureSample> edited_graph =
        automatic.curvature_graph;
    std::size_t changed_sample_count = 0;
    for (std::size_t index = 0; index < edited_graph.size(); ++index) {
        if (sample_stations_m[index] < subtle_curve_start_station_m ||
            sample_stations_m[index] > subtle_curve_end_station_m) {
            continue;
        }
        AR_EXPECT(
            std::abs(edited_graph[index].curvature_inverse_m) <
            options.curvature_threshold_inverse_m);
        AR_EXPECT(
            edited_graph[index].property ==
            HorizontalSampleProperty::straight);
        edited_graph[index].property = HorizontalSampleProperty::curve;
        ++changed_sample_count;
    }
    AR_EXPECT(changed_sample_count >= options.minimum_curve_sample_count);
    AR_EXPECT(count_curve_runs(edited_graph) == 5);

    const ReconstructionResult edited = reconstruct(samples, options, edited_graph);
    AR_EXPECT(edited.ok());
    AR_EXPECT(edited.horizontal.size() == 7);
    AR_EXPECT(count_intersections(edited.horizontal) == 5);
    AR_EXPECT(edited.curvature_graph.size() == edited_graph.size());
    for (std::size_t index = 0; index < edited_graph.size(); ++index) {
        AR_EXPECT_NEAR(
            edited.curvature_graph[index].curvature_inverse_m,
            edited_graph[index].curvature_inverse_m,
            0.0);
        AR_EXPECT(
            edited.curvature_graph[index].property ==
            edited_graph[index].property);
    }
}

AR_TEST(public_reconstruction_does_not_restore_the_fifth_curve_by_threshold_alone) {
    constexpr double subtle_curve_curvature_inverse_m = 5.0e-5;
    const detail::HorizontalAlignment source_alignment(
        make_five_curve_controls(), 0.0);
    const std::vector<double> sample_stations_m =
        make_five_curve_sample_stations(source_alignment);
    const std::vector<Point3D> samples =
        make_five_curve_reconstruction_samples(source_alignment);
    AR_EXPECT(sample_stations_m.size() == samples.size());

    double subtle_curve_start_station_m = 0.0;
    double subtle_curve_end_station_m = 0.0;
    std::size_t subtle_curve_element_count = 0;
    for (const detail::HorizontalElement& element : source_alignment.elements()) {
        if (element.kind() == HorizontalElementKind::circular &&
            std::abs(
                std::abs(element.start_curvature_inverse_m()) -
                subtle_curve_curvature_inverse_m) < 1.0e-12) {
            subtle_curve_start_station_m = element.start_station_m();
            subtle_curve_end_station_m = element.end_station_m();
            ++subtle_curve_element_count;
        }
    }
    AR_EXPECT(subtle_curve_element_count == 1);

    ReconstructionOptions options = curvature_graph_test_options();
    AR_EXPECT_NEAR(options.curvature_threshold_inverse_m, 0.001, 0.0);
    const ReconstructionResult curvature_probe = reconstruct(samples, options);
    AR_EXPECT(curvature_probe.ok());
    AR_EXPECT(curvature_probe.curvature_graph.size() == samples.size());
    AR_EXPECT(count_curve_runs(curvature_probe.curvature_graph) == 4);
    AR_EXPECT(count_intersections(curvature_probe.horizontal) == 4);

    options.curvature_threshold_inverse_m = 3.9e-5;
    AR_EXPECT(
        options.curvature_threshold_inverse_m <
        subtle_curve_curvature_inverse_m);
    const ReconstructionResult result = reconstruct(samples, options);

    AR_EXPECT(result.ok());
    AR_EXPECT(result.curvature_graph.size() == samples.size());
    AR_EXPECT(count_curve_runs(result.curvature_graph) == 4);
    AR_EXPECT(result.horizontal.size() == 6);
    AR_EXPECT(count_intersections(result.horizontal) == 4);

    std::size_t subtle_curve_sample_count = 0;
    double maximum_computed_curvature_inverse_m = 0.0;
    for (std::size_t index = 0; index < result.curvature_graph.size(); ++index) {
        const double station_m = sample_stations_m[index];
        if (station_m < subtle_curve_start_station_m ||
            station_m > subtle_curve_end_station_m) {
            continue;
        }
        maximum_computed_curvature_inverse_m = std::max(
            maximum_computed_curvature_inverse_m,
            std::abs(
                curvature_probe.curvature_graph[index].curvature_inverse_m));
        AR_EXPECT(
            curvature_probe.curvature_graph[index].property ==
            HorizontalSampleProperty::straight);
        AR_EXPECT(
            result.curvature_graph[index].property ==
            HorizontalSampleProperty::straight);
        ++subtle_curve_sample_count;
    }
    AR_EXPECT(subtle_curve_sample_count >= options.minimum_curve_sample_count);
    AR_EXPECT(
        maximum_computed_curvature_inverse_m >=
        options.curvature_threshold_inverse_m);
}

AR_TEST(public_reconstruction_rejects_invalid_curvature_graph_inputs) {
    std::vector<Point3D> points;
    for (int index = 0; index < 8; ++index) {
        points.push_back({10.0 * static_cast<double>(index), 0.0, 100.0});
    }

    const auto expect_error = [&](
        const std::vector<HorizontalCurvatureSample>& graph,
        DiagnosticCode code,
        std::size_t input_index) {
        const ReconstructionResult result = reconstruct(points, {}, graph);
        AR_EXPECT(result.status == StatusCode::invalid_input);
        AR_EXPECT(result.diagnostics.size() == 1);
        AR_EXPECT(result.diagnostics.front().code == code);
        AR_EXPECT(result.diagnostics.front().input_index == input_index);
        AR_EXPECT(result.horizontal.empty());
        AR_EXPECT(result.vertical.empty());
        AR_EXPECT(result.projected_stations_m.empty());
        AR_EXPECT(result.curvature_graph.empty());
    };

    expect_error(
        {{0.0, HorizontalSampleProperty::straight}},
        DiagnosticCode::curvature_graph_size_mismatch,
        no_input_index);

    std::vector<HorizontalCurvatureSample> graph(
        points.size(), {0.0, HorizontalSampleProperty::straight});
    graph[3].curvature_inverse_m =
        std::numeric_limits<double>::quiet_NaN();
    expect_error(graph, DiagnosticCode::non_finite_value, 3);

    graph[3].curvature_inverse_m = 0.0;
    graph[4].property = static_cast<HorizontalSampleProperty>(99);
    expect_error(
        graph,
        DiagnosticCode::invalid_horizontal_sample_property,
        4);

    graph[4].property = HorizontalSampleProperty::straight;
    graph[3].property = HorizontalSampleProperty::curve;
    expect_error(
        graph,
        DiagnosticCode::invalid_curvature_graph_shape,
        3);
}

AR_TEST(public_reconstruction_rejects_invalid_horizontal_output_options) {
    const auto expect_invalid = [](double radius_rounding_m, double spiral_rounding_m) {
        ReconstructionOptions options;
        options.horizontal_curve_radius_rounding_m = radius_rounding_m;
        options.horizontal_spiral_length_rounding_m = spiral_rounding_m;
        const auto result = reconstruct(
            {{0.0, 0.0, 100.0}, {1000.0, 0.0, 100.0}}, options);
        AR_EXPECT(result.status == StatusCode::invalid_input);
        AR_EXPECT(result.diagnostics.size() == 1);
        AR_EXPECT(result.diagnostics.front().code == DiagnosticCode::non_finite_value);
        AR_EXPECT(result.diagnostics.front().input_index == no_input_index);
        AR_EXPECT(result.horizontal.empty());
        AR_EXPECT(result.vertical.empty());
        AR_EXPECT(result.projected_stations_m.empty());
        AR_EXPECT(result.curvature_graph.empty());
    };

    expect_invalid(-1.0, 0.0);
    expect_invalid(0.0, -1.0);
    expect_invalid(std::numeric_limits<double>::quiet_NaN(), 0.0);
    expect_invalid(0.0, std::numeric_limits<double>::quiet_NaN());
    expect_invalid(std::numeric_limits<double>::infinity(), 0.0);
    expect_invalid(0.0, std::numeric_limits<double>::infinity());
}

AR_TEST(public_reconstruction_projects_with_the_processed_horizontal_scheme) {
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
    std::vector<Point3D> samples;
    for (double station = 0.0; station < seed_alignment.end_station_m(); station += 10.0) {
        const auto state = seed_alignment.evaluate(station);
        samples.push_back({state.point.x, state.point.y, 100.0});
    }
    const auto end = seed_alignment.evaluate(seed_alignment.end_station_m());
    samples.push_back({end.point.x, end.point.y, 100.0});

    ReconstructionOptions options;
    options.horizontal_curve_radius_rounding_m = 10.0;
    options.horizontal_spiral_length_rounding_m = 5.0;
    options.equalize_horizontal_spiral_lengths = true;
    options.vertical_second_difference_threshold_candidates = {1.0e-6};
    options.grade_rounding_per_mille = 0.0;
    const auto result = reconstruct(samples, options);

    AR_EXPECT(result.ok());
    AR_EXPECT(result.horizontal.size() == 3);
    AR_EXPECT_NEAR(
        result.horizontal[1].radius_m / 10.0,
        std::trunc(result.horizontal[1].radius_m / 10.0),
        1.0e-12);
    AR_EXPECT_NEAR(
        result.horizontal[1].entry_spiral_length_m,
        result.horizontal[1].exit_spiral_length_m,
        0.0);
    AR_EXPECT_NEAR(
        result.horizontal[1].entry_spiral_length_m / 5.0,
        std::trunc(result.horizontal[1].entry_spiral_length_m / 5.0),
        1.0e-12);

    const detail::HorizontalAlignment processed_alignment(
        result.horizontal, options.start_station_m);
    AR_EXPECT(result.projected_stations_m.size() == samples.size());
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto projection = processed_alignment.project(
            {samples[index].x, samples[index].y});
        AR_EXPECT(projection.valid);
        AR_EXPECT_NEAR(
            result.projected_stations_m[index], projection.station_m, 1.0e-9);
    }

    ReconstructionOptions infeasible_options = options;
    infeasible_options.horizontal_curve_radius_rounding_m = 2000.0;
    infeasible_options.horizontal_spiral_length_rounding_m = 0.0;
    infeasible_options.equalize_horizontal_spiral_lengths = false;
    const auto infeasible = reconstruct(samples, infeasible_options);
    AR_EXPECT(infeasible.status == StatusCode::infeasible_geometry);
    AR_EXPECT(infeasible.diagnostics.size() == 1);
    AR_EXPECT(infeasible.diagnostics.front().code == DiagnosticCode::degenerate_fit);
    AR_EXPECT(infeasible.horizontal.empty());
    AR_EXPECT(infeasible.vertical.empty());
    AR_EXPECT(infeasible.projected_stations_m.empty());
    AR_EXPECT(infeasible.curvature_graph.empty());
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
