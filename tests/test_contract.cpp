#include "test_framework.h"

#include "alignment_reconstruct/adjustment.h"
#include "alignment_reconstruct/point_cloud_fit.h"
#include "alignment_reconstruct/reconstruction.h"

#include <type_traits>

using namespace alignment_reconstruct;

static_assert(std::is_standard_layout<Point2D>::value, "Point2D must be standard-layout");
static_assert(std::is_standard_layout<Point3D>::value, "Point3D must be standard-layout");
static_assert(std::is_standard_layout<HorizontalCurvatureSample>::value,
              "HorizontalCurvatureSample must be standard-layout");
static_assert(std::is_standard_layout<HorizontalControlPoint>::value,
              "HorizontalControlPoint must be standard-layout");
static_assert(std::is_standard_layout<VerticalControlPoint>::value,
              "VerticalControlPoint must be standard-layout");

AR_TEST(public_contract_uses_metres_with_explicit_millimetre_helpers) {
    PointAdjustment adjustment;
    adjustment.horizontal_offset_m = -0.0125;
    adjustment.lift_drop_m = 0.034;

    AR_EXPECT_NEAR(adjustment.horizontal_offset_mm(), -12.5, 1.0e-12);
    AR_EXPECT_NEAR(adjustment.lift_drop_mm(), 34.0, 1.0e-12);
}

AR_TEST(vertical_control_point_carries_station_elevation_and_radius) {
    VerticalControlPoint point;
    point.station_m = 1234.5;
    point.elevation_m = 56.789;
    point.vertical_curve_radius_m = 10000.0;

    AR_EXPECT_NEAR(point.station_m, 1234.5, 0.0);
    AR_EXPECT_NEAR(point.elevation_m, 56.789, 0.0);
    AR_EXPECT_NEAR(point.vertical_curve_radius_m, 10000.0, 0.0);
}

AR_TEST(horizontal_curvature_sample_uses_the_public_line_curve_values) {
    AR_EXPECT(static_cast<int>(HorizontalSampleProperty::straight) == 0);
    AR_EXPECT(static_cast<int>(HorizontalSampleProperty::curve) == 1);

    HorizontalCurvatureSample sample;
    sample.curvature_inverse_m = -0.00125;
    sample.property = HorizontalSampleProperty::curve;
    AR_EXPECT_NEAR(sample.curvature_inverse_m, -0.00125, 0.0);
    AR_EXPECT(sample.property == HorizontalSampleProperty::curve);
}

AR_TEST(result_status_is_the_only_success_owner) {
    ReconstructionResult reconstruction;
    PointCloudFitResult point_cloud_fit;
    AdjustmentCalculationResult adjustments;
    AR_EXPECT(reconstruction.ok());
    AR_EXPECT(point_cloud_fit.ok());
    AR_EXPECT(adjustments.ok());

    reconstruction.status = StatusCode::invalid_input;
    point_cloud_fit.status = StatusCode::numerical_failure;
    adjustments.status = StatusCode::infeasible_geometry;
    AR_EXPECT(!reconstruction.ok());
    AR_EXPECT(!point_cloud_fit.ok());
    AR_EXPECT(!adjustments.ok());
}

AR_TEST(default_options_preserve_confirmed_horizontal_thresholds) {
    const ReconstructionOptions options;
    AR_EXPECT_NEAR(options.start_station_m, 0.0, 0.0);
    AR_EXPECT_NEAR(options.curvature_window_m, 15.0, 0.0);
    AR_EXPECT_NEAR(options.curvature_threshold_inverse_m, 0.001, 0.0);
    AR_EXPECT(options.minimum_curve_sample_count == 3);
    AR_EXPECT(options.maximum_spiral_classification_iterations == 300);
    AR_EXPECT_NEAR(options.horizontal_optimization_tolerance, 1.0e-4, 0.0);
    AR_EXPECT(options.horizontal_optimization_max_iterations == 10);
    AR_EXPECT_NEAR(options.horizontal_curve_radius_rounding_m, 0.0, 0.0);
    AR_EXPECT_NEAR(options.horizontal_spiral_length_rounding_m, 0.0, 0.0);
    AR_EXPECT(!options.equalize_horizontal_spiral_lengths);
}

AR_TEST(default_options_preserve_the_rebuild_dialog_vertical_parameters) {
    const ReconstructionOptions options;
    const std::vector<double> expected{0.001};
    AR_EXPECT(
        options.vertical_second_difference_threshold_candidates == expected);
    AR_EXPECT_NEAR(options.minimum_grade_length_m, 350.0, 0.0);
    AR_EXPECT_NEAR(options.maximum_up_grade, 1000.0, 0.0);
    AR_EXPECT_NEAR(options.maximum_down_grade, 1000.0, 0.0);
    AR_EXPECT_NEAR(
        options.maximum_grade_difference_per_mille,
        1000.0,
        0.0);
    AR_EXPECT_NEAR(
        options.minimum_vertical_curve_grade_difference_per_mille,
        1000.0,
        0.0);
    AR_EXPECT_NEAR(options.minimum_vertical_curve_radius_m, 10000.0, 0.0);
    AR_EXPECT_NEAR(options.maximum_vertical_curve_radius_m, 30000.0, 0.0);
}

AR_TEST(public_reconstruction_rejects_a_vertical_radius_limit_above_the_active_cap) {
    ReconstructionOptions options;
    options.maximum_vertical_curve_radius_m = 30000.1;
    const auto result = reconstruct(
        {{0.0, 0.0, 0.0}, {1000.0, 0.0, 1.0}},
        options);
    AR_EXPECT(result.status == StatusCode::invalid_input);
}
