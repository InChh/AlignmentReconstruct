#include "test_framework.h"

#include "alignment_reconstruct/adjustment.h"
#include "alignment_reconstruct/reconstruction.h"

#include <type_traits>

using namespace alignment_reconstruct;

static_assert(std::is_standard_layout<Point2D>::value, "Point2D must be standard-layout");
static_assert(std::is_standard_layout<Point3D>::value, "Point3D must be standard-layout");
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

AR_TEST(result_status_is_the_only_success_owner) {
    ReconstructionResult reconstruction;
    AdjustmentCalculationResult adjustments;
    AR_EXPECT(reconstruction.ok());
    AR_EXPECT(adjustments.ok());

    reconstruction.status = StatusCode::invalid_input;
    adjustments.status = StatusCode::infeasible_geometry;
    AR_EXPECT(!reconstruction.ok());
    AR_EXPECT(!adjustments.ok());
}

AR_TEST(default_options_preserve_confirmed_horizontal_thresholds) {
    const ReconstructionOptions options;
    AR_EXPECT_NEAR(options.start_station_m, 0.0, 0.0);
    AR_EXPECT_NEAR(options.curvature_window_m, 15.0, 0.0);
    AR_EXPECT_NEAR(options.curvature_threshold_inverse_m, 0.00008, 0.0);
    AR_EXPECT(options.minimum_curve_sample_count == 4);
    AR_EXPECT(options.maximum_spiral_classification_iterations == 100);
}
