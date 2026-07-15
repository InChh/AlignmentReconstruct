#include "test_framework.h"

#include "legacy_reference/legacy_vertical_geometry.h"
#include "vertical/vertical_alignment.h"

#include <stdexcept>
#include <vector>

using alignment_reconstruct::VerticalControlPoint;
using alignment_reconstruct::detail::VerticalAlignment;

AR_TEST(vertical_alignment_evaluates_tangents_and_physical_circular_curves) {
    const std::vector<VerticalControlPoint> controls{
        {0.0, 100.0, 0.0},
        {1000.0, 110.0, 12000.0},
        {2200.0, 104.0, 9000.0},
        {3200.0, 112.0, 0.0}
    };
    const VerticalAlignment alignment(controls);
    for (const double station : {0.0, 250.0, 850.0, 1000.0, 1150.0,
                                 1900.0, 2200.0, 2450.0, 3000.0, 3200.0}) {
        const auto actual = alignment.evaluate(station);
        const double expected = legacy_reference::vertical_elevation(controls, station);
        AR_EXPECT_NEAR(actual.elevation_m, expected, 1.0e-9);
    }
    AR_EXPECT(alignment.evaluate(1000.0).on_vertical_curve);
    AR_EXPECT(alignment.evaluate(2200.0).on_vertical_curve);
}

AR_TEST(vertical_alignment_treats_zero_radius_as_a_grade_break) {
    const std::vector<VerticalControlPoint> controls{
        {100.0, 10.0, 0.0},
        {200.0, 11.0, 0.0},
        {300.0, 9.0, 0.0}
    };
    const VerticalAlignment alignment(controls);
    AR_EXPECT_NEAR(alignment.evaluate(150.0).elevation_m, 10.5, 1.0e-12);
    AR_EXPECT_NEAR(alignment.evaluate(200.0).elevation_m, 11.0, 1.0e-12);
    AR_EXPECT_NEAR(alignment.evaluate(250.0).elevation_m, 10.0, 1.0e-12);
    AR_EXPECT(!alignment.evaluate(200.0).on_vertical_curve);
}

AR_TEST(vertical_alignment_rejects_non_monotonic_and_overlapping_geometry) {
    bool non_monotonic_rejected = false;
    try {
        const VerticalAlignment invalid({
            {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}
        });
    } catch (const std::invalid_argument&) {
        non_monotonic_rejected = true;
    }
    AR_EXPECT(non_monotonic_rejected);

    bool overlap_rejected = false;
    try {
        const VerticalAlignment invalid({
            {0.0, 0.0, 0.0},
            {100.0, 10.0, 10000.0},
            {200.0, 0.0, 10000.0},
            {300.0, 10.0, 0.0}
        });
    } catch (const std::invalid_argument&) {
        overlap_rejected = true;
    }
    AR_EXPECT(overlap_rejected);
}

AR_TEST(vertical_alignment_rejects_legacy_encoded_radius_values) {
    bool rejected = false;
    try {
        const VerticalAlignment invalid({
            {0.0, 0.0, 0.0}, {1000.0, 10.0, 50.0}, {2000.0, 0.0, 0.0}
        });
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    AR_EXPECT(rejected);
}
