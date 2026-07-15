#include "fixture_reader.h"
#include "test_framework.h"

#include <cmath>

AR_TEST(field_fixtures_have_strict_expected_shapes_and_units) {
    const auto directory = test_support::fixture_directory();
    const auto points = test_support::read_point_fixture(
        directory / "k1413_k1462_points.csv");
    const auto horizontal = test_support::read_horizontal_fixture(
        directory / "k1413_k1462_horizontal.csv");
    const auto offsets = test_support::read_offset_fixture(
        directory / "k1413_k1462_offsets.csv");

    AR_EXPECT(points.size() == 1569);
    AR_EXPECT(horizontal.size() == 22);
    AR_EXPECT(offsets.size() == 1569);

    for (std::size_t index = 0; index < points.size(); ++index) {
        AR_EXPECT(offsets[index].point_index == index);
        AR_EXPECT_NEAR(offsets[index].point.x, points[index].x, 0.0);
        AR_EXPECT_NEAR(offsets[index].point.y, points[index].y, 0.0);
        AR_EXPECT_NEAR(
            offsets[index].horizontal_offset_mm,
            offsets[index].horizontal_offset_m * 1000.0,
            1.0e-9);
    }

    AR_EXPECT(horizontal.front().control_point.kind ==
              alignment_reconstruct::HorizontalControlPointKind::start);
    AR_EXPECT(horizontal.back().control_point.kind ==
              alignment_reconstruct::HorizontalControlPointKind::end);
}
