#pragma once

#include "alignment_reconstruct/types.h"

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace test_support {

struct HorizontalFixtureRow {
    std::size_t control_index = 0;
    std::string name;
    alignment_reconstruct::HorizontalControlPoint control_point;
};

struct OffsetFixtureRow {
    std::size_t point_index = 0;
    alignment_reconstruct::Point2D point;
    double station_m = 0.0;
    int property = 0;
    double horizontal_offset_mm = 0.0;
    double horizontal_offset_m = 0.0;
};

std::filesystem::path fixture_directory();

std::vector<alignment_reconstruct::Point2D> read_point_fixture(
    const std::filesystem::path& path);

std::vector<HorizontalFixtureRow> read_horizontal_fixture(
    const std::filesystem::path& path);

std::vector<OffsetFixtureRow> read_offset_fixture(
    const std::filesystem::path& path);

} // namespace test_support
