#include "fixture_reader.h"
#include "test_framework.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace test_support {
namespace {

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::ifstream open_fixture(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("Cannot open fixture: " + path.string());
    }
    return stream;
}

void require_finite(double value, const std::filesystem::path& path, std::size_t line) {
    if (!std::isfinite(value)) {
        throw std::runtime_error(
            "Non-finite fixture value at " + path.string() + ':' + std::to_string(line));
    }
}

alignment_reconstruct::HorizontalControlPointKind parse_kind(const std::string& value) {
    using alignment_reconstruct::HorizontalControlPointKind;
    if (value == "start") {
        return HorizontalControlPointKind::start;
    }
    if (value == "intersection") {
        return HorizontalControlPointKind::intersection;
    }
    if (value == "end") {
        return HorizontalControlPointKind::end;
    }
    throw std::runtime_error("Unknown horizontal control point kind: " + value);
}

} // namespace

std::filesystem::path fixture_directory() {
    const std::vector<std::filesystem::path> candidates{
        std::filesystem::path(__FILE__).parent_path() / "fixtures",
        std::filesystem::current_path() / "tests" / "fixtures"
    };
    bool incomplete_fixture_set = false;
    for (const auto& candidate : candidates) {
        const bool has_points = std::filesystem::exists(
            candidate / "k1413_k1462_points.csv");
        const bool has_horizontal = std::filesystem::exists(
            candidate / "k1413_k1462_horizontal.csv");
        const bool has_offsets = std::filesystem::exists(
            candidate / "k1413_k1462_offsets.csv");
        if (has_points && has_horizontal && has_offsets) {
            return std::filesystem::canonical(candidate);
        }
        incomplete_fixture_set = incomplete_fixture_set
            || has_points || has_horizontal || has_offsets;
    }
    if (incomplete_fixture_set) {
        throw std::runtime_error(
            "Private field fixture set is incomplete; see tests/fixtures/README.md");
    }
    skip("Private field fixtures are not included in the public repository");
}

std::vector<alignment_reconstruct::Point2D> read_point_fixture(
    const std::filesystem::path& path) {
    auto stream = open_fixture(path);
    std::string line;
    std::getline(stream, line);
    if (line != "point_index,x,y") {
        throw std::runtime_error("Unexpected point fixture header: " + line);
    }

    std::vector<alignment_reconstruct::Point2D> points;
    std::size_t line_number = 1;
    while (std::getline(stream, line)) {
        ++line_number;
        const auto fields = split_csv_line(line);
        if (fields.size() != 3 || std::stoull(fields[0]) != points.size()) {
            throw std::runtime_error("Malformed point fixture row " + std::to_string(line_number));
        }
        alignment_reconstruct::Point2D point{std::stod(fields[1]), std::stod(fields[2])};
        require_finite(point.x, path, line_number);
        require_finite(point.y, path, line_number);
        points.push_back(point);
    }
    return points;
}

std::vector<HorizontalFixtureRow> read_horizontal_fixture(
    const std::filesystem::path& path) {
    auto stream = open_fixture(path);
    std::string line;
    std::getline(stream, line);
    if (line != "control_index,name,kind,x,y,radius_m,entry_spiral_length_m,exit_spiral_length_m") {
        throw std::runtime_error("Unexpected horizontal fixture header: " + line);
    }

    std::vector<HorizontalFixtureRow> rows;
    std::size_t line_number = 1;
    while (std::getline(stream, line)) {
        ++line_number;
        const auto fields = split_csv_line(line);
        if (fields.size() != 8 || std::stoull(fields[0]) != rows.size()) {
            throw std::runtime_error("Malformed horizontal fixture row " + std::to_string(line_number));
        }
        HorizontalFixtureRow row;
        row.control_index = rows.size();
        row.name = fields[1];
        row.control_point.kind = parse_kind(fields[2]);
        row.control_point.position = {std::stod(fields[3]), std::stod(fields[4])};
        row.control_point.radius_m = std::stod(fields[5]);
        row.control_point.entry_spiral_length_m = std::stod(fields[6]);
        row.control_point.exit_spiral_length_m = std::stod(fields[7]);
        require_finite(row.control_point.position.x, path, line_number);
        require_finite(row.control_point.position.y, path, line_number);
        require_finite(row.control_point.radius_m, path, line_number);
        require_finite(row.control_point.entry_spiral_length_m, path, line_number);
        require_finite(row.control_point.exit_spiral_length_m, path, line_number);
        rows.push_back(row);
    }
    return rows;
}

std::vector<OffsetFixtureRow> read_offset_fixture(const std::filesystem::path& path) {
    auto stream = open_fixture(path);
    std::string line;
    std::getline(stream, line);
    if (line != "point_index,x,y,station_m,property,horizontal_offset_mm,horizontal_offset_m") {
        throw std::runtime_error("Unexpected offset fixture header: " + line);
    }

    std::vector<OffsetFixtureRow> rows;
    std::size_t line_number = 1;
    while (std::getline(stream, line)) {
        ++line_number;
        const auto fields = split_csv_line(line);
        if (fields.size() != 7 || std::stoull(fields[0]) != rows.size()) {
            throw std::runtime_error("Malformed offset fixture row " + std::to_string(line_number));
        }
        OffsetFixtureRow row;
        row.point_index = rows.size();
        row.point = {std::stod(fields[1]), std::stod(fields[2])};
        row.station_m = std::stod(fields[3]);
        row.property = std::stoi(fields[4]);
        row.horizontal_offset_mm = std::stod(fields[5]);
        row.horizontal_offset_m = std::stod(fields[6]);
        require_finite(row.point.x, path, line_number);
        require_finite(row.point.y, path, line_number);
        require_finite(row.station_m, path, line_number);
        require_finite(row.horizontal_offset_mm, path, line_number);
        require_finite(row.horizontal_offset_m, path, line_number);
        rows.push_back(row);
    }
    return rows;
}

} // namespace test_support
