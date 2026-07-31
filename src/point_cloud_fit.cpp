#include "alignment_reconstruct/point_cloud_fit.h"

#include "point_cloud/point_cloud_fitter.h"

#include <cmath>
#include <exception>
#include <new>
#include <string>
#include <utility>

namespace alignment_reconstruct {
namespace {

PointCloudFitResult error_result(
    StatusCode status,
    DiagnosticCode code,
    std::size_t input_index,
    std::string message) {
    PointCloudFitResult result;
    result.status = status;
    result.diagnostics.push_back({
        DiagnosticSeverity::error, code, input_index, std::move(message)
    });
    return result;
}

} // namespace

PointCloudFitResult fit_point_cloud(
    const std::vector<Point3D>& unordered_points,
    double interval_m) {
    if (unordered_points.size() < 10) {
        return error_result(
            StatusCode::invalid_input,
            DiagnosticCode::too_few_points,
            no_input_index,
            "At least ten XYZ point-cloud points are required");
    }
    if (!std::isfinite(interval_m) || interval_m <= 0.0) {
        return error_result(
            StatusCode::invalid_input,
            DiagnosticCode::invalid_sample_interval,
            no_input_index,
            "Point-cloud sample interval must be finite and greater than zero");
    }
    for (std::size_t index = 0; index < unordered_points.size(); ++index) {
        const auto& point = unordered_points[index];
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            return error_result(
                StatusCode::invalid_input,
                DiagnosticCode::non_finite_value,
                index,
                "Point-cloud point contains a non-finite coordinate");
        }
    }

    try {
        PointCloudFitResult result;
        result.centerline_points =
            detail::PointCloudFitter().fit_and_sample(unordered_points, interval_m);
        return result;
    } catch (const detail::PointCloudFitError& error) {
        return error_result(
            error.status(),
            error.diagnostic_code(),
            no_input_index,
            error.what());
    } catch (const std::bad_alloc&) {
        return error_result(
            StatusCode::numerical_failure,
            DiagnosticCode::degenerate_fit,
            no_input_index,
            "Point-cloud fitting could not allocate the required memory");
    } catch (const std::exception& error) {
        return error_result(
            StatusCode::numerical_failure,
            DiagnosticCode::degenerate_fit,
            no_input_index,
            error.what());
    }
}

} // namespace alignment_reconstruct
