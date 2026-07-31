#pragma once

#include "alignment_reconstruct/types.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace alignment_reconstruct::detail {

class PointCloudFitError final : public std::runtime_error {
public:
    PointCloudFitError(
        StatusCode status,
        DiagnosticCode diagnostic_code,
        std::string message);

    StatusCode status() const noexcept;
    DiagnosticCode diagnostic_code() const noexcept;

private:
    StatusCode status_;
    DiagnosticCode diagnostic_code_;
};

class PointCloudFitter {
public:
    std::vector<Point3D> fit_and_sample(
        const std::vector<Point3D>& points,
        double interval_m) const;
};

} // namespace alignment_reconstruct::detail
