# Error Handling

## Public Result Contract

Expected input and geometry failures are returned as values, not thrown across the DLL boundary. `include/alignment_reconstruct/types.h` defines the shared model:

```cpp
enum class StatusCode {
    success,
    invalid_input,
    infeasible_geometry,
    numerical_failure
};

struct Diagnostic {
    DiagnosticSeverity severity;
    DiagnosticCode code;
    std::size_t input_index;
    std::string message;
};
```

`ReconstructionResult`, `PointCloudFitResult`, and `AdjustmentCalculationResult` each own a `status`, output values, `diagnostics`, and an inline `ok()` helper. As fixed by `tests/test_contract.cpp`, `status` is the only success owner; callers must not infer success from an empty diagnostic list or non-empty output.

## Boundary Validation and Exception Translation

Follow the existing public wrappers in `src/reconstruction.cpp`, `src/adjustment.cpp`, and `src/point_cloud_fit.cpp`:

1. Validate point counts, finite numbers, ranges, ordering, and contract invariants before invoking an algorithm owner.
2. Return `StatusCode::invalid_input` with a specific stable `DiagnosticCode` for caller-correctable contract errors.
3. Translate infeasible geometry to `StatusCode::infeasible_geometry` and unexpected numeric/runtime failures to `StatusCode::numerical_failure`.
4. Catch implementation exceptions at the public free-function boundary. Do not allow expected validation or geometry exceptions to escape the DLL.

The local boundary-helper pattern is:

```cpp
ResultType result;
result.status = status;
result.diagnostics.push_back({
    DiagnosticSeverity::error, code, input_index, std::move(message)
});
return result;
```

Specialized internal errors may preserve richer classification. For example, `PointCloudFitError` carries a `StatusCode` and `DiagnosticCode`, which `src/point_cloud_fit.cpp` copies into the public result.

## Diagnostic Rules

- Use an existing `DiagnosticCode` when its meaning matches. A new failure category that changes the public enum is a contract change and requires README synchronization and tests.
- Set `input_index` to the original input position when one point/control caused the error. Use `no_input_index` only for whole-request or geometry-wide failures.
- Keep messages readable and stable enough for diagnosis, but consumers should branch on status/code rather than parse English text.
- Preserve input order and identity. Never silently sort, drop, or repair input to avoid returning an error.
- Do not return partial output as success. `AlignmentAdjustmentCalculator::calculate` clears accumulated points before returning a point-specific failure; other entry points construct an error result without successful outputs.
- Use `DiagnosticSeverity::warning` only for a genuinely successful, explicitly supported degraded outcome. Existing normal failures use `error`; do not downgrade failures to keep `ok()` true.

## Consumer Pattern

Caller-side reporting belongs outside the DLL. `samples/main.cpp` follows the supported pattern:

```cpp
const ReconstructionResult result = reconstruct(points);
if (!result.ok()) {
    std::cerr << result.diagnostics.front().message << '\n';
    return 1;
}
```

## Testing Requirements

Every new or changed failure path needs a test that fixes all observable fields relevant to the contract: `StatusCode`, `DiagnosticCode`, `input_index`, and whether output is empty/preserved. `tests/test_adjustment.cpp`, `tests/test_contract.cpp`, `tests/test_reconstruction.cpp`, and `tests/test_point_cloud_fit.cpp` are the reference patterns.

Avoid generic `catch (...)`, assertion-only public validation, hard-coded success fallbacks, or using exceptions as normal caller-facing control flow.
