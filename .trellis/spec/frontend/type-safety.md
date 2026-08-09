# Type Safety: C++ Public Contract

## Applicability

TypeScript conventions are not applicable. The relevant type system is C++17, and the only caller-visible type owner is `include/alignment_reconstruct/`. Public headers use standard-library types and the `alignment_reconstruct` namespace; private implementation types stay in `alignment_reconstruct::detail`.

## Established Patterns

- Use value structs with default member initialization, as in `Point3D`, `HorizontalControlPoint`, and `VerticalControlPoint`.
- Use `enum class` for closed status, diagnostic, control-point, and element categories.
- Use semantic field suffixes for units: `_m`, `_inverse_m`, `_per_mille`; millimeters are derived helpers only.
- Use `std::size_t` for input indices and `no_input_index` when no individual input applies.
- Accept read-only collections and definitions by `const&`; return owner-specific result values.
- Keep inline, non-throwing convenience operations such as `ok()`, `horizontal_offset_mm()`, and `lift_drop_mm()` in public types so they do not expand DLL exports.
- Preserve standard-layout public geometry/control structs; `tests/test_contract.cpp` uses `static_assert` to fix this expectation.

Example from `include/alignment_reconstruct/types.h`:

```cpp
struct PointAdjustment {
    std::size_t point_index = 0;
    double station_m = 0.0;
    double horizontal_offset_m = 0.0;
    double lift_drop_m = 0.0;
};
```

## Runtime Validation and ABI

C++ static types do not replace boundary validation. The public wrappers reject non-finite values, invalid ranges, ordering violations, unsupported geometry, and insufficient point-cloud support with typed status/diagnostic results.

Public types contain `std::vector` and `std::string`, so the DLL exposes an MSVC C++ ABI rather than a stable C ABI. Callers must use compatible x64 MSVC toolsets and matching Debug `/MDd` or Release `/MD`, as documented in `README.md` and `docs/aegis/adr/ADR-0003-dynamic-library-export-boundary.md`.

Avoid untyped multi-column arrays, magic numeric encodings, C-style casts, widened public types for testing, or adding non-standard-library types to public headers.
