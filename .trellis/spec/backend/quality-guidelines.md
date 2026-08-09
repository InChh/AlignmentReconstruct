# Quality Guidelines

## Compiler and Dependency Baseline

`AlignmentReconstruct.vcxproj`, the test project, and the sample project define the enforced baseline:

- Visual Studio 2026, platform toolset `v145`, Windows x64 only.
- C++17, conformance mode, SDL checks, `/W4`, and warnings as errors.
- Debug uses `/MDd`; Release uses `/MD`.
- Production public headers and DLL code depend only on the C++17 standard library.
- The DLL exports exactly `fit_point_cloud`, `reconstruct`, and `calculate_adjustments` through `include/alignment_reconstruct/api.h`.

Do not relax compiler warnings, change toolsets/configurations to make a failure disappear, or add external production libraries such as Eigen/PCL, AutoCAD/ObjectARX, MFC, COM, MDB, or Excel runtimes.

## Required Implementation Patterns

- Preserve `x = N`, `y = E`, `z = H`; meters are authoritative. Use explicit suffixes such as `_m`, `_inverse_m`, `_rad`, and `_per_mille`.
- Preserve the one-station invariant. `HorizontalAlignment::project` owns station; the same projected station drives `VerticalAlignment::evaluate`.
- Add behavior to the existing narrow owner. Do not duplicate projection, spiral, vertical-curve, reconstruction, sign, or sampling formulas.
- Use C++17 RAII, value types, standard containers, scoped enums, const references for read-only inputs, and explicit option objects rather than global state.
- Keep public headers minimal and stable. Public field, unit, sign, input-range, diagnostic, default-option, or export changes require baseline/design review, focused tests, and `README.md` synchronization.
- Preserve input order contracts: `reconstruct` requires ordered input; `fit_point_cloud` is row-order independent; neither may silently discard invalid points.
- Keep maintained text in CRLF. Do not edit generated `build/` or `.scratch/` artifacts.

The public type style is fixed in `include/alignment_reconstruct/types.h` and checked in `tests/test_contract.cpp`:

```cpp
struct VerticalControlPoint {
    double station_m = 0.0;
    double elevation_m = 0.0;
    double vertical_curve_radius_m = 0.0;
};

static_assert(std::is_standard_layout<VerticalControlPoint>::value,
              "VerticalControlPoint must be standard-layout");
```

## Forbidden Patterns

- A second production owner or fallback for an existing algorithm.
- Generic algorithm grab-bag files created only to share unrelated formulas.
- Exporting `detail::*`, test seams, or internal classes.
- Linking or including `tests/legacy_reference` from the production project.
- Copying legacy `CString`, `CArray`, `AcGe`, CAD/UI/database code, global configuration, or raw multi-column arrays.
- Silent sorting, point deletion, polyline fallback, geometry repair, or hard-coded expected results.
- Absolute local XLSX/MDB/CSV paths in source, tests, project files, README, or committed docs.
- Treating a historical verification report as proof that the current worktree builds or passes.

## Tests

Every behavior change needs at least one focused test that fails before the change and passes after it. Use the repository's `AR_TEST`, `AR_EXPECT`, and `AR_EXPECT_NEAR` framework from `tests/test_framework.h`; include explicit tolerances and enough values in failures to diagnose numeric drift.

Choose coverage proportional to the owner and contract risk. Established cases include straight/circular/unequal-spiral geometry, element joins and endpoints, right/left offset sign, convex/concave/zero-radius vertical behavior, shared station, reconstruction round trips, modified intersections, point-cloud row-order invariance and support, and stable invalid-input diagnostics.

Private `tests/fixtures/*.csv` evidence is ignored and must stay private. A missing optional fixture is an explicit skip, not a pass. Field-parity claims report per-field maximum error, RMS where meaningful, and tolerance.

## Verification Commands

Run commands from Git Bash at the repository root. For a production behavior or build-system change, the normal gate is:

```bash
MSYS2_ARG_CONV_EXCL='*' MSBuild.exe \
  AlignmentReconstruct.sln /m /t:Rebuild \
  /p:Configuration=Debug /p:Platform=x64 /v:minimal
./build/Debug/AlignmentReconstruct.Tests.exe
./build/Debug/AlignmentReconstruct.Sample.exe

MSYS2_ARG_CONV_EXCL='*' MSBuild.exe \
  AlignmentReconstruct.sln /m /t:Rebuild \
  /p:Configuration=Release /p:Platform=x64 /v:minimal
./build/Release/AlignmentReconstruct.Tests.exe
./build/Release/AlignmentReconstruct.Sample.exe
```

If `MSBuild.exe` is not on `PATH`, use the installed Visual Studio 2026 x64 MSBuild absolute path; do not downgrade the project. Also run the production-boundary scan from `AGENTS.md`, verify new/moved files in every applicable project/filter/shared list, and use `dumpbin /exports` when the export boundary could change. Tests and the sample must load `AlignmentReconstruct.dll`, and the production DLL must not link test objects.

Documentation-only changes require targeted content, link, and template-marker checks plus `git diff --check`; do not claim a build, test, lint, or type-check that was not run.

## Review Checklist

- Does the change preserve public units, coordinates, signs, station, diagnostics, and supported input boundaries?
- Is responsibility in the unique existing owner with the documented dependency direction?
- Are public/internal/test/reference boundaries unchanged or deliberately reviewed?
- Is the test focused, deterministic, tolerance-aware, and capable of catching the pre-change behavior?
- Are project registries, filters, README/contracts, and verification evidence updated where the change requires them?
- Were only checks actually executed reported, with skipped or unavailable gates stated as remaining risk?
