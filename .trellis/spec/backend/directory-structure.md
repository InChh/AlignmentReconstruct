# Directory Structure

## Scope

AlignmentReconstruct is one native library, not a routes/services web application. File placement follows public-contract and algorithm-owner boundaries. The authoritative owner map is in `AGENTS.md`, `docs/aegis/baseline/2026-07-13-revised-baseline.md`, and `docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md`.

## Repository Layout

```text
include/alignment_reconstruct/   public C++ contract and DLL visibility
src/                             production implementation
  alignment/                     adjustment orchestration
  geometry/                      typed numeric primitives, fits, and parity helpers
  horizontal/                    horizontal owners and reconstruction parity helpers
  point_cloud/                   point-cloud fitting owner
  vertical/                      vertical reconstruction/alignment owners
tests/                           deterministic, contract, parity, and fixture tests
  legacy_reference/              test-only legacy equations; never production
  fixtures/                      private ignored CSV evidence plus public README
samples/main.cpp                 public three-entry-point consumer example
tools/                            offline reference-data inspection/export only
docs/aegis/                       product baseline, design, ADRs, and work evidence
docs/verification/                recorded verification evidence
build/ and .scratch/              generated or temporary; never source authority
```

`AlignmentReconstruct.vcxproj` defines the DLL and public wrapper sources. `AlignmentReconstruct.InternalSources.props` is the shared list of private production sources used by the DLL and white-box tests. Tests and the sample have separate `.vcxproj` files and reference the DLL project.

## Ownership and Dependency Direction

Keep the existing direction:

```text
public free function
  -> narrow detail owner/orchestrator
  -> HorizontalAlignment / VerticalAlignment
  -> typed geometry elements
```

Concrete owners are:

- `src/horizontal/horizontal_reconstructor.*`: horizontal reconstruction.
- `src/vertical/vertical_reconstructor.*`: vertical reconstruction.
- `src/horizontal/horizontal_alignment.*`: final horizontal construction, evaluation, and nearest projection; the caller-visible station owner.
- `src/vertical/vertical_alignment.*`: vertical validation and elevation evaluation.
- `src/alignment/adjustment_calculator.*`: orchestration only.
- `src/point_cloud/point_cloud_fitter.*`: point-cloud binning, robust fitting, and XY-arc-length sampling.

Two legacy-named production-private seams are part of the current horizontal reconstruction path: `src/geometry/legacy_line_fit.*` supplies the active tangent-fit behavior, while `src/horizontal/legacy_horizontal_alignment.*` is used by the classifier and optimizer for the legacy-compatible curve objective. They are subordinate implementation details of `HorizontalReconstructor`, not `tests/legacy_reference`, public API, adjustment-calculation owners, or fallback routes. Keep their role narrow; do not export them or route `calculate_adjustments` through them.

The orchestration pattern is visible in `src/alignment/adjustment_calculator.cpp`:

```cpp
const auto projection = horizontal_.project({point.x, point.y});
const auto vertical = vertical_.evaluate(projection.station_m);
adjustment.station_m = projection.station_m;
adjustment.design_elevation_m = vertical.elevation_m;
```

Do not move projection, spiral, vertical-curve, or sign formulas into the orchestrator. Do not create a generic algorithm or `utils` dumping ground when an existing owner is responsible.

## Placement Rules

- Put stable caller-visible values and functions only in `include/alignment_reconstruct/`. Public headers include standard-library headers and other public project headers only.
- Put each of the three exported free-function definitions in the existing root wrappers: `src/point_cloud_fit.cpp`, `src/reconstruction.cpp`, and `src/adjustment.cpp`.
- Put private types and algorithms in the narrowest matching `src/<domain>/` directory and the `alignment_reconstruct::detail` namespace.
- Put black-box public-contract tests in `tests/test_contract.cpp` or the matching public feature test. Put owner-level tests beside the existing `test_horizontal_*`, `test_vertical_*`, `test_geometry.cpp`, or `test_point_cloud_fit.cpp` patterns.
- Keep minimal legacy comparisons in `tests/legacy_reference/`; production code must not include or link them.
- Keep caller-facing usage in `samples/main.cpp`; it may include only public headers.
- Keep MDB/XLSX inspection and fixture-export scripts in `tools/`; they are offline evidence tooling, not DLL dependencies.
- When adding, deleting, or moving C++ files, update the applicable `.vcxproj`, `.vcxproj.filters`, and `AlignmentReconstruct.InternalSources.props` lists together. Search all three registries before editing because production and test compilation intentionally share the internal list.

## Naming and File Conventions

- Use `namespace alignment_reconstruct`; use `alignment_reconstruct::detail` for non-public implementation.
- Follow existing lower snake-case file names such as `horizontal_alignment.cpp` and `point_cloud_fitter.h`.
- Use PascalCase for structs/classes/enums and snake_case for public free functions, methods, variables, and enum values.
- Encode physical semantics in names: `_m`, `_inverse_m`, `_rad`, and `_per_mille` are established suffixes in `include/alignment_reconstruct/types.h` and owner implementations.
- Use RAII, standard containers, value types, and explicit options. Do not import `CString`, `CArray`, `AcGe`, global legacy configuration, or ten-column raw array models.
- Maintained source, project, and Markdown files use CRLF as enforced by `.gitattributes`.

## Public Boundary

`include/alignment_reconstruct/api.h` is the only DLL-visibility owner. Only `fit_point_cloud`, `reconstruct`, and `calculate_adjustments` carry `ALIGNMENT_RECONSTRUCT_API`. Never export `detail::*` to make testing easier; `tests/AlignmentReconstruct.Tests.vcxproj` compiles shared internal sources for white-box coverage.
