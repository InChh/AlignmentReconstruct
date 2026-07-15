# RailAlignmentCore Initial Baseline（已被取代）

Date: `2026-07-11`
Status: `superseded on 2026-07-13`

> 本快照保留用于记录最初方案。项目已更名为 `AlignmentReconstruct`，且第二项能力已扩展为联合计算拨量和抬落道量。当前基线见 `2026-07-13-revised-baseline.md`。

## 1. Purpose

This baseline fixes the approved product scope and architecture boundary before the new static library is implemented. Later reviews use it to distinguish implementation drift from a requirement or design change.

## 2. Workspace Structure

The target is a new sibling project at `D:\Programming\C++\Projects\既有线\RailAlignmentCore`. At baseline time only Aegis design records exist. The planned solution contains one C++17 static library and one x64 console test project.

## 3. Current Authority Surfaces

- User-approved requirements and design in the active Codex task.
- Legacy behavior evidence in `D:\Programming\C++\Projects\既有线\主控`.
- `docs/aegis/specs/2026-07-11-rail-alignment-core-design.md` for the approved target design.
- No ADR or public API header exists yet.

## 4. Product / Requirement Baseline

### 4.1 Current Truth

The library must expose two independent capabilities:

1. Reconstruct horizontal and vertical railway alignment from ordered XYZ survey points. Horizontal output contains intersection coordinates, radius, entry transition length, and exit transition length. Vertical output contains grade-change mileage and elevation.
2. Given survey points and a horizontal alignment defined by intersection coordinates, radius, entry transition length, and exit transition length, calculate a signed horizontal offset for every input point.

The project must use the latest installed Visual Studio 2026 toolchain and C++17, produce a static library, and contain no ObjectARX or MFC dependency. Samples, focused tests, legacy-parity evidence, and fresh builds are required.

Input survey points are ordered from alignment start to alignment end. Coordinates, elevations, mileage, radii, transition lengths, and offsets use metres internally and in the primary API. Offset millimetres are a derived convenience conversion.

### 4.2 Non-negotiables

1. No ARX, MFC, CString, AcGe, AutoCAD, MDB, Excel, or UI type may appear in the library project or public headers.
2. Horizontal and vertical reconstruction must both be real implementations; neither may be replaced with a placeholder, polyline-only approximation, or hard-coded sample result.
3. Offset calculation must evaluate straight, transition, and circular elements and preserve legacy sign orientation: right is positive and left is negative when looking along increasing mileage.
4. Public results must carry actionable validation diagnostics rather than silently dropping bad input.
5. Legacy compatibility claims require direct numerical evidence, not build success alone.

### 4.3 Product Non-goals

- No CAD drawing, entity creation, command registration, dialogs, database persistence, Excel I/O, or railway-project management.
- No unordered-point route inference in the first version.
- No vertical offset or lift/drop calculation in the point-offset API.
- No integration back into the legacy ObjectARX project in this goal.

## 5. Architecture / Runtime Boundary Baseline

### 5.1 Current Truth

`RailAlignmentCore` is the canonical owner of all extracted pure geometry and reconstruction behavior. The legacy ObjectARX repository remains behavior evidence only. A test-only `legacy_reference` implementation may preserve narrowly scoped equations for parity comparison, but it cannot be linked by production code or become a second production owner.

Planned production owners:

- Horizontal reconstruction: curvature classification, segmentation, line and circle fitting, transition estimation, and parameter optimization.
- Vertical reconstruction: projection-derived mileage/elevation samples, second-difference classification, grade-line fitting, merging, and grade-change intersections.
- Alignment evaluation: intersection parameters to line elements, point-at-mileage evaluation, closest projection, and signed offset.

### 5.2 Architecture Non-negotiables

1. Public headers depend only on the C++17 standard library.
2. Production code never includes files from the legacy repository.
3. Test reference code is isolated under `tests/legacy_reference` and excluded from the library project.
4. Railway-standard parameters formerly read from globals are explicit reconstruction options.
5. Input/output units and sign conventions are documented at the public contract.

### 5.3 Architecture Non-goals

- Recreating the legacy `RHA` object model or its generic ten-column line-element storage.
- Preserving legacy UI state, global variables, binary layouts, or exception behavior.
- Maintaining VS2012/v110 ABI compatibility.

## 6. Ownership / Contract Snapshot

| Surface | Canonical owner |
| --- | --- |
| Public data types and results | `include/rail_alignment` |
| Horizontal reconstruction | `HorizontalReconstructor` implementation |
| Vertical reconstruction | `VerticalReconstructor` implementation |
| Element construction/evaluation and offsets | `HorizontalAlignment` implementation |
| Legacy numerical comparison | test-only `tests/legacy_reference` |
| Samples and executable verification | `RailAlignmentCore.Tests` |

## 7. Current State and Risks

The design is approved but implementation has not started. Principal risks are accidental CAD/MFC dependency leakage, hidden legacy global parameters, ambiguous endpoint semantics, failure to model transitions accurately, and parity tests that share too much implementation with production.

## 8. Alignment Use

Read the Product / Requirement Baseline before changing API behavior, supported inputs, units, output fields, or acceptance criteria. Read the Architecture / Runtime Boundary Baseline before changing owners, dependencies, project layout, test reference boundaries, or compatibility policy. Report `scope: both` when a change affects both public behavior and ownership.

## 9. Compatibility Boundary

Compatibility means numerical agreement with the relevant legacy algorithms for supported, valid inputs. It does not mean ABI compatibility, CAD object compatibility, persistence compatibility, or acceptance of undefined legacy edge cases.
