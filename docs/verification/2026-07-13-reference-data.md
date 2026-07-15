# Reference Data Verification

Date: `2026-07-13`

## Sources

- User-provided private field workbook and database.
- Exact paths, source names, raw rows, and data fingerprints are intentionally omitted from the public repository.

## Workbook Inspection

- Workbook sheets: `Sheet1`, `Sheet2`, `Sheet3`, `拨量`, `线路参数`, `计算结果`.
- `Sheet1`: 1569 ordered N/E point rows, no header, no z column.
- `拨量`: 1569 data rows plus header.
- `线路参数`: 22 control-point rows plus header.

## MDB Inspection

- Provider: `Microsoft.ACE.OLEDB.16.0`.
- `既有基线交点数据表`: 22 rows for `既有基线`.
- `全线坐标法拨量计算结果表`: 1569 rows for `既有基线`.
- MDB control-point lengths and radii are stored to millimetre precision; offset values are stored in millimetres.

## Generated Fixtures

The three field-reference CSV files are private local evidence and are excluded by `.gitignore`. The local snapshot contains 1569 point rows, 22 horizontal control rows, and 1569 offset rows. The workbook point coordinates and database offset-table coordinates agree exactly after decimal parsing (`max_abs_dx = 0`, `max_abs_dy = 0`).

## Coverage Boundary

This user-provided workbook contains only horizontal N/E coordinates. It proves horizontal reconstruction, station and offset behavior. It cannot prove vertical reconstruction or lift/drop; those obligations remain and use deterministic XYZ fixtures plus legacy-equation reference tests.
