# Database Guidelines

## Applicability

Database access is deliberately not applicable to the production library. `README.md`, `AGENTS.md`, and `docs/aegis/baseline/2026-07-13-revised-baseline.md` define MDB, Excel, persistence, file parsing, and project management as caller responsibilities or non-goals. There is no ORM, schema, migration system, transaction convention, query layer, or production database naming convention in this repository.

## Production Boundary

- Do not add database, ODBC/OLE DB, COM, Office, MDB, Excel, or file-parsing dependencies under `include/`, `src/`, or `AlignmentReconstruct.vcxproj`.
- Public functions continue to accept typed in-memory values from `include/alignment_reconstruct/types.h`. A consuming application must translate persisted records into those values before calling the DLL.
- Do not create repository, data-access, migration, or query abstractions inside this numeric library. Such integration belongs in a caller-owned adapter outside AlignmentReconstruct.

The intended boundary is demonstrated by `samples/main.cpp`, which constructs `Point3D` and `AlignmentDefinition` values in memory and calls the public functions directly.

## Reference Data and Fixtures

Live database interaction is confined to offline verification tools such as `tools/inspect_reference_mdb.ps1` and `tools/export_reference_data.ps1`. They inspect user-provided evidence and produce test fixtures; they are not compiled by the DLL project. Tests and verification documents may consume or describe exported reference evidence, but they do not connect to a database at test runtime or enter the production DLL.

- `tests/fixtures/*.csv` is private field evidence excluded by `.gitignore`; never commit, push, publish, or embed it.
- The public repository keeps only `tests/fixtures/README.md`.
- Tests that require unavailable private fixtures must report an explicit skip through the existing test support; they must not masquerade as passing.
- Never write a local XLSX/MDB path into source, tests, documentation, or project files.
- Compatibility reports compare fields with stated maximum error, RMS where applicable, and tolerance. A successful build is not database/field-data parity evidence. See `docs/verification/2026-07-13-reference-data.md` and `docs/verification/2026-07-13-build-and-parity.md`.

## Review Check

For production-boundary changes, run the dependency scan from `AGENTS.md`. Any `XLDataBase`, `Excel`, COM, MDB, or other persistence hit in `include`, `src`, or `AlignmentReconstruct.vcxproj` is a design violation, not a convention to document around.
