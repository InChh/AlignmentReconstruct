# Native Library Development Guidelines

The generated `backend` label maps to this repository's C++ production-library layer. AlignmentReconstruct is not a web backend: it is a Visual Studio 2026, C++17, x64 DLL with three numeric public entry points and no server, database, or logging runtime.

Read [AGENTS.md](../../../AGENTS.md) and [README.md](../../../README.md) before these focused guides. Product and architecture authority is recorded in `docs/aegis/baseline/2026-07-13-revised-baseline.md`, the ADRs under `docs/aegis/adr/`, and the public headers under `include/alignment_reconstruct/`.

## Guide Index

| Guide | Project-specific purpose | Applicability |
| --- | --- | --- |
| [Directory Structure](./directory-structure.md) | Production owners, dependency direction, and file placement | Active |
| [Database Guidelines](./database-guidelines.md) | Database exclusion and private reference-data boundary | Guardrail only |
| [Error Handling](./error-handling.md) | Result, status, diagnostic, validation, and exception-boundary conventions | Active |
| [Logging Guidelines](./logging-guidelines.md) | No-production-logging boundary and caller responsibility | Guardrail only |
| [Quality Guidelines](./quality-guidelines.md) | C++17, testing, build, export, dependency, and review gates | Active |

All files in this directory describe the repository as it exists. They do not authorize public-contract, ownership, dependency, or scope changes.
