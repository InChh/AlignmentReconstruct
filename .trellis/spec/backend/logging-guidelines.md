# Logging Guidelines

## Applicability

The production DLL has no logging framework, log levels, structured log schema, global logger, telemetry, console ownership, or file logging. This is intentional: AlignmentReconstruct is a numeric library called by a host application. Its supported failure channel is the result/diagnostic contract in `include/alignment_reconstruct/types.h`.

## Production Rules

- Do not write to `std::cout`, `std::cerr`, debugger output, files, the Windows event log, or a third-party logging sink from `include/` or `src/`.
- Do not add a logging dependency or global callback as an implementation convenience. That would expand the production dependency and runtime contract.
- Return actionable `Diagnostic` values at public boundaries. Preserve the stable status/code/index fields described in [Error Handling](./error-handling.md).
- Do not include raw private fixture rows, local paths, secrets, or caller data dumps in diagnostic messages.

## Caller and Test Output

Console output in `samples/main.cpp` is consumer behavior, not a production logging precedent. The sample prints result diagnostics to `std::cerr` and successful values to `std::cout`. The console test runner under `tests/` may report pass/fail/skip state because it is a separate executable.

If a consuming CAD/UI/server application needs timestamps, levels, correlation IDs, persistence, or telemetry, it should translate the returned diagnostics into its own logging system after the DLL call. Do not add those host concerns to this repository's public API.

## Verification

When a change could introduce output side effects, search `include` and `src` for stream output, logging APIs, and new non-standard includes. Any such production hit requires an explicit contract/scope decision; there is no existing log-level or structured-logging convention to extend.
