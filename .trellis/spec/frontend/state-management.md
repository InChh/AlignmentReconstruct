# State Management

## Applicable Native Pattern

There is no frontend/global/server state library. AlignmentReconstruct is value-in/result-out and owns no application session, singleton store, cache, database connection, or persisted configuration.

Public state is explicit in `include/alignment_reconstruct/types.h`:

```cpp
ReconstructionOptions options;
const ReconstructionResult reconstructed = reconstruct(points, options);

AlignmentDefinition alignment;
alignment.horizontal = reconstructed.horizontal;
alignment.vertical = reconstructed.vertical;
const AdjustmentCalculationResult adjusted =
    calculate_adjustments(measured_points, alignment);
```

This pattern is also exercised by `samples/main.cpp` and round-trip tests under `tests/`.

## Rules

- Pass configuration through `ReconstructionOptions` or explicit public arguments; do not add hidden global legacy parameters.
- Treat returned results as caller-owned values. The library does not retain caller inputs between calls.
- Preserve the single station source: horizontal projection produces `station_m`, and vertical evaluation consumes that exact value.
- Keep UI selection, URL state, file state, database state, progress indicators, and caching in the consuming application.
- Do not add a global store, service locator, mutable singleton, or framework state dependency to `src/`.

There is no criterion for promoting local state to frontend global state in this repository because no frontend exists.
