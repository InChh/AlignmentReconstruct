# Hook Guidelines: Not Applicable

This repository has no frontend framework, custom hooks, reactive lifecycle, data-fetching client, server cache, or hook naming convention. The DLL performs synchronous in-memory numeric calls through three public C++ free functions.

Do not introduce React-style hooks, async data-fetching abstractions, or framework dependencies under `include/` or `src/`. File/database/network acquisition belongs to the caller, which converts data into `std::vector<Point3D>` and other public value types before calling the library.

`samples/main.cpp` is the reference integration sequence: prepare values, call a public function, check `ok()`, and pass successful output to the next public function. Any UI-specific wrapper around that sequence belongs to a separate consumer and follows that consumer's conventions.
