# Frontend Directory Structure: Not Applicable

## Current Layout

There is no frontend directory or web package in this repository. The caller-visible surface is:

```text
include/alignment_reconstruct/   public C++ values and three exported functions
samples/main.cpp                 console consumer example using public headers
```

`src/` is private DLL implementation, not frontend application code. `samples/main.cpp` is a native console sample, not a page, component tree, or UI shell. The actual repository layout and placement rules are documented in [Backend Directory Structure](../backend/directory-structure.md).

## Boundary

- Do not create `frontend/`, `web/`, `components/`, `pages/`, `hooks/`, `styles/`, or browser asset directories inside AlignmentReconstruct for ordinary library work.
- Do not place CAD drawing, dialogs, command registration, visualization, or coordinate conversion into this DLL. `AGENTS.md`, `README.md`, and the revised baseline define those as caller concerns or non-goals.
- A requested UI is a scope/architecture decision for a separate consuming project. It must not change the library's numeric `x = N`, `y = E`, `z = H`, meter, station, or diagnostic contracts by implication.

No frontend module naming or feature-folder convention exists to copy.
