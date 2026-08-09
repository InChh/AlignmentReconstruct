# Component Guidelines: Not Applicable

AlignmentReconstruct has no UI component system, props model, templates, styling solution, or accessibility implementation. No React, Vue, Angular, native widget, MFC dialog, or CAD entity belongs to the production DLL.

The closest caller example is `samples/main.cpp`: it constructs public C++ values, calls `fit_point_cloud`, `reconstruct`, and `calculate_adjustments`, checks `result.ok()`, and renders results in the caller. This demonstrates the boundary, not a component convention.

If a consuming application builds components around the DLL:

- keep component, formatting, accessibility, and interaction rules in that application's repository;
- translate UI/CAD coordinates at the caller boundary without changing `Point2D`/`Point3D` semantics;
- branch on `StatusCode` and `DiagnosticCode`, then present `Diagnostic::message` as appropriate for that host;
- keep persistence, file parsing, and display state outside this library.

Do not add component-oriented public types or export internal geometry classes to make a UI easier to write. Public API changes require their own contract review and are governed by `include/alignment_reconstruct/`, `README.md`, and the Aegis baseline.
