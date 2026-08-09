# Frontend Quality Guidelines: Not Applicable

There is no frontend lint configuration, browser build, component test suite, DOM accessibility gate, CSS convention, or TypeScript type-check in this repository. Do not report such checks and do not add unused tooling for them.

## Applicable Guardrails

- Confirm ordinary library changes do not add JavaScript/TypeScript frontend packages, component frameworks, browser assets, or UI runtime dependencies.
- Keep `samples/main.cpp` as a public native consumer: it includes only public library headers and standard-library headers, checks result status, and owns console presentation.
- Keep CAD/UI display, accessibility, localization, file/database access, and host logging outside the DLL.
- Review all public-interface effects under the native [Backend Quality Guidelines](../backend/quality-guidelines.md), including C++17 `/W4`/warnings-as-errors, tests, dependency scan, export scan, and Debug/Release x64 gates.

## Scope Changes

If the user explicitly requests a UI, treat it as a new product/package decision. Establish the consuming project's location, framework, ownership, build, test, accessibility, and type rules before implementation. Do not infer those choices from this generated directory and do not retrofit them into the numeric DLL.
