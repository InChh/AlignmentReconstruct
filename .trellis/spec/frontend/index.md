# Frontend Guidelines: Not Applicable

AlignmentReconstruct contains no frontend package, browser/runtime UI, components, pages, hooks, styling, assets, accessibility layer, TypeScript, or state-management library. It is a pure C++17 x64 numeric DLL. `README.md` explicitly assigns interface display, file access, persistence, and coordinate adaptation to the consuming application.

These generated frontend files are retained as explicit scope guardrails so future tasks do not invent framework conventions:

| Guide | What it records | Applicability |
| --- | --- | --- |
| [Directory Structure](./directory-structure.md) | Absence of a frontend tree and placement boundary | Guardrail only |
| [Component Guidelines](./component-guidelines.md) | Components/props/styling/accessibility are caller-owned | Not applicable |
| [Hook Guidelines](./hook-guidelines.md) | Hooks and data fetching are absent | Not applicable |
| [State Management](./state-management.md) | Value-in/result-out native state model | Guardrail only |
| [Type Safety](./type-safety.md) | Applicable C++ public type contract; no TypeScript | C++ mapping only |
| [Quality Guidelines](./quality-guidelines.md) | No-frontend checks and scope-change rule | Guardrail only |

Do not add a frontend framework to satisfy these template filenames. A real UI would be a separate consuming project with its own conventions and approval.
