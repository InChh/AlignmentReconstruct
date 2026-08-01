# Proof Bundle - 2026-08-01-dynamic-library

## Method Pack Boundary

This proof bundle is an advisory Aegis Method Pack record. It does not determine evidence sufficiency, produce authoritative `GateDecision`, or grant `completion authority`.

## Task Intent

- Requested outcome: 将静态库改为动态库，仅导出 fit_point_cloud、reconstruct 和 calculate_adjustments
- Scope: 公共导出可见性、MSBuild 产物、运行库、测试链接拓扑和文档

## Impact

- Compatibility boundary: Existing C++ signatures remain; callers use x64, matching MSVC ABI, and /MDd or /MD.
- Non-goals:
- C ABI
- algorithm changes
- static-library fallback

## Evidence Bundle Refs

- docs/aegis/work/2026-08-01-dynamic-library/evidence-bundle-draft-target-verification.json

## Drift Check

- Scope status: aligned
- Compatibility status: public signatures and metre contract preserved; DLL ABI requirement documented
- Retirement status: static-library configuration and src/types.cpp retired without fallback
- Advisory decision: continue
