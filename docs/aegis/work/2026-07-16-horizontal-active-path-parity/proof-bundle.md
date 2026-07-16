# Proof Bundle - 2026-07-16-horizontal-active-path-parity

## Method Pack Boundary

This proof bundle is an advisory Aegis Method Pack record. It does not determine evidence sufficiency, produce authoritative `GateDecision`, or grant `completion authority`.

## Task Intent

- Requested outcome: 以参考项目当前活跃平面重构路径为唯一算法权威，继续迁移并把现场重构控制点误差压到毫米级
- Scope: AlignmentReconstruct 的 HorizontalReconstructor、曲率/分段、直线/圆拟合、缓和曲线识别、Powell/Brent 优化及其测试和治理文档；旧仓库只读

## Impact

- Compatibility boundary: 纯 C++17、x64、无 ARX/MFC；旧仓库不修改；公共函数和字段形状不变；平面默认迭代参数按旧活跃常量校正；私有现场数据不进入 Git
- Non-goals:
- 不参考注释代码作为权威，不修改纵面算法，不修改旧插件，不加入 MDB/Excel 生产依赖，不硬编码现场结果

## Evidence Bundle Refs

- docs/aegis/work/2026-07-16-horizontal-active-path-parity/evidence-bundle-draft-strict-field-green.json
- docs/aegis/work/2026-07-16-horizontal-active-path-parity/evidence-bundle-draft-strict-field-red.json

## Drift Check

- Scope status: inside HorizontalReconstructor Task 7 boundary
- Compatibility status: public entrypoints, units, coordinate semantics, and pure C++ boundary unchanged; strict field parity achieved
- Retirement status: heuristic classifier, curvature-ranked circle selection, local objective range, and Nelder-Mead production path removed
- Advisory decision: continue
