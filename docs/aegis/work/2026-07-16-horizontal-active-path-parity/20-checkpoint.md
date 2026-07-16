# 平面重构活跃旧路径毫米级迁移 - Checkpoint

## Initial Checkpoint (historical RED)

- Task ID: 2026-07-16-horizontal-active-path-parity
- Current todo: 锁定活跃旧调用链并建立 0.001 m 严格失败测试
- Active slice: 平面分类和优化链的逐函数差异映射
- Blocked on: none
- Next step: 收紧现场重构测试并运行，保存 RED 指标

## Initial DriftCheckDraft (historical RED)

- Scope status: inside HorizontalReconstructor Task 7 boundary
- Compatibility status: public API and pure C++ boundary unchanged; strict numerical parity not yet achieved
- Retirement status: current heuristic classifier and Nelder-Mead remain active pending faithful replacement
- New risk signals:
- legacy active branch behavior includes three-state sample properties and exact Powell/Brent termination
- Advisory decision: continue

## Checkpoint Update

- Current todo: 迁移活跃旧直曲稳定化与 IDSpiral 三态边界
- Active slice: 逐行迁移 IDdentifyLineOrCurve 和 IDSpiral 的可达分支
- Completed todos:
- 锁定 `OnBnClickedButtonZdfd` → `CCeDianNiHe` 直曲分段链，以及 `OnBnClickedButtonNhht` → `FormLineSeg` → `IDSpiral` → `SwingAlgorithm` → `AllQXNH2` 活跃主链
- 建立逐类型 0.001 m 严格现场 RED，确认当前最大误差
- Evidence refs:
- strict-field-red
- Blocked on: none
- Next step: 建立旧分类阶段与新分类阶段的索引级诊断，并按活跃代码替换简化分类

## Final Checkpoint

- Current todo: complete
- Active slice: none
- Completed todos:
  - 以 `OnBnClickedButtonZdfd` 和 `OnBnClickedButtonNhht` 锁定当前活跃入口，排除 `_SW`、`New`、`opti`、单交点摆动和注释实现。
  - 迁移直曲稳定化、反向曲线处理、三态 `IDSpiral`、`SwingAlgorithm`、`AllQXNH2` 圆/交点/初始缓长和 Powell/Brent 目标链。
  - 删除简化小岛合并、曲率排名选圆点、局部目标样本和 Nelder-Mead 生产路径。
  - 修正 `YNH2` 圆心均值的浮点运算顺序，严格现场四类误差全部进入 `0.001 m`。
  - Debug/Release Rebuild、30 项测试、样例、拨量、纵面和生产边界扫描通过。
- Evidence refs: `strict-field-red`、`strict-field-green`、`debug-release-regression`、`production-boundary-scan`
- Blocked on: none
- Next step: none for this task boundary

## Final Drift Check

- Scope status: inside `HorizontalReconstructor` Task 7 boundary
- Compatibility status: public entrypoints、单位、坐标语义和纯 C++17 边界不变；严格现场数值目标已达到
- Retirement status: 简化 classifier 和 Nelder-Mead 主路径已删除；生产扫描无残留符号
- Residual scope: `AllQXNH2` 中少于 3 个圆样本时的 `BCTTP`/`BCTPP` 稀疏分支不在本次现场 22 点验收证据内，不据此宣称该边界已完成数值对照
- Decision: complete for the framed task
