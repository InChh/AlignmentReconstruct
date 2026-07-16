# 平面重构活跃旧路径毫米级迁移 - Intent

## TaskIntentDraft

- Requested outcome: 以参考项目当前活跃平面重构路径为唯一算法权威，继续迁移并把现场重构控制点误差压到毫米级
- Goal: 22 个控制点数量与类型完全一致，交点坐标、半径、进口缓长、出口缓长相对 MDB 最大误差分别不超过 0.001 m
- Success evidence:
- 严格现场 RED/GREEN 回归、逐阶段旧函数映射、Debug/Release 全量测试、拨量与纵面回归、生产依赖与私有数据边界扫描
- Stop condition: 全部严格证据满足为 done；证据不全为 needs-verification；活跃路径或必需数据无法读取为 blocked；需要扩大公共契约或修改旧仓库为 scope-exceeded
- Non-goals:
- 不参考注释代码作为权威，不修改纵面算法，不修改旧插件，不加入 MDB/Excel 生产依赖，不硬编码现场结果
- Scope: AlignmentReconstruct 的 HorizontalReconstructor、曲率/分段、直线/圆拟合、缓和曲线识别、Powell/Brent 优化及其测试和治理文档；旧仓库只读
- Change kinds:
- migration
- Risk hints:
- 旧文件含注释实现和死代码；必须从 `OnBnClickedButtonZdfd`、`OnBnClickedButtonNhht` 两个活跃按钮追踪到 `CCeDianNiHe` 当前被调函数，禁止依据同名 `_SW`、`New`、`opti`、注释分支或当前近似实现自行设计

## BaselineReadSetHint

- docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md
- docs/aegis/baseline/2026-07-13-revised-baseline.md
- docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md
- ../主控/QXBoLiangCalDlg.cpp:70-258 active UI chain
- ../主控/CeDianNiHe.cpp:848-4917 active callees

## BaselineUsageDraft

- Required baseline refs:
- docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md
- docs/aegis/baseline/2026-07-13-revised-baseline.md
- docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md
- ../主控/QXBoLiangCalDlg.cpp:70-258 active UI chain
- ../主控/CeDianNiHe.cpp:848-4917 active callees
- Acknowledged before plan:
- none
- Cited in plan:
- none
- Missing refs:
- docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md
- docs/aegis/baseline/2026-07-13-revised-baseline.md
- docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md
- ../主控/QXBoLiangCalDlg.cpp:70-258 active UI chain
- ../主控/CeDianNiHe.cpp:848-4917 active callees
- Advisory decision: needs-baseline-readback

## ImpactStatementDraft

- Compatibility boundary: 纯 C++17、x64、无 ARX/MFC；旧仓库不修改；公共函数和字段形状不变；平面默认迭代参数按旧活跃常量校正；私有现场数据不进入 Git
- Affected layers:
- 平面重构核心数值 owner 与现场兼容验证
- Owners:
- HorizontalReconstructor and its internal classifier/optimizer components
- Invariants:
- 旧活跃路径的调用顺序、分段边界、变量顺序 (l1,R,l2)、目标函数和停止条件必须保持；不得新增 fixture 专用路径或生产 fallback
- Non-goals:
- 不参考注释代码作为权威，不修改纵面算法，不修改旧插件，不加入 MDB/Excel 生产依赖，不硬编码现场结果

These records are Method Pack drafts / hints, not authoritative runtime decisions.

## Slice Card

- Goal: 以活跃旧路径替换平面重构中的近似分段和优化。
- Parent plan/spec: `docs/aegis/plans/2026-07-13-alignment-reconstruct-implementation.md` Task 7；`docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md` 第 3.2 节。
- Files: `src/horizontal/curvature_classifier.*`、`src/horizontal/horizontal_reconstructor.*`、`src/horizontal/alignment_optimizer.*`、必要的内部圆拟合 helper、`tests/test_horizontal_reconstruction.cpp`。
- Boundary: 公共函数和字段形状不变，旧仓库只读，私有夹具不入 Git，生产库仍仅依赖 C++17 标准库；允许把原近似算法默认值校正为旧活跃常量。
- Verification: 严格现场逐字段对照、合成平面回归、拨量/纵面全量回归、Debug/Release 构建、生产依赖和私有数据扫描。
- Stop: 22 个控制点数量/类型精确，四类最大误差均 `<= 0.001 m`；不达标时继续差分诊断，不放宽容差。

## Root-cause and repair boundary

- Symptom: 当前实现虽恢复 22 个点，但四类参数仍有分米到数米误差。
- Root cause: 新实现使用二值小岛合并、曲率排名选点、局部目标样本和 Nelder-Mead，而活跃旧路径使用 `CCeDianNiHe::IDSpiral` / `SwingAlgorithm` 三态边界、完整直线段范围和 Powell/Brent。
- Topology: `single-root-multi-symptom`；共同上游是 Task 7 曾用近似算法代替活跃路径。
- Falsifier: 如果只是浮点尾差，分段状态、样本范围和优化器应与旧代码一致；源码对照已证明三者都不一致。
- Change Necessity: 文档或调容差不能修复算法差异；最小代码边界是 `HorizontalReconstructor` 及现有内部 classifier/optimizer；Decision: `code-change`。
- Pre-edit owner fit: 修改属于既有平面重构 owner；通用圆拟合若被分段与最终优化共用，抽取为最窄内部 helper，不新增第二套算法。

## Anti-Entropy Declaration

- Deletion Class: `code-retirement`
- Old Path/Object: 新项目内的简化曲率小岛合并、`select_circular_samples`和 Nelder-Mead 主路径。
- New Canonical Owner: 同一 `HorizontalReconstructor` 内部的活跃旧路径等价实现。
- Expected Preserved Behavior: 公共类型、单位、诊断、调用方接口和纯 C++ 边界。
- Expected Retired Behavior: 不再用新项目自行设计的启发式决定分段或参数。
- External Boundary Touched: no
- Source-of-Truth Data Risk: none
- User Confirmation Required: no
- Retirement Decision: `delete-first`；不保留两套生产实现或 fallback。

## BaselineUsageDraft

- Required baseline refs:
- docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md
- docs/aegis/baseline/2026-07-13-revised-baseline.md
- docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md
- ../主控/QXBoLiangCalDlg.cpp:70-258 active UI chain
- ../主控/CeDianNiHe.cpp:848-4917 active callees
- Delivered context refs:
- none
- Acknowledged before plan:
- docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md
- docs/aegis/baseline/2026-07-13-revised-baseline.md
- docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md
- ../主控/QXBoLiangCalDlg.cpp:70-258 active UI chain
- ../主控/CeDianNiHe.cpp:848-4917 active callees
- Cited in plan:
- docs/aegis/plans/2026-07-13-alignment-reconstruct-implementation.md#Task-7
- Missing refs:
- none
- Advisory decision: continue
