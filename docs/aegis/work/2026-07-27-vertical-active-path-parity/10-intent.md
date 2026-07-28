# 纵断面活跃路径迁移与MDB一致性 - Intent

## TaskIntentDraft

- Requested outcome: 使用全部1569个实测XYZ测点完成纵断面重构，并与MDB既有基线坡度表的变坡点里程、高程一致。
- Goal: 公开reconstruct接口必须保留全部输入点，成功输出纵断面控制点；按正确里程基准转换后逐点对齐MDB。
- Success evidence:
- VS2026 Debug和Release新鲜构建通过；全部1569点公开API成功；控制点数量与MDB一致；station/elevation逐点最大误差达到小数点后数值级；全部回归测试通过。
- Stop condition: done=全部证据满足；blocked=权限或必需数据不可用；needs-verification=实现存在但现场逐点证据不足；scope-exceeded=需要改变公共契约或修改旧ObjectARX生产代码。
- Non-goals:
- 不修改旧ObjectARX工程；不公开XLSX/MDB/CSV；不改变已验证平面算法。
- Scope: VerticalReconstructor、ProfileClassifier、公开reconstruct纵面编排、纵面测试与验证记录。
- Change kinds:
- migration
- Risk hints:
- 旧算法依赖活跃调用顺序、规范参数、断链里程和噪声分类；私有现场数据不能进入公开仓库。

## BaselineReadSetHint

- docs/aegis/baseline/2026-07-13-revised-baseline.md；docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md；用户实际运行的 `ZDMSJ::FitSlope` → `CRebuildDlg::OnBnClickedOk` → `CReBuildGMX::FitSlope` 纵面重构链及其输出坡度表

## BaselineUsageDraft

- Required baseline refs:
- docs/aegis/baseline/2026-07-13-revised-baseline.md；docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md；`ZDMSJ.cpp`、`RebuildDlg.cpp`、`ReBuildGMX.cpp`、`DrawSlope.cpp` 和 `BAS_CALC_FUN.cpp` 的当前未注释执行链
- Acknowledged before plan:
- 已读取并逐阶段核对
- Cited in plan:
- docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md 3.4
- Missing refs:
- none
- Advisory decision: baseline-acknowledged

## ImpactStatementDraft

- Compatibility boundary: 保持C++17纯标准库、公共API和物理竖曲线半径语义不变。
- Affected layers:
- vertical reconstruction implementation and verification
- Owners:
- VerticalReconstructor
- Invariants:
- 所有输入点参与；平面投影station是纵面唯一里程；不引入第二个生产实现。
- Non-goals:
- 不修改旧ObjectARX工程；不公开XLSX/MDB/CSV；不改变已验证平面算法。

These records are Method Pack drafts / hints, not authoritative runtime decisions.
