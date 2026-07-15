# AlignmentReconstruct 证据包

日期：`2026-07-13`  
状态：`done evidence assembled`

## Goal Closure

- Goal status：`done`
- Success evidence：公共 API、真实平纵重构、联合调整量、现场平面/MDB 对照、确定性纵面闭环、Debug/Release 构建和依赖扫描。
- Stop state：`done`
- Non-goals respected：未修改旧插件，未加入 CAD/UI/MDB/Excel 生产依赖，未增加无序点寻路或综合三维移动量。

## Required Evidence Slots

- Evidence action / check performed：Debug Build + 29 tests + sample；Release Rebuild + 29 tests + sample；重构后移动交点的独立拨量对照；1569 点/22 交点现场对照；生产依赖、lib 对象和 x64 COFF 扫描；旧仓库状态复核。
- Result / exit status：所有命令退出码 0；Debug/Release 测试均为 29/29；移动交点后的两个直线段拨量与独立有向直线公式在 `1e-8 m` 内一致；依赖扫描无命中。
- Covered scope：设计规格第 15 节 8 项成功条件；详细映射见 `docs/verification/2026-07-13-build-and-parity.md`。
- Uncovered scope：用户提供的 XLSX 无 z，没有可供对照的现场纵面/抬落道量；AutoCAD 回接是非目标。
- Residual risk：新线路类型和现场噪声分布超过当前确定性与现场夹具时，需用显式 `ReconstructionOptions` 校准并增加对应回归。
- Confidence grade：`A`

## Baseline / ADR

- Product / Requirement Baseline：`aligned`
- Architecture / Runtime Boundary Baseline：`aligned`
- scope：`both`
- ADR：`docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md`
- Baseline sync：`docs/aegis/baseline/2026-07-13-revised-baseline.md` 已从“尚未实施”同步为实际所有者和验证状态。

## Governance Closure

- Repair Track：旧混合依赖逻辑已边界化迁移为独立 typed 数值所有者。
- Retirement Track：旧 ObjectARX 生产代码按用户范围保留且只读；新库内无兼容 fallback，`legacy_reference` 仅测试链接。
- Residual Risk：未执行旧插件集成替换，因此旧代码退役需要未来单独授权目标。

## Complexity Closure

- Budget status：`within-budget`
- Governed now：生产职责分为几何、typed 线元、平面评估、纵面评估、联合编排和两个 reconstructor；最大生产文件 420 行，低于 800 行软压力线。
- Deferred follow-up：无强制复杂度拆分。
- Completion impact：`complete`

## Evidence References

- `docs/verification/2026-07-13-reference-data.md`
- `docs/verification/2026-07-13-build-and-parity.md`
- `tests/test_field_reference.cpp`
- `tests/test_horizontal_reconstruction.cpp`
- `tests/test_vertical_reconstruction.cpp`
- `tests/test_reconstruction.cpp`
- `README.md`
- `docs/aegis/work/2026-07-13-alignment-reconstruct/proof-bundle.md`
