# AlignmentReconstruct 动态库改造 Intent

日期：`2026-08-01`

## Requested Outcome

将 `AlignmentReconstruct` 从静态库改为 MSVC 动态库，只导出
`fit_point_cloud`、`reconstruct` 和 `calculate_adjustments`。

## Scope and Non-goals

- Scope：公共可见性头、三个入口声明、便捷成员内联、VS 工程产物、测试/样例
  动态链接闭环及必要文档。
- Non-goals：算法调整、C ABI、静态兼容配置、CAD/ARX 集成、第三方依赖。
- Risk：公共 STL 类型跨 DLL 边界要求兼容 MSVC 工具集和匹配 `/MDd`/`/MD`。

## BaselineReadSetHint

- `AGENTS.md`
- `README.md`
- `docs/aegis/baseline/2026-07-13-revised-baseline.md`
- `docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md`
- `docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md`
- `docs/aegis/adr/ADR-0002-point-cloud-fit-owner.md`

## BaselineUsageDraft

- Required baseline refs：已全部读取。
- Acknowledged before plan refs：AGENTS、README、修订基线、完整设计、ADR-0001、ADR-0002。
- Cited in plan refs：修订基线、完整设计、ADR-0001、已确认 DLL 设计。
- Missing refs：无。
- Decision：`continue`。

## ImpactStatementDraft

- Affected layers：公共 ABI、Visual Studio 产物和运行时部署。
- Owners：`include/alignment_reconstruct/api.h`、三个现有公共入口头、生产 vcxproj。
- Invariants：三入口签名、米制单位、诊断、数值 owner 和算法均不变。
- Compatibility：源代码调用形状不变；二进制交付改为 DLL + 导入库。
- Retirement：静态库配置和 `src/types.cpp` 退役，不保留 fallback。

## Execution Readiness View

- Intent Lock：只完成已确认的三个入口 DLL 化。
- Scope Fence：不修改点云、平面、纵面、投影或调整量算法。
- Baseline Lock：公共单位和数值兼容基线保持不变。
- Approved Behavior：三个现有 C++ 函数由 DLL 导入，调用语义不变。
- Owner / Contract Constraints：`api.h` 独占可见性宏；内部 owner 不导出。
- Compatibility Boundary：VS2026/v145/x64，Debug `/MDd`，Release `/MD`。
- Retirement Boundary：删除外部便捷成员实现和静态库配置，不并行保留旧产物。
- Task Batches：公共头 → 工程配置 → 文档 → Debug/Release 动态链接验证。
- Test Obligations：48 项现有测试、样例、三项导出表、运行时依赖检查。
- Review Gates：导出表出现第四项、算法 diff 或生产依赖新增均停止。
- Drift / Rewind Rules：任何新 C ABI、适配器或算法改动返回设计阶段。
- Evidence Required Before Completion：新鲜 Rebuild、测试、样例、dumpbin、依赖扫描、CRLF。
- Advisory Boundary：method-pack execution guidance only; not GateDecision, PolicySnapshot, or completion authority.
