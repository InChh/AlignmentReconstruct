# AlignmentReconstruct 抽取任务意图

日期：`2026-07-13`

## TaskIntentDraft

- 期望结果：建立独立 VS2026 C++17 静态库，完成平纵线形重构以及逐点联合拨量/抬落道量计算。
- 目标：抽取纯数值行为，移除 ARX/MFC/UI/数据库依赖，并用样例、闭环测试、旧行为对照和新鲜构建证明结果。
- 成功证据：公共 API、真实实现、确定性夹具、逐字段旧行为对照、依赖扫描、Debug/Release x64 构建和测试日志。
- 停止条件：按设计规格定义为 done、blocked、needs-verification 或 scope-exceeded。
- 非目标：CAD/UI/MDB/Excel、无序点寻路、综合三维移动量、旧插件回接、VS2012 ABI 兼容。
- 范围：新兄弟项目；旧 ObjectARX 仓库为只读证据。
- 风险：隐藏全局参数、缓和曲线几何、竖曲线半径语义、端点策略、平纵里程分叉、参考实现耦合。

## BaselineReadSetHint

实施前必须读取：

- `docs/aegis/baseline/2026-07-13-revised-baseline.md`
- `docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md`
- 设计规格第 2 节列出的旧代码证据。

## BaselineUsageDraft

- Required baseline refs：修订基线、修订设计、边界化旧代码证据。
- Acknowledged before plan refs：用户需求、项目名和工具链约束、现有调用链探索结果。
- Cited in design refs：设计规格第 2 节源文件位置。
- Missing refs：用户提供的现场工作簿没有 z，因此没有现场纵面/抬落道量基准；该缺口由独立旧公式和确定性 XYZ 闭环覆盖，但不误报为现场对照。
- Decision：用户已确认设计并授权实施；实现与完成证据见 `20-checkpoint.md` 和 `90-evidence.md`。

## ImpactStatementDraft

- 影响层：公共 API、纯几何、平面重构、纵面重构、平纵评估、联合调整计算、测试、样例、工程元数据。
- 所有者：公共契约、`HorizontalReconstructor`、`VerticalReconstructor`、`HorizontalAlignment`、`VerticalAlignment`、`AlignmentAdjustmentCalculator`、测试专用参考。
- 不变量：标准库依赖、米制主单位、有序输入、右拨为正、抬道为正、平纵共享投影里程、不静默丢点。
- 兼容：受支持输入上的数值一致，不包括旧 ABI/CAD/持久化。
- 非目标：不修改旧仓库生产代码。

## Baseline Role Alignment

- Product / Requirement Baseline：旧基线漏掉抬落道量，并把纵断面输出限制为里程和高程。
- Architecture / Runtime Boundary Baseline：旧接口只拥有平面拨量，无法保证平纵共享里程。
- Result：`Design Defect`
- scope：`both`
- Next action：先用修订设计和基线替换错误定义，用户确认后再编写实施计划。

## Execution Readiness View

- Intent Lock：完整实现两个公共能力，不做占位或折线替代。
- Scope Fence：只改新项目；旧插件只读。
- Baseline Lock：修订基线和修订设计。
- Approved Behavior：修订设计已获确认，两个公共能力、字段、单位、符号和验证边界均已实施。
- Owner / Contract Constraints：每项责任只有一个生产所有者；联合计算器不复制几何。
- Compatibility Boundary：受支持有效输入的旧数值行为。
- Retirement Boundary：本目标不退役旧代码；测试参考永不进入生产。
- Task Batches：公共契约/工程、几何与评估、联合调整、平面重构、纵面重构、夹具/对照、最终验证。
- Test Obligations：确定性、闭环、旧行为对照、校验、依赖扫描、Debug/Release x64。
- Review Gates：书面设计、实施计划、架构审查、完成前验证。
- Drift / Rewind Rules：出现新依赖、第二套里程权威、公共兼容层、旧插件集成或符号/单位变化时返回设计。
- Evidence Required Before Completion：设计规格第 14 节全部证据。
- Advisory Boundary：Aegis 记录不授予完成状态。
