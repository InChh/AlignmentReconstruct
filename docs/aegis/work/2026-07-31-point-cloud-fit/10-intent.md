# 点云中心线拟合与等间隔采样 Intent

日期：`2026-07-31`

## Requested Outcome

在 `AlignmentReconstruct` 中新增公开的 `fit_point_cloud`，输入米制、行序无关的
三维铁路点云和采样间隔，输出沿拟合中心线 XY 水平弧长等间隔分布的米制三维点。

## Scope

- 新公共契约、独立点云拟合 owner、测试、工程清单和契约文档。
- 精确迁移本任务已经验证的 Python 分箱、鲁棒局部二次拟合和弧长采样行为。
- 真实点云仅作为环境变量驱动的本机对照，不进入仓库。

## Non-goals

- 不改变 `reconstruct` 的有序输入契约。
- 不实现任意拓扑、回头或自交点云的通用寻路。
- 不引入 CAD、文件解析、数据库、Python 运行时或第三方数值库。

## BaselineReadSetHint

- `AGENTS.md`
- `README.md`
- `docs/aegis/baseline/2026-07-13-revised-baseline.md`
- `docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md`
- `docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md`
- `include/alignment_reconstruct/`

## BaselineUsageDraft

- Required refs: 上述全部文件。
- Acknowledged before implementation: 是。
- Cited by plan:
  `docs/aegis/plans/2026-07-31-point-cloud-fit-implementation.md`。
- Missing refs: 无。
- Decision: `continue`

## ImpactStatementDraft

- Public API: 追加 `fit_point_cloud` 和 `PointCloudFitResult`。
- Architecture: 追加 `PointCloudFitter`，不改变现有几何 owner。
- Units: 新 API 输入、间隔和输出全部为米。
- Compatibility: 现有 API 和枚举已有值保持不变；诊断码只追加。
- Verification: 合成、失败、洗牌、真实 Python 对照和 Debug/Release 全量回归。

## Execution Readiness View

- Intent Lock: C++ 中复现已验证 Python 点云拟合采样行为。
- Scope Fence: 纯数值静态库，不做通用寻路或文件 I/O。
- Baseline Lock: C++17、标准库、米制、稳定诊断、唯一 owner。
- Owner / Contract Constraints: `PointCloudFitter` 独占点云拟合；旧 owner 不变。
- Compatibility Boundary: 追加式公共 API。
- Retirement Boundary: 无 fallback；Python 只作迁移参考。
- Test Obligations: 合成、非法输入、输入洗牌、真实逐点对照、全量回归。
- Review Gates: API、数值一致性、生产依赖和双配置构建。
- Drift / Rewind Rules: 若需要改变 `reconstruct` 或引入任意拓扑寻路则停止。
- Evidence Required Before Completion: 计划中 Task 5 的全部证据。
- Advisory Boundary: 仅为方法流程提示，不是完成授权。
