# 点云中心线拟合与等间隔采样 Reflection

日期：`2026-07-31`

## Goal Closure

- Goal：把本会话 Python 点云中心线算法迁移为 C++17 静态库公共函数。
- Result：`fit_point_cloud`、结果类型、唯一 owner、测试、样例和公共文档均已实现。
- Stop state：`done`。
- Non-goals respected：没有加入文件解析、CAD/ARX/MFC、第三方数值库、通用点云
  寻路或旧 Python 运行时。

## Key Decisions

- 没有改变 `reconstruct` 的有序中心线输入契约；宽带无序点云由独立
  `PointCloudFitter` 压缩。
- 输入行序被明确排除在计算之外；输出方向固定为最小 X 到最大 X。
- 固定复现 Python 的一米分箱、`12 m` / `6 m` 半窗和三轮鲁棒拟合参数。
- 采样间隔按 XY 水平弧长定义，Z 不参与；实际终点按 Python 的 `0.01 m` 规则保留。

## Verification Reflection

- 合成测试约束数学契约，真实 26012 点数据约束跨语言迁移误差，两者都需要；
  只做其中一个不足以支持完成结论。
- Release 静态库最初使用 `dumpbin /symbols` 没有匹配公共名称。读取静态库成员索引后
  确认这是检查模式不适合该库产物；改用 `dumpbin /linkermember:2` 后直接看到
  `fit_point_cloud` 符号，未修改生产代码。
- Debug 和 Release 都重新构建并运行同一真实参考测试，避免只用 Debug 证明数值路径。

## Repair and Retirement

- Repair track：新增的是缺失能力，不是修补现有重构；正确 owner 是新
  `PointCloudFitter`，公共层只做校验和诊断映射。
- Retirement track：现有 `reconstruct`、`HorizontalReconstructor` 和 Python 成果
  均未被替换。Python 仅保留为本次外部参考，不链接、不复制到生产库，也没有第二套
  C++ fallback。

## Architecture and Baseline

- ADR gate：公共 API 和唯一 owner 难以低成本撤销；不扩张 `reconstruct` 的原因若
  无上下文会令人困惑；独立 owner、复用旧 owner、第三方库和通用寻路之间存在真实
  权衡，因此创建 ADR-0002。
- Baseline sync：修订基线、设计规格、README 和 AGENTS 已从两项公共能力同步为三项，
  并记录 X 单值适用边界。
- Complexity closure：`within-budget`；最大新增 owner 574 行，无重复职责或 fallback。

## Residual Risk

如果未来数据不能沿 X 单值展开，需要先增加明确的方向/拓扑契约并重新设计 owner；
不能在当前实现中按样本追加特例。当前完成结论只覆盖已记录的 X 分箱铁路点云边界。
