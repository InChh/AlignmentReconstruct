# ADR-0002 - 点云中心线拟合与等间隔采样所有者

Status: `recorded-from-work`
Date: `2026-07-31`

## Source Evidence

- 已实现的 fit_point_cloud、PointCloudFitter、定向测试和真实 Python 数值对照
## Context

调用方需要把行序可能无意义的宽带 XYZ 铁路点云压缩为三维中心线点；现有 reconstruct 要求有序中心线输入且输出参数化平纵方案。

## Decision

新增公共 fit_point_cloud；PointCloudFitter 独占一米 X 分箱、中位数锚点、鲁棒局部二次拟合和按 XY 水平弧长采样；输入行序无关但要求线路沿 X 单值展开。

## Alternatives Considered

- 让 reconstruct 自动排序和压缩点云；拒绝，因为会破坏既有有序输入契约。
- 引入 PCL、Eigen 或 Python 运行时；拒绝，因为生产库只允许 C++17 标准库。
- 实现任意拓扑点云寻路；拒绝，因为超出已验证数据和用户批准边界。
## Consequences

- C++ 调用方可在内存中得到等间隔中心线，再按需要传给 reconstruct。
- 非 X 单值或空间支持不足的数据明确失败，不猜测路径。
## Compatibility Boundary

复现本任务已验证算法，不承诺任意点云拓扑、三维弧长间隔、文件解析或旧 Python ABI；现有 reconstruct 与 calculate_adjustments 保持不变。

## Retirement Impact

不替换或保留第二套 C++ owner；Python 只作为迁移验证参考，不进入生产库。

## Baseline Sync

- Needed: needed
- Target: docs/aegis/baseline/2026-07-13-revised-baseline.md
- Action: update baseline
- Reason: 公共能力、所有权图和兼容边界均新增，基线已同步第三个入口和 PointCloudFitter。

## Evidence References

- tests/test_point_cloud_fit.cpp
- docs/aegis/plans/2026-07-31-point-cloud-fit-implementation.md
- docs/aegis/work/2026-07-31-point-cloud-fit/90-evidence.md
## Boundary

This ADR is an advisory Aegis Method Pack record. It does not grant completion authority or replace project-authoritative architecture sources.
