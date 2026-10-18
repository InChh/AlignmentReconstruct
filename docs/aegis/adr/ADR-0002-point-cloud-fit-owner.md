# ADR-0002 - 点云中心线拟合与等间隔采样所有者

Status: `recorded-from-work`
Date: `2026-07-31`
Amended: `2026-10-18`

## Source Evidence

- 已实现的 fit_point_cloud、PointCloudFitter、定向测试和真实 Python 数值对照
## Context

调用方需要把行序可能无意义的宽带 XYZ 铁路点云压缩为三维中心线点；现有 reconstruct 要求有序中心线输入且输出参数化平纵方案。

## Decision

新增公共 fit_point_cloud；PointCloudFitter 独占平面主方向估计、一米局部纵向分箱、中位数锚点、鲁棒局部二次拟合和按 XY 水平弧长采样。输入行序无关，但要求线路具有稳定全局纵向并沿该方向单值展开。主/次平面协方差特征值比低于 `4.0` 时失败；主方向距公共 x 或 y 轴不超过 `6°` 时吸附到该轴。

2026-10-18：取消每箱至少 3 点和至少 20 箱的硬门槛，支持稀疏无宽度中心线和带宽度点云；仍要求公共入口至少 10 个有限点以及至少 3 个不同局部纵向锚点。短范围缩小箱宽；稀疏支持按实际邻距扩展局部拟合窗口并归一化自变量。稀疏宽带按横向残差尺度确定平面窗口，以局部 PCA 切向估计断面纵向位置，用点数加权分箱均值和鲁棒拟合提取中心线及高程，减小端部横向宽度造成的纵向偏移。该局部坐标处理不生成参数化线路，不取代 `HorizontalAlignment` 的方案投影和 station 权威。密集且局部支持充分的数据保留原路径，公共函数签名、单位、坐标和导出不变。

## Alternatives Considered

- 让 reconstruct 自动排序和压缩点云；拒绝，因为会破坏既有有序输入契约。
- 引入 PCL、Eigen 或 Python 运行时；拒绝，因为生产库只允许 C++17 标准库。
- 实现任意拓扑点云寻路；拒绝，因为超出已验证数据和用户批准边界。
## Consequences

- C++ 调用方可在内存中得到等间隔中心线，再按需要传给 reconstruct。
- 无稳定平面主方向、沿所选纵向非单值或空间支持不足的数据明确失败，不猜测路径。
- 输出会逆变换回调用方原始 XY 坐标系，公共 `x=N/y=E/z=H` 契约和 DLL ABI 不变。
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
