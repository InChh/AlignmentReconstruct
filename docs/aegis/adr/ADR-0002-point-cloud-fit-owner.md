# ADR-0002 - 点云中心线拟合与等间隔采样所有者

Status: `recorded-from-work`
Date: `2026-07-31`
Amended: `2026-10-20`

## Source Evidence

- 已实现的 fit_point_cloud、PointCloudFitter、定向测试和真实 Python 数值对照
## Context

调用方需要把行序可能无意义的宽带 XYZ 铁路点云压缩为三维中心线点；现有 reconstruct 要求有序中心线输入且输出参数化平纵方案。

## Decision

新增公共 fit_point_cloud；PointCloudFitter 独占平面主方向估计、一米局部纵向分箱、中位数锚点、鲁棒局部二次拟合和按 XY 水平弧长采样。输入行序无关，但要求线路具有稳定全局纵向并沿该方向单值展开。主/次平面协方差特征值比低于 `4.0` 时失败；主方向距公共 x 或 y 轴不超过 `6°` 时吸附到该轴。

2026-10-18：取消每箱至少 3 点和至少 20 箱的硬门槛，支持稀疏无宽度中心线和带宽度点云；仍要求公共入口至少 10 个有限点以及至少 3 个不同局部纵向锚点。短范围缩小箱宽；稀疏支持按实际邻距扩展局部拟合窗口并归一化自变量。稀疏宽带按横向残差尺度确定平面窗口，以局部 PCA 切向估计断面纵向位置，用点数加权分箱均值和鲁棒拟合提取中心线及高程，减小端部横向宽度造成的纵向偏移。该局部坐标处理不生成参数化线路，不取代 `HorizontalAlignment` 的方案投影和 station 权威。密集且局部支持充分的数据保留原路径，公共函数签名、单位、坐标和导出不变。

同日精度优化：实测对照发现上述宽带平面均值仍会追随偏侧采样密度，端点及内部区段均存在米级偏差。宽带平面改为局部 1% / 99% 分位回归边界的中点，边界不共同支持曲率时采用局部一次趋势，并迭代校正断面纵向位置；高程保留加权鲁棒拟合。该中心表示已观测宽带的几何中心，不能由严重单侧缺测的点云推断未观测边界。稀疏无宽度与既有密集路径保持原算法，数值回归和证据见 `docs/verification/2026-10-18-wide-cloud-centering.md`。

后续回归修复：独立极端分位边界对高斯及重尾散点、连续少量粗差和端部数值扰动不稳定。按分位跨度识别中央集中的散点并沿用鲁棒中心趋势；有界宽带改用共享二次中心及线性半宽的联合分位模型，以平滑分位损失、阻尼 Newton 和线搜索求解。全局分布判断由纵向分区的局部残差分布确认，避免渐变宽度误判。分位邻域使用至少 100 个观测（不足时全部）及 3 个不同纵向位置，并合并几乎重复的末端网格；分离的少数粗差降低边界拟合权重，并以有效支持确定采样范围。宽带投影切向由较宽窗口的鲁棒中心趋势提供；仅新增宽带辅助二次拟合允许在鲁棒权重退化时，以同邻域距离权重重试同阶模型，失败仍上抛原诊断。PCA 保持原局部协方差估计，高程保持原鲁棒模型。输入行序不变性、任意主方向、原始两份数据及其他分布的验证边界见 `docs/verification/2026-10-18-point-cloud-robustness-and-headings.md`。

2026-10-20：历史版本同条件对照确认，位置残差的改善不足以证明中心线形状及后续缓长参数正确。对于全部观测可按空间投影分为近共线宽横断面的输入，PointCloudFitter 在原趋势后利用可靠断面方向恢复中心形状，以非均匀间距二阶方向修正减少积分离散误差，用全部原始观测的双侧分位数确定公共法向偏移。分组与方向均按确定性的局部平面基准求取，不读取输入行序、不预设断面点数或 R/L 参数；结构不足或恢复不满足原单值边界时仍采用原趋势。该机制可覆盖稀疏及密集的结构宽带，非结构化、中央集中散点和无宽度路径保持既有语义。高程使用同一物理断面的原观测和既有鲁棒模型，采样仍由本 owner 的 XY 弧长实现完成。未引入参数化线路、第二套投影或外部依赖。证据和已知短缓长边界见 docs/verification/2026-10-20-scanline-centerline-fidelity.md。

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
