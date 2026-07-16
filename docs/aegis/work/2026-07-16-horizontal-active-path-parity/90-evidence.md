# 平面重构活跃旧路径毫米级迁移 - Evidence

## Authority Evidence

- 活跃 UI 入口：`../主控/QXBoLiangCalDlg.cpp:70-258`。
- 直曲自动分段：`OnBnClickedButtonZdfd` → `Sort` → `JMSurPtLC` → `CalCurvature` → `MarkProperty` → `IDdentifyLineOrCurve` → 动态阈值 5 → `DealFanxiangQX` → `CalFJD` → `CheckCurve` → 空的 `Checkline` → 动态阈值 6。
- 平面线位生成：`OnBnClickedButtonNhht` → `FormLineSeg`；有曲线时 `IDSpiral` → `SwingAlgorithm`；最后 `AllQXNH2`。
- 数值被调函数：`../主控/CeDianNiHe.cpp:848-4917`；交点方案建线和投影继续以 `RHA.cpp`、`BAS_CALC_FUN.cpp` 的活跃函数为依据。
- 排除项：`CH_V_Cal` 的 `_SW` 平面副本、`IDdentifyLineOrCurve_New`、`AllQXNH2opti`、`SwingAlgorithm_SinglePI`、注释实现和 UI/CAD/Excel 代码均未作为本次平面算法权威。

## Strict RED

- Artifact key: `strict-field-red`
- Source: 2026-07-16 迁移前 Debug 严格现场测试。
- Result: 22 个控制点数量保留，但最大误差为坐标 `0.232059 m`、半径 `3.16358 m`、进口缓长 `5.92673 m`、出口缓长 `3.8956 m`；测试退出 1。

## Strict GREEN

- Artifact key: `strict-field-green`
- Source: Visual Studio 2026 Debug 和 Release 全量测试，2026-07-16。
- Control count/type: 22 个控制点，数量、顺序和类型完全一致。
- Maximum coordinate error: `0.000000649004 m`，control 12。
- Maximum radius error: `0.000417710 m`，control 4。
- Maximum entry spiral error: `0.000433860 m`，control 8。
- Maximum exit spiral error: `0.000482413 m`，control 7。
- Acceptance: 四类最大误差均 `< 0.001 m`；Debug 和 Release 结果一致。

## Regression Evidence

- Debug `Rebuild` 退出 0；30 项测试 failures 0、skipped 0；样例退出 0。
- Release `Rebuild` 退出 0；30 项测试 failures 0、skipped 0；样例退出 0。
- 现场公共拨量：最大里程误差 `0.000128355 m`，最大拨量误差 `0.000105252 m`，拨量 RMS `0.0000130251 m`。
- 修改重构交点 `y + 25 m` 后，`calculate_adjustments` 的进、出直线拨量与独立有向直线公式分别一致到测试容差，整线最大绝对拨量 `17.2808661702 m`。
- 纵断面、竖曲线、抬落道量和公共诊断测试全部通过。

## Root Cause Evidence

- 旧 `YNH2` 用 `sum / count` 直接计算圆心均值；迁移实现一度改写为 `sum * (1.0 / count)`。
- 两式数学等价但最后一位浮点种子不同；现场 Powell 目标较平坦，该差异最终放大为厘米级半径和缓长漂移。
- 恢复旧运算顺序后，独立源码顺序 oracle 与生产实现均进入毫米级；没有放宽容差或加入现场专用分支。

## Boundary Evidence

- `include`、`src` 和生产 `.vcxproj` 的 ARX/MFC/CAD/Excel/测试依赖扫描无命中。
- 运行时代码、测试、样例、README 和项目文件的本机绝对路径扫描无命中。
- Git 跟踪的 `tests/fixtures/*.csv` 数量为 0。
- 临时诊断、探针测试、Nelder-Mead、曲率排名选圆点和旧简化路径符号扫描无命中。

## Evidence Boundary

本证据完成本次已框定的现场 22 控制点毫米级迁移和相关回归验收。少于 3 个圆样本时 `AllQXNH2` 可达的 `BCTTP`/`BCTPP` 稀疏分支没有现场夹具覆盖，不在本证据的完成声明内。
