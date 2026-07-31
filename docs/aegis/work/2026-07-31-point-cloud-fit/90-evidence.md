# 点云中心线拟合与等间隔采样 Evidence

日期：`2026-07-31`

## Delivered Scope

- 公共入口：
  `fit_point_cloud(const std::vector<Point3D>& unordered_points, double interval_m)`。
- 公共结果：`PointCloudFitResult`，包含米制 `centerline_points`、状态和诊断。
- 唯一生产 owner：`detail::PointCloudFitter`。
- 算法：一米 X 空间分箱、每箱 XYZ 中位数、Y/Z 三轮鲁棒局部二次拟合、
  按 XY 水平弧长等间隔采样并保留实际终点。
- 工程清单、三入口样例、README、AGENTS、设计、基线和 ADR-0002 已同步。

## Fresh Build and Regression Evidence

| 检查 | 结果 |
| --- | --- |
| 目标仓库 `Debug|x64` solution Rebuild | 退出 `0`，`/W4 /WX`，静态库、测试和样例均生成 |
| 目标仓库 Debug 全量测试 | 48 个，失败 `0`，跳过 2 个未请求的可选外部夹具测试；既有私有现场夹具实际通过 |
| Debug 三入口样例 | 退出 `0`，点云输出 101 个中心线点，后续重构和调整量成功 |
| 目标仓库 `Release|x64` solution Rebuild | 退出 `0`，静态库、测试和样例均生成 |
| 目标仓库 Release 全量测试 | 48 个，失败 `0`，跳过 2 个未请求的可选外部夹具测试；既有私有现场夹具实际通过 |
| Release 三入口样例 | 退出 `0`，结果与 Debug 一致 |

点云测试共 9 个，覆盖直线中心、XY 水平弧长采样、输入行序不变性、大间隔端点、
非法间隔、非有限坐标索引、每箱最小点数、最小空间分箱数和真实 Python 参考。

## Real Point-Cloud Parity

- 输入：本会话原始坐标文件，26012 个 XYZ 点；测试边界把毫米乘 `0.001` 转为米。
- 调用：`fit_point_cloud(points_m, 10.0)`。
- C++ 输出：262 个米制中心线点。
- Python 参考：262 个点；C++ 每个 X/Y/Z 坐标误差均不超过
  `0.0000001 m`（`0.0001 mm`）。
- 将 26012 行输入全部反转后再次调用，262 个输出点逐坐标完全一致。
- 私有输入和参考只在验证时临时复制到忽略的 `build/` 目录，验证后临时副本已删除；
  原始文件未修改，源码、测试、README 和工程文件没有本机绝对数据路径。

## Boundary and Artifact Evidence

- 生产依赖扫描：
  `afx|AcDb|AcGe|ObjectARX|CString|ads_|aced|XLDataBase|Excel|legacy_reference`
  在 `include`、`src` 和生产 `.vcxproj` 中无命中。
- 绝对测试数据路径扫描在 `include`、`src`、`tests`、`README.md`、工程文件和
  `samples` 中无命中。
- 库和测试 `.vcxproj` / `.filters` 均包含新增公共头、owner 源文件和测试文件。
- `dumpbin /linkermember:2 build/Release/AlignmentReconstruct.lib` 可见
  `fit_point_cloud@alignment_reconstruct` 公共 C++ 符号。
- `git diff --check` 退出 `0`。
- 隔离副本补丁应用后，目标仓库 30 个涉及文件逐字节一致。
- `aegis-workspace.py bundle` 已生成 proof bundle，
  `aegis-workspace.py check --root .` 在目标仓库退出 `0`。

## Complexity Closure

- Budget status: `within-budget`
- 最大新增生产文件：`src/point_cloud/point_cloud_fitter.cpp`，574 行，低于 800 行
  软压力线。
- 公共转发层保持 85 行；测试文件 264 行。
- 新职责没有塞入现有重构或线路 owner；没有新增 fallback、适配器或重复生产路径。
- Net entropy: `increased-with-justification`，对应一个用户要求且职责独立的新 owner。

## Uncovered Boundary

- 只验证并支持沿 X 方向单值展开的铁路点云；没有验证回头、自交、分岔或任意方向
  点云寻路。
- 两个与本次无关、需要另行请求外部 XYZ/纵面文件的可选测试按契约跳过；目标仓库
  既有私有线路现场夹具和本次点云真实参考测试均已实际运行并通过。

## Confidence

`A`：有目标函数真实数据逐点对照、输入反序对照、双配置全量回归、样例和生产边界
证据；未覆盖范围与公开适用边界一致。

## EvidenceBundleDraft

- Artifact key: target-verification
- Type: build-test-parity
- Source: Debug/Release Rebuild, 48-test suites, samples, real 26012-point Python parity, dependency and symbol scans
- Summary: All commands exited 0; 262 C++ coordinates match Python within 1e-7 m and reversed input is identical
- Verifier: Codex direct command evidence on 2026-07-31
