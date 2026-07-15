# AlignmentReconstruct 构建与数值对照记录

日期：`2026-07-13`  
补充验证：`2026-07-14`（重构后移动交点再计算拨量）

## 1. 验证对象

- 目录：`D:\Programming\C++\Projects\既有线\AlignmentReconstruct`
- 解决方案：`AlignmentReconstruct.sln`
- 生产产物：`AlignmentReconstruct.lib`
- 工具链：Visual Studio 2026，MSBuild `18.7.8`，MSVC `14.51.36231`，`v145`，C++17，x64
- 测试程序：`AlignmentReconstruct.Tests.exe`
- 公共样例：`AlignmentReconstruct.Sample.exe`

## 2. 新鲜构建证据

在校验并清理 `build` 目录后，使用 Git Bash 执行：

```bash
MSYS2_ARG_CONV_EXCL='*' \
  'D:/Programs/Microsoft Visual Studio/2026/MSBuild/Current/Bin/amd64/MSBuild.exe' \
  AlignmentReconstruct.sln /m /t:Build \
  /p:Configuration=Debug /p:Platform=x64 /v:minimal

./build/Debug/AlignmentReconstruct.Tests.exe
./build/Debug/AlignmentReconstruct.Sample.exe

MSYS2_ARG_CONV_EXCL='*' \
  'D:/Programs/Microsoft Visual Studio/2026/MSBuild/Current/Bin/amd64/MSBuild.exe' \
  AlignmentReconstruct.sln /m /t:Rebuild \
  /p:Configuration=Release /p:Platform=x64 /v:minimal

./build/Release/AlignmentReconstruct.Tests.exe
./build/Release/AlignmentReconstruct.Sample.exe
```

结果：

- `Debug|x64`：静态库、测试和样例全部编译/链接成功；29 项测试通过，0 失败；样例退出码 0。
- `Release|x64`：执行完整 Rebuild；29 项测试通过，0 失败；样例退出码 0。
- `dumpbin /headers build/Release/AlignmentReconstruct.lib`：COFF machine 为 `8664 (x64)`。
- `lib /list`：生产库只含 14 个生产 `.obj`，不含 `legacy_reference` 或测试对象。
- 74 个维护文本文件通过 CRLF 扫描；私有现场夹具在本地验证时保持字节不变，公开仓库不包含这些文件。
- Aegis workspace `bundle` 和 `check` 均成功，proof bundle 已落盘并进入索引。

## 3. 现场数据证据

权威来源是用户提供的私有现场工作簿和数据库。公开记录只保留验证范围：1569 个测点、22 个平面控制点和 1569 个拨量结果；原始路径、数据行和数据指纹不进入公开仓库。

### 3.1 公共调整量 API 对照 MDB

`calculate_adjustments` 对 1569 个点的结果：

| 字段 | 最大绝对误差 | 最大误差点 | RMS 误差 | 测试容差 |
| --- | ---: | ---: | ---: | ---: |
| 投影里程 | `0.000128355 m` | 775 | - | `0.02 m` |
| 带符号拨量 | `0.000105252 m` | 61 | `0.0000130251 m` | 最大 `0.002 m`，RMS `0.0005 m` |

数值偏差小于 0.11 mm，且正负号与 MDB 一致。

### 3.2 现场平面重构对照

`reconstruct` 的平面主链恢复 22 个控制点：

| 字段 | 最大绝对误差 | 最大误差交点 | 测试容差 |
| --- | ---: | ---: | ---: |
| 交点坐标平面距离 | `0.232059 m` | JD2 | `0.5 m` |
| 曲线半径 | `3.16358 m` | JD6 | `150 m` |
| 进/出口缓和曲线长 | `5.92673 m` | JD3 | `40 m` |

这些容差是对含实测拨量和采样间距的重构结果设定，不用于已知方案的直接求值；已知方案的直线/圆/缓和曲线坐标级公式对照精度为 `1e-9/1e-7 m`。

### 3.3 重构线路修改交点后的拨量验证

`public_adjustment_after_reconstructed_intersection_move_matches_tangent_geometry` 执行完整公共调用链：先由带圆曲线和不等缓长的 XYZ 样本调用 `reconstruct`，再将重构结果中唯一中间交点的 `y` 坐标增加 `25 m`，最后将原线路全部测点传给 `calculate_adjustments`。

- 全部原测点均返回调整结果，没有丢点；全线最大绝对拨量为 `18.3637375675 m`。
- 在交点前、后两条受影响直线段各选一个测点，用独立有向直线叉积公式计算期望拨量。
- 进口侧：API 为 `-4.16576273637 m`，独立期望为 `-4.16576273637 m`。
- 出口侧：API 为 `-2.61415574829 m`，独立期望为 `-2.61415574829 m`。
- 米值容差为 `1e-8 m`，毫米便捷值容差为 `1e-5 mm`；Debug 和 Release 均通过。

## 4. 纵断面与联合闭环

原 XLSX 工作表只有 N/E，没有 z，因此不将它误报为现场纵面证据。纵面使用以下独立证据：

- `legacy_reference::vertical_elevation` 按活跃 `VerticleArc` 物理圆半径公式生成样本；
- 非等间距二阶差商与人工解对照；
- 直坡段、多变坡点、凸/凹圆竖曲线、零半径折坡、重叠拒绝和特殊 `<100` 旧编码拒绝；
- 纵面重构恢复变坡点 station/elevation/radius；
- 完整 XYZ 样本重构后回灌 `calculate_adjustments`，证明拨量与抬落道量共享同一 station，且设计线上残差在闭环容差内。

## 5. 依赖和边界审查

执行：

```bash
rg -n -i \
  'afx|AcDb|AcGe|ObjectARX|CString|ads_|aced|XLDataBase|Excel|legacy_reference' \
  include src AlignmentReconstruct.vcxproj
```

结果：无命中。

进一步核对：

- `AlignmentReconstruct.vcxproj` 为 `StaticLibrary`，Debug/Release 均为 `v145` + `stdcpp17` + x64。
- 公共头文件只包含 C++ 标准库头文件。
- `AlignmentAdjustmentCalculator` 仅调用 `HorizontalAlignment::project` 和 `VerticalAlignment::evaluate`，没有第二套投影或竖曲线公式。
- `tests/legacy_reference` 只在测试工程清单中，生产 `.lib` 不含对应对象。
- 旧 `D:\Programming\C++\Projects\既有线\主控` 仓库状态与开始时一致；本实现没有改写旧生产代码。

## 6. 设计规格完成审计

| 成功条件 | 权威证据 | 结论 |
| --- | --- | --- |
| 指定目录中有 solution/静态库/测试 | 文件树、Debug/Release 产物 | 已证明 |
| 两个公共操作是真实实现 | `src/reconstruction.cpp`、`src/adjustment.cpp`及闭环/现场测试 | 已证明 |
| 平面与纵面输出字段完整 | 公共契约、合成与现场重构测试 | 已证明 |
| 逐点联合调整结果 | 现场公共 API 测试和平纵合成测试 | 已证明 |
| 确定性、闭环、旧行为对照 | 29 项测试及上述数值指标 | 已证明 |
| Debug/Release x64 | 全新 Build + Rebuild + 两组测试 | 已证明 |
| 无 ARX/MFC 泄漏 | 依赖扫描、项目清单、lib 对象列表 | 已证明 |
| 架构与契约无未解决漂移 | ADR-0001、修订基线、唯一所有者/依赖方向审查 | `aligned`，`scope: both` |

## 7. 未覆盖范围和剩余风险

- 没有现场 z/纵面方案/抬落道量 MDB 对照数据；当前纵面证据是独立旧公式和确定性闭环，不宣称未提供的现场纵面数据已对照。
- 没有集成回 AutoCAD/ObjectARX 宿主；这是明确非目标，不是静态库缺失。
- 目标目录未初始化 Git，因此没有提交/分支证据；用 Aegis 工作记录和完整文件树保存可恢复证据。

结论：对已确认的静态库产品边界，完成证据等级为 `A`。
