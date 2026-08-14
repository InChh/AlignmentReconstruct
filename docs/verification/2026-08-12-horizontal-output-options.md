# 平面取整与等缓长选项验证记录

日期：`2026-08-12`

## 1. 验证对象

本次将参考项目“全线坐标法”的三个纯数值平面成果选项加入
`ReconstructionOptions`：

- `horizontal_curve_radius_rounding_m`；
- `horizontal_spiral_length_rounding_m`；
- `equalize_horizontal_spiral_lengths`。

`HorizontalReconstructor` 在现有 `(l1, R, l2)` 三变量优化之后，按“前后缓长取算术平均
→ 半径取整 → 两个缓长分别取整”处理最终控制点。默认值为 `0.0/0.0/false`。

## 2. 参考行为核对

- `../主控/QXBoLiangCalDlg.cpp`：半径标准选项映射为 `0/1/10/50/100 m`，
  缓长标准选项映射为 `0/1/5/10 m`，默认不取整且不勾选等缓长。
- `../主控/CeDianNiHe.cpp`：活跃 `AllQXNH2` 路径先三变量优化，再取缓长平均，
  最后取整半径、前缓长和后缓长。
- `../主控/BAS_CALC_FUN.cpp`：`Digit` 在余数严格大于半步时进位，恰好半步停留在较低档。
- 参考项目在实施前后 `git status --short` 均无输出；本任务未修改参考项目。

## 3. 测试先行证据

先只增加新契约和行为测试，执行 `Debug|x64 Rebuild`。构建按预期失败：

- `test_contract.cpp` 报告三个新字段尚不是 `ReconstructionOptions` 成员；
- `test_horizontal_reconstruction.cpp` 报告 `apply_horizontal_output_options` 尚不存在；
- `test_reconstruction.cpp` 报告公共非法参数和处理后线路测试缺少新字段。

实现后同一组测试可编译并通过。

## 4. 定向行为覆盖

| 行为 | 证据 |
| --- | --- |
| 默认不改变平面参数 | 关闭选项时半径、两个缓长和原优化误差逐字段保持 |
| 参考标准步长 | 半径 `1/10/50/100 m`、缓长 `1/5/10 m` 表驱动测试 |
| `Digit` 恰好半步规则 | `124.999/125.0/125.001` 按 `10 m` 取整得到 `120/120/130 m` |
| 先平均后取整 | `14/18 m` 先平均为 `16 m`，再按 `10 m` 得到 `20/20 m` |
| 非法步长 | 两字段的负数、NaN 和无穷大均返回 `invalid_input/non_finite_value/no_input_index`，且不返回部分输出 |
| 不可行几何不回退 | 将已拟合半径按 `2000 m` 取整到零时返回 `infeasible_geometry`，所有输出为空 |
| 唯一处理后线路 | 重建返回线路后独立重建 `HorizontalAlignment`，逐点重新投影的 station 与 `projected_stations_m` 在 `1e-9 m` 内一致 |

## 5. 新鲜构建与运行

在 Git Bash 中使用 VS2026 x64 MSBuild `18.8.2` 执行：

```bash
MSYS2_ARG_CONV_EXCL='*' \
  'D:/Programs/Microsoft Visual Studio/2026/MSBuild/Current/Bin/amd64/MSBuild.exe' \
  AlignmentReconstruct.sln /m /t:Rebuild \
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

- `Debug|x64`：Rebuild 成功；54 项测试通过，0 失败，3 项外部环境夹具按契约跳过；样例退出码 0。
- `Release|x64`：Rebuild 成功；54 项测试通过，0 失败，3 项外部环境夹具按契约跳过；样例退出码 0。
- 现场平面默认选项回归恢复 22 个控制点；最大坐标误差
  `0.000000649004 m`，最大半径误差 `0.00041771 m`，最大进/出口缓长误差
  `0.00043386/0.000482413 m`，均小于现有 `0.001 m` 容差。
- 公共调整量现场回归保持：投影 station 最大误差 `0.000128355 m`，
  拨量最大误差 `0.000105252 m`，拨量 RMS `0.0000130251 m`。

跳过的 3 项是需要额外环境变量的点云 Python 参考、外部 XYZ 重构和外部纵面夹具；
本地已有的私有现场平面/MDB 回归已实际运行，不是跳过。

## 6. 导出、依赖和边界

- `dumpbin /exports` 对 Debug/Release DLL 均报告 `3 number of functions`和
  `3 number of names`：`fit_point_cloud`、`reconstruct`、`calculate_adjustments`。
- `dumpbin /dependents` 证明 Debug/Release 的测试和样例都依赖
  `AlignmentReconstruct.dll`。
- 生产项目和 `AlignmentReconstruct.InternalSources.props` 无测试或 `legacy_reference`
  条目；`dumpbin /symbols` 在两个 DLL 中均无测试/参考符号名称。
- 生产边界扫描 `afx|AcDb|AcGe|ObjectARX|CString|ads_|aced|XLDataBase|Excel|legacy_reference`
  无命中。
- 未新增、删除或移动 C++ 文件，无项目/过滤器/共享源清单更新需求。
- `git diff --check` 通过；本次维护文本保持 CRLF；差异未引入本机 XLSX/MDB/CSV 路径。

## 7. 剩余边界

- 本次不包含把 DLL 集成回 ObjectARX 宿主，因此未执行 AutoCAD 对话框冒烟。
- `ReconstructionOptions` 结构体布局已变化；调用方必须使用新头文件重新编译，并配套使用同版本导入库和 DLL。
