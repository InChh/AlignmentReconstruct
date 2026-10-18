# 私有现场参考测试夹具

生产静态库不读取 Excel、Access 或 CSV。本目录只公开夹具说明，不公开任何现场测点、线路方案或拨量数据。`tests/fixtures/*.csv` 已加入 `.gitignore`，不得提交、推送、上传到发布包或粘贴到公开问题中。

获得数据授权的维护者可以在本地放置以下完整文件：

- `k1413_k1462_points.csv`
- `k1413_k1462_horizontal.csv`
- `k1413_k1462_offsets.csv`

三份文件全部存在时，现场回归测试自动运行；全部缺失时相关测试明确显示 `[SKIP]`；只存在部分文件时测试失败，以避免误用不完整基线。公开克隆不需要这些文件即可构建并运行其余确定性测试。

## 稀疏无宽度与带宽度点云对照

`point_cloud_fit_handles_the_optional_sparse_and_wide_reference_pair` 通过两个环境变量读取私有米制 CSV；均无表头，每行三列 `x,y,z`：

- `ALIGNMENT_RECONSTRUCT_EXTERNAL_SPARSE_CENTERLINE_CSV`：按线路方向排列的无宽度中心线参考。
- `ALIGNMENT_RECONSTRUCT_EXTERNAL_WIDE_CLOUD_CSV`：相同线路范围的带宽度散点，行序不限。

测试对两组数据分别调用 `fit_point_cloud(..., 5.0)`，核查反转行序后逐坐标完全一致，并以无宽度参考折线比较平面最大差 / RMS 和同位置高程最大差 / RMS。当前门槛用于本次私有数据回归，不是通用工程验收标准；具体数值与本轮证据见 `docs/verification/2026-10-18-point-cloud-sparse-and-wide.md`。未提供任一变量时明确跳过，原始数据和转换后的 CSV 保持忽略，不提交。

## 坐标和单位映射

- 公共 `x` = 旧数据 `N` / `纬距`。
- 公共 `y` = 旧数据 `E` / `经距`。
- 半径和缓和曲线长使用米。
- 原拨量字段使用毫米；`horizontal_offset_m` 是除以 1000 后的米值。
- 沿里程增加方向观察，右侧拨量为正。

## 覆盖边界

参考工作簿没有 z/高程列，因此这些夹具只作为平面重构、投影里程和拨量的权威对照。纵断面重构与抬落道量由确定性三维数据和测试专用旧公式验证。

## 重新生成

确需重新生成时，由有权限的维护者把私有原始文件路径作为命令参数传给 `tools/inspect_reference_workbook.mjs` 和 `tools/export_reference_data.ps1`。不要把原始路径、数据行或数据指纹写回公开源码和文档，生成后再次确认 `git status` 不列出任何 CSV。
