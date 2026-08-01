# AlignmentReconstruct DLL 构建与链接验证

日期：`2026-08-01`

## 验证目标

- 产物从静态库切换为 `AlignmentReconstruct.dll` 和导入库 `.lib`。
- DLL 只导出三个公共入口。
- Visual Studio 项目引用调用方能链接并加载 DLL。
- 原数值测试和样例不回归。

## 隔离副本结果

- Debug/Release x64 全量 Rebuild 均退出 0。
- 每个配置运行 48 项测试，失败 0；公开副本因没有私有/可选外部夹具跳过 8 项。
- 每个配置的三入口样例退出 0。
- Debug/Release DLL 导出表均只有三个名称：
  `calculate_adjustments`、`fit_point_cloud`、`reconstruct`。
- 测试和样例的依赖表均包含 `AlignmentReconstruct.dll`。
- Release `AlignmentReconstruct.lib` 是导入库而不是旧静态实现库。

## 目标目录结果

- Debug/Release x64 全量 Rebuild 均退出 0，0 警告、0 错误。
- 每个配置运行 48 项测试，失败 0，跳过 3 项未启用的可选外部夹具。
- 每个配置的样例退出 0。
- Debug/Release DLL 均为 3 个函数、3 个名称，且名称精确对应三个公共入口。
- Debug/Release 测试和样例的依赖表均包含 `AlignmentReconstruct.dll`。
