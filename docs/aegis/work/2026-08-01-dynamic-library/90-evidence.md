# AlignmentReconstruct 动态库改造 Evidence

日期：`2026-08-01`

## Delivered Scope

- 新增 `api.h` 并只标记三个公共自由函数。
- 工程由 `StaticLibrary` 切换为 `DynamicLibrary`。
- Debug `/MDd`、Release `/MD`。
- 5 个结果/便捷成员内联，`src/types.cpp` 退役。
- 共享内部源清单让白盒测试保持覆盖而不导出 `detail::*`。

## Isolated Build Evidence

| 检查 | 结果 |
| --- | --- |
| 修改前 Debug/Release 基线 | 两个配置 Rebuild、48 项测试和样例均成功；公开副本跳过 8 个私有/可选夹具 |
| 修改后 Debug x64 Rebuild | DLL、导入库、测试、样例生成成功 |
| 修改后 Debug 测试 | 48 项，失败 0，跳过 8 |
| 修改后 Debug 样例 | 退出 0，三个入口串联成功 |
| 修改后 Release x64 Rebuild | DLL、导入库、测试、样例生成成功 |
| 修改后 Release 测试 | 48 项，失败 0，跳过 8 |
| 修改后 Release 样例 | 退出 0，结果与 Debug 一致 |

## Export and Runtime Evidence

- Debug/Release `dumpbin /exports AlignmentReconstruct.dll` 均显示
  `3 number of functions` 和 `3 number of names`。
- 三个名称分别为 `calculate_adjustments`、`fit_point_cloud`、`reconstruct`。
- 测试和样例的 `/dependents` 均包含 `AlignmentReconstruct.dll`。
- Release 导入库成员仅包含三个业务符号、对应 `__imp_` 符号及标准导入描述符。

## Diagnostic Closure

第一次 Debug DLL 构建时，DLL 和样例成功，但白盒测试因直接调用未导出的
`detail::*` 出现 19 个未解析符号。根因是原测试拓扑依赖静态库完整对象符号。
修复没有导出内部符号，而是由 DLL 和测试共同引用一个内部源清单；第二次 Debug
构建消除了原始 19 个链接错误，全部测试恢复通过。

## Target Repository Evidence

| 检查 | 结果 |
| --- | --- |
| Debug x64 Rebuild | 退出 0，0 警告、0 错误 |
| Debug 测试/样例 | 48 项测试，失败 0，跳过 3；样例退出 0 |
| Release x64 Rebuild | 退出 0，0 警告、0 错误 |
| Release 测试/样例 | 48 项测试，失败 0，跳过 3；样例退出 0 |
| Debug/Release 导出表 | 各 3 个函数、3 个名称，仅三个公共入口 |
| Debug/Release 运行时依赖 | 测试和样例均依赖 `AlignmentReconstruct.dll` |

补丁已同步到 `D:\Programming\C++\Projects\既有线\AlignmentReconstruct`，目标目录
验证状态为 `verified`。
