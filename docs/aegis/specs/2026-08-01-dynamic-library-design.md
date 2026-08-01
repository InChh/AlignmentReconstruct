# AlignmentReconstruct 动态库设计规格

日期：`2026-08-01`
状态：`用户已确认`
架构审查：`需要`

## 1. 目标

将 `AlignmentReconstruct` 从 x64 C++17 静态库改为 MSVC 动态库，保持既有数值、
单位、诊断和 C++ 函数签名不变，只导出三个公共入口：

1. `fit_point_cloud`
2. `reconstruct`
3. `calculate_adjustments`

## 2. 采用方案

新增公共 `api.h`，由 `ALIGNMENT_RECONSTRUCT_API` 统一控制
`__declspec(dllexport)` / `__declspec(dllimport)`。DLL 工程定义
`ALIGNMENT_RECONSTRUCT_EXPORTS`；所有调用工程不定义该宏。

三个公共入口分别在现有头文件声明处使用该宏。结果类型的 `ok()` 以及
`PointAdjustment` 的毫米换算函数改为头文件内联，避免它们成为额外 DLL 导出项；
原 `src/types.cpp` 因而退役。

## 3. 构建和运行时契约

- `AlignmentReconstruct.vcxproj` 的 `ConfigurationType` 改为
  `DynamicLibrary`。
- Debug 明确使用 `/MDd`，Release 明确使用 `/MD`。
- 产物为 `AlignmentReconstruct.dll` 和同名导入库
  `AlignmentReconstruct.lib`。
- 测试和样例继续通过 `ProjectReference` 使用库，并作为真实 DLL 调用方编译。
- 白盒测试所需的 `detail::*` 实现由测试目标从共享内部源清单直接编译；这些符号不
  进入 DLL 导出表。公共测试仍通过项目引用调用三个 DLL 入口。
- 调用方必须使用 x64、兼容的 MSVC 工具集、相同 Debug/Release 及 DLL 运行库配置。
- DLL 必须位于调用方可执行模块同目录或 Windows DLL 搜索路径中。

公共 API 继续传递 `std::vector`、`std::string` 和按值结果，因此这是 MSVC C++
ABI，不新增 C ABI，也不承诺跨编译器或不兼容运行库边界。

## 4. 不变契约

- `Point3D`、里程、半径、高程、拨量、抬落道量和点云采样间隔继续使用米。
- 三个入口的参数、返回类型、默认选项和诊断行为不变。
- 所有内部 owner、算法依赖方向及现场数值容差不变。
- 不引入 ARX、MFC、COM、PCL、Eigen 或其他生产依赖。

## 5. 验收标准

1. Debug/Release x64 解决方案全量 Rebuild 成功，测试和样例退出码均为 0。
2. `dumpbin /exports AlignmentReconstruct.dll` 只显示上述三个公共 C++ 入口。
3. 测试和样例目标不定义 `ALIGNMENT_RECONSTRUCT_EXPORTS`，通过导入库链接并在
   DLL 存在时运行成功。
4. `AlignmentReconstruct.lib` 被确认是导入库；旧静态库产物不再作为支持路径。
5. README、AGENTS、基线、设计和 ADR 同步动态库及部署约束。

## 6. 非目标

- 不保留静态库并行构建配置。
- 不增加 C 接口、COM 接口或版本化二进制兼容层。
- 不改变三个入口以外的业务能力或数值实现。
