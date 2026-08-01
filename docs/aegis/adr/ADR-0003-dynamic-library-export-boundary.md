# ADR-0003 - 三入口动态库导出边界

Status: `recorded-from-work`
Date: `2026-08-01`

## Source Evidence

- 用户确认将静态库改为动态库，只导出三个主要函数。
- Debug/Release x64 DLL、导入库、测试和样例构建运行结果。
- `dumpbin /exports` 和调用方 `/dependents` 检查。

## Context

调用方通过 Visual Studio 项目引用消费本库。静态库把全部对象符号暴露给最终链接，
既不形成运行时 DLL，也无法限定公共二进制入口。目标是保持现有 C++ 函数签名，改为
可部署 DLL，并把导出表限制为三个业务入口。

## Decision

- `AlignmentReconstruct` 改为 MSVC x64 DLL。
- `include/alignment_reconstruct/api.h` 独占 `dllimport/dllexport` 规则。
- 只导出 `fit_point_cloud`、`reconstruct`、`calculate_adjustments`。
- 结果 `ok()` 和毫米换算成员内联，不增加导出项。
- Debug/Release 分别固定 `/MDd` 和 `/MD`。
- 白盒测试从共享内部源清单编译 `detail::*`，不得通过扩大 DLL 导出满足测试。

## Alternatives Considered

- 保留静态库：不能满足用户确认的动态部署目标。
- 使用 `.def` 导出修饰后的 C++ 名称：对编译器修饰规则脆弱，公共头也缺少
  `dllimport` 契约。
- 新增 C ABI：需要重新设计容器、字符串、结果和所有权，超出本次三入口边界。
- 导出全部内部测试符号：泄漏实现细节并破坏唯一公共契约，拒绝。

## Consequences

- 产物为 `AlignmentReconstruct.dll` 和同名导入库 `.lib`。
- Visual Studio 项目引用可以链接三个入口；运行时仍需部署 DLL。
- 公共 STL 类型继续采用 MSVC C++ ABI，调用方必须使用兼容工具集、x64 和匹配 CRT。
- 旧同名静态 `.lib` 的交付语义退役，不保留双构建 fallback。

## Compatibility Boundary

三个函数签名、公共字段、米制单位、诊断和数值行为保持不变；不承诺旧静态二进制、
跨编译器 ABI、`/MT` 调用方、C ABI、ARX/CAD 类型或未定义输入兼容。

## Retirement Impact

- 退役 `StaticLibrary` 配置和 `src/types.cpp` 外部便捷成员实现。
- `AlignmentReconstruct.lib` 现在只作为导入库。
- 内部算法 owner 不变且不导出；没有兼容适配器或第二个生产入口。

## Baseline Sync

- Needed: needed
- Target: `AGENTS.md`、`README.md`、修订基线和完整设计
- Action: update baseline
- Reason: 生产产物、运行时部署和 ABI 边界均发生持久变更。

## Evidence References

- `docs/aegis/specs/2026-08-01-dynamic-library-design.md`
- `docs/aegis/plans/2026-08-01-dynamic-library-implementation.md`
- `docs/aegis/work/2026-08-01-dynamic-library/90-evidence.md`
- `docs/verification/2026-08-01-dll-build-and-link.md`

## Boundary

This ADR is an advisory Aegis Method Pack record. It does not grant completion authority or replace project-authoritative architecture sources.
