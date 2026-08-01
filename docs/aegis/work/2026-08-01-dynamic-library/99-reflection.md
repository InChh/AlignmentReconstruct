# AlignmentReconstruct 动态库改造 Reflection

日期：`2026-08-01`

## Goal

只导出三个主要 C++ 函数，并让 Visual Studio 项目引用真实消费 DLL。

## Architecture Reflection

- DLL 可见性只有一个 owner：`api.h`。
- 算法 owner、公共类型、单位和诊断均未改变。
- 白盒测试没有迫使生产 DLL 暴露 `detail::*`；共享内部源清单只属于构建/测试边界。
- 静态库配置和额外外部成员实现被删除，没有并行 fallback。

## Debugging Reflection

- Goal：恢复 DLL 下的白盒测试并维持三项导出。
- DeeperCause：no；根因停在测试产物拓扑与 DLL 导入库语义的系统边界。
- Evidence：第一次 19 个内部符号 LNK2019/LNK2001；共享源清单后同一 Rebuild 通过。
- Risk/Unknown：三个可选外部夹具未启用；DLL 的公共 STL 类型仍要求调用方保持相同
  MSVC ABI、x64 平台及匹配的 `/MDd` 或 `/MD` 运行库。
- Decision：继续到目标验证，未扩大导出表。

## Completion State

隔离副本和目标目录均为 `verified`；未执行提交、推送或第三方应用集成测试。
