# AlignmentReconstruct 动态库改造 Checkpoint

日期：`2026-08-01`

## TodoCheckpointDraft

- Completed：设计确认；规格和计划落盘；隔离副本和目标目录 Debug/Release 验证通过。
- Active：无。
- Pending：无；提交和推送不在本次授权范围内。
- Blockers：无。
- Next：由用户决定是否提交和推送。

## Baseline Evidence

- Debug/Release x64 Rebuild 均退出 0。
- 每个配置执行 48 项测试，失败 0；公开隔离副本因无私有/可选夹具跳过 8 项。
- 两个样例退出 0。

## Debug DLL Diagnostic

- DLL 和导入库已成功生成。
- 样例通过项目引用链接成功。
- 测试链接失败的 19 个符号全部属于 `detail::*` 白盒接口；它们不应进入用户要求的
  三项 DLL 导出表。
- Root Cause：静态库时代测试可从完整 `.lib` 解析内部符号；DLL 导入库只包含显式
  导出，原测试拓扑隐含依赖了静态产物形状。
- PatchShape：测试/构建边界修正，不扩大生产导出。
- CanonicalOwner：共享内部源清单和测试工程引用。
- Decision：测试目标直接编译同一份内部源清单，公共入口仍从 DLL 导入。

## DriftCheckDraft

- Intent：aligned。
- Scope：仅 ABI/构建/文档。
- Compatibility：数值 API 和米制契约不变。
- New owner/fallback：仅新增必要的 `api.h` 可见性 owner；无 fallback。
- Retirement：静态配置和 `src/types.cpp` 保持明确退役。
- Execution Readiness View：aligned。
- Decision：`continue`。
