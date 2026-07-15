# ADR-0001 - 独立纯 C++ 线路重构库及唯一几何所有者

Status: `recorded-from-work`
Date: `2026-07-13`

## Source Evidence

- 已实现并通过现场数值对照的 VS2026 C++17 静态库、测试和样例。
## Context

旧算法分散在 ObjectARX/MFC/UI/数据库混合类中；新库需要同时支持 XYZ 平纵重构以及完整平纵方案下的拨量和抬落道量，同时杜绝两套投影里程和生产依赖泄漏。

## Decision

以 include/alignment_reconstruct 作为唯一公共契约；HorizontalAlignment 独占平面建线与投影，VerticalAlignment 独占按里程求设计高程，AlignmentAdjustmentCalculator 只编排二者；HorizontalReconstructor 和 VerticalReconstructor 分别独占平纵重构；旧公式仅保留在 tests/legacy_reference。

## Alternatives Considered

- 直接链接或裁剪旧 RHA/BAS_CALC_FUN/CH_V_Cal 类；拒绝，因为会携带 ARX、MFC、全局状态、UI 和数组契约。
- 分别暴露拨量和抬落道接口并各自投影；拒绝，因为会产生两套 station 权威。
- 使用三维最短距离或折线近似；拒绝，因为不符合铁路拨量、抬落道和 Euler 缓和曲线语义。
## Consequences

- 生产库只依赖 C++17 标准库，可由普通 x64 C++ 程序静态链接。
- 公共参数统一使用米，右拨为正，抬落道为设计高程减实测高程；纵面范围必须覆盖平面里程。
- 新增算法必须进入既有唯一所有者，禁止在公共编排层复制线元、投影或竖曲线公式。
## Compatibility Boundary

数值兼容受支持的活跃旧算法，不兼容旧 ABI、ARX/CAD 实体、DWG、MDB、Excel、UI 或未定义输入。

## Retirement Impact

旧 ObjectARX 插件不在本目标中退役或修改；新项目内不保留生产 fallback，测试参考永不链接到静态库。

## Baseline Sync

- Needed: needed
- Target: docs/aegis/baseline/2026-07-13-revised-baseline.md
- Action: update baseline
- Reason: 修订基线仍写着源代码工程尚未创建，需要同步为实际所有者、公共契约和已验证边界。

## Evidence References

- docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md
- docs/aegis/plans/2026-07-13-alignment-reconstruct-implementation.md
- tests/test_field_reference.cpp
- tests/test_reconstruction.cpp
## Boundary

This ADR is an advisory Aegis Method Pack record. It does not grant completion authority or replace project-authoritative architecture sources.
