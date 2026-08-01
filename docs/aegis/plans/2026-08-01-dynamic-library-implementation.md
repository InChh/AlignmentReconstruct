# AlignmentReconstruct 动态库改造实施计划

## Goal

把现有 x64 C++17 静态库改为只导出三个主要函数的 MSVC DLL，同时保持现有
函数签名、数值行为和项目引用用法。

## Architecture

`include/alignment_reconstruct/api.h` 是唯一 DLL 可见性契约；三个公共自由函数使用
导出宏，所有结果便捷成员保持头文件内联。内部 owner 和算法文件不导出。

## Tech Stack

Visual Studio 2026、MSVC v145、C++17、x64、MSBuild、Git Bash。

## Baseline/Authority Refs

- 用户于 2026-08-01 确认的 MSVC C++ DLL 方案
- `AGENTS.md`
- `README.md`
- `docs/aegis/baseline/2026-07-13-revised-baseline.md`
- `docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md`
- `docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md`
- `docs/aegis/specs/2026-08-01-dynamic-library-design.md`

## Compatibility Boundary

源代码级三个入口、公共类型、米制单位、诊断和数值行为保持不变。二进制交付从
静态库变为 DLL 加导入库；旧静态 `.lib` 不兼容新的部署方式。调用方必须使用兼容
MSVC C++ ABI、x64 和匹配的 `/MDd` 或 `/MD`。

## TDD Route

- Mode: off
- Decision: skipped
- Strict authority: not applicable
- Test posture: post-change regression
- Reason: 用户未要求 strict TDD；变更核心是构建和 ABI 边界，现有测试/样例已覆盖
  三个公共入口。
- Verification: 全量 Rebuild、测试、样例、导出表和导入库检查。

## Requirement Ready Check

- Requirement source refs: 用户本轮请求及已确认设计
- Goals and scope refs: 仅三个主要导出函数，现有 C++ API 不变
- User / scenario refs: Visual Studio 调用方通过项目引用消费 DLL
- Requirement item refs: 动态库、三个导出、ProjectReference、匹配运行库
- Acceptance / verification criteria refs: 设计规格第 5 节
- Open blocker questions: 无
- Decision: ready

## Change Necessity

- User-visible need: 项目引用应生成并消费动态库，三个主要入口可从 DLL 导入。
- No-change / non-code option: 继续静态库不能满足动态部署要求。
- Why code change is necessary: 声明必须携带稳定的 dllimport/dllexport 契约；非导出
  成员必须内联，才能将导出表限定为三个入口。
- Minimum change boundary: 公共可见性头、三个入口声明、结果便捷成员及 VS 工程配置。
- Decision: code-change

## Existence Check

- Proposed new surface: `api.h` DLL 可见性宏。
- Existing owner / reuse candidate: `include/alignment_reconstruct/` 公共契约目录。
- Why existing surface is insufficient: 当前没有统一导入/导出定义。
- Creation proof: 三个头文件必须共享完全相同的生产者/消费者条件。
- Entropy / retirement impact: 新增一个小型公共构建头；删除 `src/types.cpp` 外部成员实现。
- Decision: add-with-proof

## Architecture Integrity Lens

- Invariant: 仅三个公共自由函数跨 DLL 导出边界。
- Canonical owner / contract: `api.h` 和三个现有公共入口头文件。
- Responsibility overlap: 无；算法 owner 不获得导出职责。
- Higher-level simplification: 便捷成员内联，不再制造额外导出符号。
- Retirement / falsifier: `src/types.cpp` 和静态库配置退役；导出表出现第四个项目即失败。
- Verdict: 保持单一可见性 owner 并继续实施。

## Tasks

### Task 1：建立 DLL 公共可见性边界

Files: 新增 `include/alignment_reconstruct/api.h`；修改 `point_cloud_fit.h`、
`reconstruction.h`、`adjustment.h`、`types.h`；删除 `src/types.cpp`。

Steps:

1. 定义 `ALIGNMENT_RECONSTRUCT_API`：生产工程宏为 `dllexport`，调用方为
   `dllimport`。
2. 只在三个公共自由函数声明前添加该宏。
3. 将三个 `ok()` 和两个毫米换算函数改为类内定义。
4. 删除不再需要的 `src/types.cpp`。
5. 扫描公共头，确认没有第四处 `ALIGNMENT_RECONSTRUCT_API`。

### Task 2：切换 Visual Studio 产物并保持白盒测试隔离

Files: 新增 `AlignmentReconstruct.InternalSources.props`；修改
`AlignmentReconstruct.vcxproj`、`AlignmentReconstruct.vcxproj.filters`、测试和样例
工程。

Steps:

1. 两个配置改为 `DynamicLibrary`。
2. DLL 工程定义 `ALIGNMENT_RECONSTRUCT_EXPORTS`。
3. Debug/Release 分别显式选择 `MultiThreadedDebugDLL` / `MultiThreadedDLL`。
4. 工程加入 `api.h`，移除 `src/types.cpp`。
5. 把内部实现源文件清单移入一个共享 `.props`；DLL 和白盒测试均编译同一源文件，
   但测试继续通过项目引用导入三个公共入口，绝不导出 `detail::*`。
6. 保持目标名和 OutDir 不变，使 DLL、导入库、测试和样例位于同一配置目录。

### Task 3：同步契约和架构记录

Files: `README.md`、`AGENTS.md`、修订基线、完整设计、ADR-0001、新增 ADR-0003、
`docs/aegis/INDEX.md`。

Steps:

1. 将静态链接说明替换为 DLL、导入库和运行时部署说明。
2. 记录旧静态库路径已退役，不保留双构建 owner。
3. 明确项目引用必须链接库依赖项并让 DLL 可被运行时找到。
4. 保持算法、单位和 C++ ABI 风险说明不变。

### Task 4：验证动态链接闭环

Files: 不新增生产实现；使用现有测试和样例作为调用方。

Steps:

1. Debug x64 Rebuild，运行全量测试和样例；白盒 detail 测试由测试目标内的共享
   内部源清单满足，公共入口由 DLL 导入库满足。
2. Release x64 Rebuild，运行全量测试和样例。
3. 对两份 DLL 执行 `dumpbin /exports`，逐项核对只有三个业务导出。
4. 对测试/样例执行依赖检查，确认它们依赖 `AlignmentReconstruct.dll`。
5. 运行生产依赖扫描、绝对路径扫描、CRLF 检查和 `git diff --check`。

## Plan Pressure Test

- Owner / contract / retirement: 可见性由单一头文件拥有，静态配置和 types.cpp 退役。
- Architecture integrity / higher-level path: 不导出内部类，不增加适配层或第二套接口。
- Verification scope: 构建、运行、导出、导入和文档均有检查。
- Task executability: 文件和命令边界明确。
- Pressure result: proceed

## Risks

- 公共 STL 类型要求调用方和 DLL 使用兼容 MSVC ABI 与同类 CRT。
- 调用方运行目录缺少 DLL 时会出现启动阶段模块加载错误。
- 同名 `.lib` 现在是导入库，不能再当作旧静态实现分发。

## Retirement

- 退役：静态库配置、旧静态 `.lib` 交付语义、`src/types.cpp`。
- 保留：三个公共 C++ 函数签名、所有算法 owner、测试和样例。
- 不新增静态兼容 fallback；如未来需要 C ABI，应作为独立设计处理。
