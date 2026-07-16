# AlignmentReconstruct 实施计划

日期：`2026-07-13`

## Goal

依据 `docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md`，在 `D:\Programming\C++\Projects\既有线\AlignmentReconstruct` 完成独立 VS2026、C++17、x64 静态库、测试和示例，提供：

1. XYZ 测点平面与纵断面重构；
2. XYZ 测点相对完整平纵方案的拨量和抬落道量联合计算。

## Architecture

- 公共契约：`include/alignment_reconstruct`。
- 数学基础：`src/geometry`。
- 平面线路建线/求点/投影：`HorizontalAlignment`。
- 纵断面按里程求高程：`VerticalAlignment`。
- 联合计算：`AlignmentAdjustmentCalculator`，只编排，不复制几何公式。
- 平面重构：`HorizontalReconstructor`。
- 纵面重构：`VerticalReconstructor`。
- 旧行为参考：仅限 `tests/legacy_reference`；生产项目不编译旧代码。

## Tech Stack

- Visual Studio Community 2026：`D:\Programs\Microsoft Visual Studio\2026`
- MSBuild：`D:\Programs\Microsoft Visual Studio\2026\MSBuild\Current\Bin\amd64\MSBuild.exe`
- Platform Toolset：`v145`
- Language Standard：`stdcpp17`
- Platform：`x64`
- Production dependencies：C++17 standard library only
- Tests：自包含控制台测试运行器，无第三方测试框架

## Baseline / Authority Refs

- `docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md`
- `docs/aegis/baseline/2026-07-13-revised-baseline.md`
- 设计规格第 3 节列出的 high-value legacy seams。
- 旧仓库根：`D:\Programming\C++\Projects\既有线\主控`。
- 用户提供的私有现场水平测点、平面方案和拨量权威结果；原始路径和数据不进入公开仓库。

## Compatibility Boundary

- 新库与受支持的活跃旧算法做数值行为兼容，不做旧 ABI、CAD 实体、DWG、MDB 或 UI 兼容。
- 公共主单位为米；现场 MDB 拨量字段为毫米，夹具导入时显式换算。
- 沿里程增加方向右拨为正、左拨为负；抬落道量为 `design_elevation - measured_z`，正值抬道、负值落道。
- 每个测点只有一个由平面线路产生的投影 station，拨量与纵面高程共享该 station。
- 旧工程不修改、不重定向、不删除。

## Verification

```bash
"D:/Programs/Microsoft Visual Studio/2026/MSBuild/Current/Bin/amd64/MSBuild.exe" AlignmentReconstruct.sln /m /t:Build /p:Configuration=Debug /p:Platform=x64
./build/Debug/AlignmentReconstruct.Tests.exe
"D:/Programs/Microsoft Visual Studio/2026/MSBuild/Current/Bin/amd64/MSBuild.exe" AlignmentReconstruct.sln /m /t:Rebuild /p:Configuration=Release /p:Platform=x64
./build/Release/AlignmentReconstruct.Tests.exe
rg -n -i "afx|AcDb|AcGe|ObjectARX|CString|ads_|aced|XLDataBase|Excel" include src AlignmentReconstruct.vcxproj
```

成功标准：两种配置构建和测试退出码均为 0；依赖扫描无生产代码命中；现场 1569 点、22 个平面控制点和拨量结果在规定容差内通过；纵断面确定性和闭环测试通过。

## Requirement Ready Check

- Requirement source refs：用户需求、完整设计规格、修订基线。
- Goals and scope refs：本计划 Goal、Compatibility Boundary。
- User / scenario refs：线路重构；基于完整平纵方案计算既有测点调整量。
- Requirement item refs：设计规格第 1、6、7、11、12、15、16 节。
- Acceptance / verification refs：本计划 Verification 和各任务验证命令。
- Open blocker questions：无。现场文件只有 N/E 两列，不含 z，因此它负责水平真实数据对照；纵面和抬落道使用确定性三维夹具验证，不降低纵面实现范围。
- Decision：`ready`

## BaselineUsageDraft

- Required baseline refs：完整设计、修订基线、边界化旧代码、现场 XLSX/MDB。
- Acknowledged before plan refs：上述文档与现场数据结构均已读取。
- Cited in plan refs：本计划 Baseline / Authority Refs 和各任务 Legacy refs。
- Missing refs：没有现场纵断面三维基准数据；由确定性生成夹具覆盖，不作为实现阻塞。
- Decision：`continue`

## Change Necessity

- User-visible need：可链接的静态库、两个真实公共能力和可复现实测验证。
- No-change / non-code option：文档不能产生 `.lib`，也不能证明数值行为。
- Why code change is necessary：必须建立新生产所有者并迁移纯算法。
- Minimum change boundary：新项目工程文件、公共头文件、`src`、`tests`、`samples`、`tools` 和必要文档；旧仓库只读。
- Decision：`code-change`

## Existence Check

- Proposed new surfaces：公共契约、五个算法所有者、测试参考、现场数据抽取工具。
- Existing reuse candidate：旧算法函数是行为来源，但旧类依赖 ARX/MFC，不能成为独立静态库所有者。
- Creation proof：用户明确要求独立无 ARX/MFC 静态库；各所有者对应互不重叠职责。
- Entropy / retirement impact：不增加生产兼容层；测试参考永不被生产链接；旧生产代码本目标不退役。
- Decision：`add-with-proof`

## Architecture Integrity Lens

- Invariant：平面投影里程唯一；拨量和抬落道量共享 station。
- Canonical owners：`HorizontalAlignment`、`VerticalAlignment`、`AlignmentAdjustmentCalculator`、两个 reconstructor。
- Responsibility overlap：禁止 adjustment/reconstructor 各自复制线元或竖曲线求值。
- Higher-level simplification：两个公共自由函数封装内部所有者，调用方不管理中间状态。
- Retirement/falsifier：若生产代码引用 `legacy_reference`、ARX/MFC 或第二套投影公式，架构判定失败。
- Verdict：`aligned`

## Plan-Time Complexity Check

- Artifact class：核心数值源文件与维护测试。
- Current pressure：旧实现分散在 5k-30k 行混合职责文件中。
- Projected pressure：按模块拆分后单个源文件目标小于 800 行；超过时先拆所有者。
- Budget result：`within-budget`，但平面重构和线元投影为高压模块。
- Recommendation：新 owner 文件；不创建通用 `algorithm.cpp` 大杂烩。

## Execution Readiness View

- Intent Lock：完整实现平面、纵面、拨量和抬落道量，不用多段线或硬编码替代。
- Scope Fence：新项目内实现；现场数据只生成测试夹具；旧仓库只读。
- Baseline Lock：完整设计、修订基线和活跃旧代码。
- Approved Behavior：两个公共函数及设计规格全部字段、单位、符号和诊断。
- Owner / Contract Constraints：每项算法只有一个生产 owner；测试参考隔离。
- Compatibility Boundary：逐字段数值对照，不追求旧 ABI/CAD/MDB。
- Retirement Boundary：不退役旧插件；禁止新生产 fallback。
- Task Batches：工程/契约 → 数据基线 → 平面评估 → 纵面评估 → 联合计算 → 平面重构 → 纵面重构 → 现场对照 → 样例/验证。
- Test Obligations：RED/GREEN 单元测试、现场对照、闭环、错误诊断、依赖扫描、Debug/Release。
- Review Gates：每批构建测试；完成前架构和需求逐项审计。
- Drift / Rewind Rules：出现第二套 station、生产 Excel/MDB 依赖、非旧逻辑替代或真实对照明显漂移时停止并回读源代码。
- Evidence Required Before Completion：设计规格第 15 节全部证据。
- Advisory Boundary：本视图只指导执行，不授予完成状态。

## Task 1：建立现场验证夹具

**Files**

- Modify: `tools/inspect_reference_workbook.mjs`
- Modify: `tools/inspect_reference_mdb.ps1`
- Create: `tools/export_reference_data.ps1`
- Create: `tests/fixtures/k1413_k1462_points.csv`
- Create: `tests/fixtures/k1413_k1462_horizontal.csv`
- Create: `tests/fixtures/k1413_k1462_offsets.csv`
- Create: `tests/fixtures/README.md`

**Why**：把外部 XLSX/MDB 变成稳定、可审计、无需生产依赖的测试输入和期望值。

**Impact/Compatibility**：CSV 保留原输入顺序和 MDB 数值；N 映射到 API `x`，E 映射到 API `y`；拨量 mm 显式除以 1000 形成主单位。

**Verification**

```bash
wc -l tests/fixtures/k1413_k1462_points.csv tests/fixtures/k1413_k1462_horizontal.csv tests/fixtures/k1413_k1462_offsets.csv
```

期望：点和拨量文件各 1570 行（1 表头 + 1569 数据），平面方案 23 行（1 表头 + 22 控制点）；首尾行与源文件一致。

- [x] 编写导出工具，限定线路名 `既有基线` 并按 MDB `ID` 排序。
- [x] 首次导出并核对行数、表头、首尾记录和 SHA-256。
- [x] 将外部绝对路径只写入 fixture README，不写入生产代码。
- [x] 建立 fixture 读取器测试，验证有限数和严格行数（Task 2 测试工程建立后执行）。
- [x] 记录检查点；目标目录当前不是 Git worktree，因此不执行隐式 `git init` 或 commit。

## Task 2：创建 VS2026 解决方案和公共契约

**Files**

- Create: `AlignmentReconstruct.sln`
- Create: `AlignmentReconstruct.vcxproj`
- Create: `AlignmentReconstruct.vcxproj.filters`
- Create: `tests/AlignmentReconstruct.Tests.vcxproj`
- Create: `tests/AlignmentReconstruct.Tests.vcxproj.filters`
- Create: `samples/AlignmentReconstruct.Sample.vcxproj`
- Create: `samples/AlignmentReconstruct.Sample.vcxproj.filters`
- Create: `include/alignment_reconstruct/types.h`
- Create: `include/alignment_reconstruct/reconstruction.h`
- Create: `include/alignment_reconstruct/adjustment.h`
- Create: `tests/test_main.cpp`
- Create: `tests/test_contract.cpp`

**Why**：固定可链接边界、工具集、单位、符号、结果和诊断契约。

**Impact/Compatibility**：公共头文件仅标准库；结果顺序与输入一致；`VerticalControlPoint` 必须包含 `vertical_curve_radius_m`。

**Verification**：用 Debug x64 MSBuild 构建；初始契约测试验证默认值、毫米换算、标准布局和无 CAD 类型。

- [x] 先写契约编译测试，引用所有公共类型和两个函数签名。
- [x] 运行构建并确认因项目/头文件缺失而 RED。
- [x] 建立三个项目和公共头文件，使契约测试编译通过。
- [x] 运行 Debug x64 测试并执行生产依赖扫描。
- [x] 更新工作检查点，记录构建命令和输出。

## Task 3：迁移纯几何和 typed 平面线元

**Files**

- Create: `src/geometry/vector_math.h`
- Create: `src/geometry/vector_math.cpp`
- Create: `src/geometry/line_fit.h`
- Create: `src/geometry/line_fit.cpp`
- Create: `src/horizontal/horizontal_element.h`
- Create: `src/horizontal/horizontal_element.cpp`
- Create: `tests/test_geometry.cpp`
- Create: `tests/legacy_reference/legacy_horizontal_geometry.h`
- Create: `tests/legacy_reference/legacy_horizontal_geometry.cpp`

**Legacy refs**：`BAS_CALC_FUN.cpp:767-985`、`:2279-2350`；`CH_V_Cal.cpp:628-673`、`:936-1059`、`:1586-1664`。

**Why**：建立不依赖十列数组的直线、圆、Euler 缓和曲线数值基础。

**Verification**：人工解析解 + legacy reference 双重对照；直线/圆坐标误差 `<=1e-9 m`，缓和曲线在旧公式采样点误差 `<=1e-7 m`。

- [x] 覆盖 line/circle/spiral point-at-length 和线拟合测试。
- [x] 逐函数迁移公式，保持旧角度和左右转约定。
- [x] 比较人工解析解、旧公式、连接点位置、切向和曲率连续性。
- [x] 达到直线/圆 `1e-9 m`、缓和曲线 `1e-7 m` 公式对照精度。
- [x] 扫描生产代码不含 `AcGe`/裸线元数组。

## Task 4：实现 HorizontalAlignment 与拨量

**Files**

- Create: `src/horizontal/horizontal_alignment.h`
- Create: `src/horizontal/horizontal_alignment.cpp`
- Create: `tests/test_horizontal_alignment.cpp`

**Legacy refs**：`RHA.cpp:3348-4003`；`BAS_CALC_FUN.cpp:987-1165`、`:2445-2897`、`:4768`；`RHA.cpp:26041-26188`。

**Why**：为给定交点方案提供唯一的平面建线、按里程求点、最近投影和右正左负拨量所有者。

**Verification**：直线、圆、等/不等缓长、多交点、线元接缝、端点外投影；现场 MDB 22 个交点建线后对 1569 点比较 station 和 offset。

- [x] 覆盖 typed elements、端点、线元接缝和左右符号测试。
- [x] 迁移 `getpxy_DATA`、`xlpoint_pz`、`PROJ_ML` 的行为到 typed owner。
- [x] 运行合成测试和现场 1569 点/22 交点对照。
- [x] 记录 station、拨量最大/RMS 误差和最大误差点。
- [x] 对数值漂移回读相关旧函数及调用者并修正，不使用 fixture 专用分支。

## Task 5：实现 VerticalAlignment

**Files**

- Create: `src/vertical/vertical_alignment.h`
- Create: `src/vertical/vertical_alignment.cpp`
- Create: `tests/test_vertical_alignment.cpp`
- Create: `tests/legacy_reference/legacy_vertical_geometry.h`
- Create: `tests/legacy_reference/legacy_vertical_geometry.cpp`

**Legacy refs**：`BAS_CALC_FUN.cpp:10269-10576`；`DrawSlope.cpp:6337-6367`。

**Why**：建立纵断面方案验证、直坡段/竖曲线求设计高程的唯一所有者。

**Verification**：单坡、多坡、凸/凹竖曲线、半径 0、控制点、曲线起终点、重叠拒绝；与最小旧公式参考逐点比较。

- [x] 覆盖圆竖曲线两侧、切点、凸/凹曲线和半径 0 折坡测试。
- [x] 迁移 `MLToH_Bas` 和 `VerticleArc` 的受支持物理半径分支。
- [x] 用独立最小旧公式逐点对照设计高程。
- [x] 验证高度和切向连续并拒绝竖曲线重叠。
- [x] 扫描确认生产代码没有特殊 `<100` 旧编码 fallback。

## Task 6：实现联合拨量和抬落道量 API

**Files**

- Create: `src/alignment/adjustment_calculator.h`
- Create: `src/alignment/adjustment_calculator.cpp`
- Create: `src/adjustment.cpp`
- Create: `tests/test_adjustment.cpp`

**Legacy refs**：`ZDMSJ.cpp:10949-10968`；`DrawSlope.cpp:3306-3376`、`:6337-6367`。

**Why**：实现第二公共能力，保证 station 单一来源。

**Verification**：每点 `station` 与 horizontal projection 完全相同；`lift_drop_m == design_elevation_m - z`；正抬负落；输入顺序不变。

- [x] 覆盖同时有平曲线和竖曲线的联合结果测试。
- [x] 实现只编排两个 alignment owner 的 calculator 和公共函数。
- [x] 验证 station 单一来源、输入顺序、米/mm 换算和正负号。
- [x] 运行现场水平方案测试并确认加入任意 z 不改变 station/拨量。
- [x] 用 XYZ 设计线闭环验证抬落道量。

## Task 7：迁移平面重构

**Files**

- Create: `src/horizontal/horizontal_reconstructor.h`
- Create: `src/horizontal/horizontal_reconstructor.cpp`
- Create: `src/horizontal/curvature_classifier.h`
- Create: `src/horizontal/curvature_classifier.cpp`
- Create: `src/horizontal/alignment_optimizer.h`
- Create: `src/horizontal/alignment_optimizer.cpp`
- Create: `tests/test_horizontal_reconstruction.cpp`

**Legacy refs**：设计规格第 3.2 节全链；需要时扩展阅读 `BCTTP`、`BCTPP`、`BAS_CALC_FUN` 和 `RHA` 的被调函数。

**Why**：实现第一公共能力的平面部分，不以现场方案硬编码替代重构。

**Verification**：合成直线/圆/缓和/多交点闭环；现场 1569 点重构得到 22 个控制点，并逐点比较坐标、R、l1、l2 和重算拨量。

- [x] 覆盖曲率、分类、直线段、圆拟合、交点、缓长和优化阶段。
- [x] 按活跃调用顺序迁移并用显式 options 替代全局值。
- [x] 运行合成闭环和现场 1569 点全链。
- [x] 恢复 22 个控制点并记录交点、半径、进/出口缓长最大误差及索引。
- [x] 对不一致处扩展阅读真实调用者和参数来源；未加入 fixture 专用分支。

### Task 7 继续迁移（2026-07-16）

上述勾选项只证明当时的近似实现恢复了 22 个控制点，不证明已完成活跃旧算法迁移。现场对照仍存在交点 `0.232059 m`、半径 `3.16358 m`、进口缓长 `5.92673 m`和出口缓长 `3.8956 m` 的最大误差，因此 Task 7 重新打开。

**唯一算法权威**：从“全线坐标拟合”对话框的两个活跃按钮逐层追踪。`CQXBoLiangCalDlg::OnBnClickedButtonZdfd` 当前执行 `Sort` → `JMSurPtLC` → `CalCurvature` → `MarkProperty` → `IDdentifyLineOrCurve` → `Dynamic_Adjust_Property_SetAdjust(5)` → `DealFanxiangQX` → `CalFJD` → `CheckCurve` → 空的 `Checkline` → `Dynamic_Adjust_Property_SetAdjust(6)`；随后 `CQXBoLiangCalDlg::OnBnClickedButtonNhht` 执行 `FormLineSeg`，有曲线时执行 `IDSpiral` → `SwingAlgorithm`，最后执行 `AllQXNH2`。平面数值实现位于 `CeDianNiHe.cpp`；同名 `_SW`、`New`、`opti`、`SwingAlgorithm_SinglePI`、注释代码和未被两个按钮调用的历史实现都不是本任务算法权威。

**新验收条件**：

- [x] 建立严格 RED：22 个控制点数量和类型必须完全一致；交点坐标、半径、进口缓长、出口缓长分别统计最大误差。
- [x] 迁移活跃直曲稳定化和三态直/圆/缓边界调整，删除现有简化小岛合并的主路径权威。
- [x] 使用旧路径的完整 `SNO .. next ENO` 目标样本范围，不再只取直线端部少量点。
- [x] 保留变量顺序 `(l1, R, l2)`、拨量平方和目标及活跃 Powell/Brent 搜索与停止条件，退役 Nelder-Mead 和曲率排名选圆点启发式。
- [x] 严格 GREEN：四类最大误差均 `<= 0.001 m`，并保持拨量、纵面、Debug/Release 和无 ARX/MFC 边界回归。

**数值一致性关键点**：`YNH2` 的圆心初值使用 `sum / count` 的直接除法。将它改写成 `sum * (1.0 / count)` 虽在数学上等价，但会改变最后一位浮点种子；旧 Powell 目标在现场曲线上较平坦，该差异会放大为厘米级参数漂移，因此实现必须保留旧运算顺序。

## Task 8：迁移纵断面重构

**Files**

- Create: `src/vertical/vertical_reconstructor.h`
- Create: `src/vertical/vertical_reconstructor.cpp`
- Create: `src/vertical/profile_classifier.h`
- Create: `src/vertical/profile_classifier.cpp`
- Create: `src/reconstruction.cpp`
- Create: `tests/test_vertical_reconstruction.cpp`
- Create: `tests/test_reconstruction.cpp`

**Legacy refs**：设计规格第 3.4 节全链和第 3.5 节纵面求值；需要时扩展阅读规范参数来源，但不引入数据库依赖。

**Why**：完成 XYZ 到变坡点 station/elevation/radius 的真实纵面重构，并与平面投影里程耦合在正确层。

**Verification**：合成单坡、多坡、含凸/凹竖曲线、噪声样本；控制点数量/顺序/里程/高程/半径；重构结果回灌后拨量与抬落道量接近零。

- [x] 覆盖非等间距二阶差商、坡段拟合、交点、合并和半径拟合。
- [x] 按 `FitSlope_SW` 活跃阶段顺序迁移纯规则。
- [x] 实现公共 `reconstruct` 编排平面投影和纵面重构。
- [x] 覆盖单坡、多坡、凸/凹竖曲线和零半径折坡。
- [x] 运行纵面单元、独立旧公式、闭环和完整重构测试。

## Task 9：现场对照、样例和文档

**Files**

- Create: `samples/main.cpp`
- Create: `README.md`
- Create: `tests/test_field_reference.cpp`
- Modify: `tests/AlignmentReconstruct.Tests.vcxproj`
- Modify: `samples/AlignmentReconstruct.Sample.vcxproj`

**Why**：证明公共 API 可被普通 C++17 调用，并把现场基准纳入回归。

**Verification**：sample 从 CSV 读取点，调用两个 API 并打印控制点/逐点调整；现场测试显示每类误差统计且在批准容差内。

- [x] 建立现场 fixture 测试并确认期望结果来自 MDB，不来自生产实现。
- [x] 完成公共 sample 和 README，明确字段、单位、正负号和 station 语义。
- [x] 运行现场全量测试；保存最大误差点索引和误差统计。
- [x] 复核测试参考与生产代码无共享公式。
- [x] 明确记录现场 XLSX 无 z，纵面采用独立公式和确定性 XYZ 证据。

## Task 10：最终构建、边界和完成审计

**Files**

- Create: `docs/verification/2026-07-13-build-and-parity.md`
- Modify: `docs/aegis/work/2026-07-13-alignment-reconstruct/20-checkpoint.md`
- Create: `docs/aegis/work/2026-07-13-alignment-reconstruct/90-evidence.md`
- Create: `docs/aegis/work/2026-07-13-alignment-reconstruct/99-reflection.md`

**Why**：用新鲜证据逐项证明真实目标，而不是以“能编译”替代算法完成。

**Verification**：完整执行计划头部命令，审阅所有测试覆盖、依赖扫描、项目类型、平台工具集、现场对照和设计规格成功条件。

- [x] 清理构建输出后执行 Debug x64 Build + 29 tests + sample。
- [x] 执行 Release x64 Rebuild + 29 tests + sample。
- [x] 执行生产依赖扫描、公共头文件审查、x64 archive 审查和旧仓库状态对照。
- [x] 按设计规格每一项要求建立证据表，明确现场纵面数据缺口而不误报。
- [x] 证据覆盖静态库产品边界全部成功条件，完成目标收口。

## Risks

- 现场 XLSX 只有 N/E，没有 z：不能用于纵面真实性声明；纵面由确定性三维夹具、旧公式对照和闭环证明。
- 旧代码对 N/E 与 x/y 的命名存在交换注释：公共映射固定为 `x=N`、`y=E`，所有现场测试锁定此约定。
- 缓和曲线投影和交点处容差敏感：先逐线元验证，再做全线最小距离选择。
- 旧代码含被注释版本和历史单位混用：A级活跃调用链优先，主结果不提前厘米/毫米舍入。
- 旧全局规范参数不完整：全部显式 options；现场水平 parity 不依赖纵面规范默认值。
- 目标目录不是 Git worktree：不自动初始化 Git；使用 Aegis 检查点和证据文件保持可恢复性。

## Retirement

- 旧 ObjectARX 算法在本目标中保留且只读，不从旧插件退役。
- 新项目内不保留第二套生产实现；`legacy_reference` 只属于测试工程。
- 数据抽取脚本只生成 fixture，不被静态库或 sample 链接。
- 一旦迁移模块通过对照，临时调试输出必须删除或转为测试失败诊断；不得形成长期 fallback。

## ADR / Baseline Sync Signal

完成时评估：公共契约、typed line elements、station 单一权威、测试参考隔离和外部数据 fixture 边界是否需要 ADR；修订基线须从“尚未实现”同步为实际文件和已验证所有者状态。
