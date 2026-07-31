# 点云中心线拟合与等间隔采样实施计划

日期：`2026-07-31`

## Goal

在 `AlignmentReconstruct` 静态库中新增公共函数
`fit_point_cloud(const std::vector<Point3D>&, double interval_m)`：输入单位为米、
文件行序无关的 XYZ 铁路点云，复现本任务中已经验证的 Python 点云中心线算法，
按拟合中心线的 XY 水平弧长等间隔输出三维中心线点。

## Architecture

- 公共契约位于 `include/alignment_reconstruct/point_cloud_fit.h` 和
  `include/alignment_reconstruct/types.h`。
- `detail::PointCloudFitter` 是点云空间分箱、鲁棒局部二次拟合和弧长采样的唯一
  生产 owner，位于 `src/point_cloud/`。
- `src/point_cloud_fit.cpp` 只做公共输入校验、异常到稳定诊断的转换和 owner 编排。
- 现有 `reconstruct(...)` 继续只接收沿线路方向排序的中心线测点；不得静默改写其
  输入契约，也不得让 `PointCloudFitter` 复制 `HorizontalAlignment` 的投影职责。

## Tech Stack

- Visual Studio 2026、MSVC v145、C++17、x64 静态库。
- 生产代码仅使用 C++17 标准库。
- `/W4`、警告即错误、Debug/Release 双配置。

## Baseline / Authority Refs

- 用户在当前任务中给出的 `fit_point_cloud` 名称、米单位、输入点云和采样间隔要求。
- 本会话已验证的 Python owner：
  `process_point_cloud.py::build_centerline`、
  `robust_local_quadratic`、`sample_polyline_by_plan_length`。
- `AGENTS.md`、`README.md`。
- `docs/aegis/baseline/2026-07-13-revised-baseline.md`。
- `docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md`。
- `docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md`。

## Compatibility Boundary

- 新 API 的所有输入、间隔和输出坐标均为米。
- 输入行序不参与结果；输出方向固定为 X 坐标较小端到较大端。
- 该算法要求线路在 X 方向单值展开且空间分箱有足够支持，不是任意拓扑点云寻路。
- 固定复现 Python 参数：`1 m` X 分箱、平面半窗 `12 m`、高程半窗 `6 m`、
  三轮鲁棒局部二次拟合。
- 采样里程只累计 XY 水平弧长；Z 不参与间隔。输出包含起点、每个标准间隔点，
  并保留不足一个间隔的实际终点。
- 现有两个公共入口及其类型、单位、符号和诊断保持不变。

## TDD Route

- Mode: `off`
- Decision: `skipped`
- Strict authority: `not applicable`
- Test posture: 针对性契约和确定性回归测试；另用本会话真实点云与 Python 输出做
  环境变量驱动的私有数值对照。
- Reason: 用户未要求严格 TDD；项目要求新增一个在旧实现上无法通过、在新实现上
  通过的针对性测试。
- Verification: Debug/Release 全量构建、测试和样例；真实点云逐点对照；生产边界扫描。

## Aegis Visibility

规划压力来自新增公共 API、第三个数值 owner、输入排序边界和 Python/C++ 数值一致性；
计划把这些责任隔离在点云拟合 owner 中，避免改变现有重构和投影权威。

## BaselineUsageDraft

- Required baseline refs: 修订基线、完整设计、ADR-0001、公共头文件。
- Delivered context refs: 用户当前任务及本会话 Python 算法和真实输出。
- Acknowledged before plan refs: 全部 required refs。
- Cited in plan refs: 全部 required refs。
- Missing refs: 无。
- Decision: `continue`

## Requirement Ready Check

- Requirement source refs: 用户当前任务。
- Goals and scope refs: 新增 `fit_point_cloud`，输入米制 XYZ 点云和间隔，输出采样点。
- User / scenario refs: 将本会话离线 Python 点云处理能力放入普通 C++ 静态库。
- Requirement item refs: 行序无关、中心线拟合、XY 弧长等间隔、三维输出。
- Acceptance / verification criteria refs: 合成数据、行序不变性、非法输入、真实点云
  与 Python 输出逐点比较。
- Open blocker questions: 无；端点策略沿用已验证 Python 行为。
- Decision: `ready`

## Change Necessity

- User-visible need: C++ 调用方需要直接在内存中完成点云拟合和采样。
- No-change / non-code option: 继续调用 Python 或先在外部预处理。
- Why code change is necessary: 当前静态库没有无序点云中心线 owner 或公共入口，
  外部 Python 不能满足库内调用要求。
- Minimum change boundary: 新公共头文件、结果类型、点云 owner、公共转发、测试、
  工程清单和必要契约文档。
- Decision: `code-change`

## Existence Check

- Proposed new surface: `fit_point_cloud` 和 `PointCloudFitter`。
- Existing owner / reuse candidate: `reconstruct`、`HorizontalReconstructor`。
- Why existing surface is insufficient: 它们要求已经排序的中心线测点，并输出参数化
  平纵方案；让它们接收宽带无序点云会破坏现有输入和输出契约。
- Creation proof: 新 owner 只负责宽带点云压缩成等间隔中心线点，职责与现有 owner
  不重叠。
- Entropy / retirement impact: 不新增 fallback，不保留第二套实现；Python 仅作为
  本次迁移验证参考，不进入生产库。
- Decision: `add-with-proof`

## Architecture Integrity Lens

- Invariant: 现有 `reconstruct` 不排序、不丢点；新函数独立声明其行序无关语义。
- Canonical owner / contract: `PointCloudFitter` / `point_cloud_fit.h`。
- Responsibility overlap: 不构造线路方案、不投影、不计算拨量或设计高程。
- Higher-level simplification: 公共自由函数只编排一个 owner。
- Retirement / falsifier: 若测试发现算法需要任意拓扑寻路或复用投影 owner，本边界
  失效并停止扩张。
- Verdict: 新 owner 边界成立。

## Plan-Time Complexity Check

- Artifact class: 新的独立数值 owner。
- Target files: `src/point_cloud/point_cloud_fitter.*`。
- Current pressure: 现有 owner 已大且语义不同，直接加入会形成职责混杂。
- Projected post-change pressure: 独立文件约数百行，公共转发保持小型。
- Budget result: `within-budget`
- Recommendation: `add owner file`

## Files

- 新增：`include/alignment_reconstruct/point_cloud_fit.h`
- 修改：`include/alignment_reconstruct/types.h`、`src/types.cpp`
- 新增：`src/point_cloud_fit.cpp`
- 新增：`src/point_cloud/point_cloud_fitter.h`
- 新增：`src/point_cloud/point_cloud_fitter.cpp`
- 新增：`tests/test_point_cloud_fit.cpp`
- 修改：库/测试 `.vcxproj` 与 `.filters`
- 修改：`README.md`、`AGENTS.md`、修订基线和 Aegis 索引
- 新增：`docs/aegis/adr/ADR-0002-point-cloud-fit-owner.md`

## Tasks

### Task 1：固定公共契约和失败语义

- Files: 公共头文件、`types.h`、`types.cpp`、契约测试。
- Why: 调用方需要米制三维点和稳定失败结果。
- Change Necessity: 现有结果类型不能表达点云拟合输出。
- Impact / Compatibility: 只追加枚举值和结果类型，不改已有值的顺序或语义。
- Steps:
  1. 增加 `PointCloudFitResult`、点云专用诊断码和 `ok()`。
  2. 声明 `fit_point_cloud(unordered_points, interval_m)`。
  3. 测试默认成功状态、米单位和无表头式纯数值结果契约。
- Verification:
  `AlignmentReconstruct.Tests.exe point_cloud`

### Task 2：移植点云中心线 owner

- Files: `src/point_cloud/point_cloud_fitter.*`、`src/point_cloud_fit.cpp`。
- Why: 在 C++ 内复现已验证算法。
- Change Necessity: 标准库中不存在空间中位数分箱、鲁棒局部拟合和弧长采样组合。
- Impact / Compatibility: 不调用或改写现有重构、alignment 或 adjustment owner。
- Steps:
  1. 实现确定性中位数、X 空间分箱和支持度校验。
  2. 实现加权局部二次最小二乘、三轮 Tukey 双权鲁棒拟合。
  3. 拟合 Y/Z 中心、补齐实际端点、按 XY 累计弧长插值采样。
  4. 公共入口校验有限数、点数和正间隔并返回稳定诊断。
- Verification:
  合成直线、打乱输入、非法间隔、非有限值和稀疏分箱测试。

### Task 3：真实数据 Python/C++ 对照

- Files: `tests/test_point_cloud_fit.cpp`。
- Why: 合成测试不能证明跨语言迁移没有数值漂移。
- Impact / Compatibility: 真实文件仅通过环境变量提供，不提交私有数据或绝对路径。
- Steps:
  1. 测试读取可选毫米点云和 Python 毫米中心线参考并转换为米。
  2. 调用 `fit_point_cloud(..., 10.0)`。
  3. 逐点比较点数和 XYZ，报告最大误差；缺少环境变量时明确跳过。
- Verification:
  设置两个环境变量后运行过滤测试，预期 262 点且误差在确定容差内。

### Task 4：同步工程和公共文档

- Files: `.vcxproj`、`.filters`、`README.md`、`AGENTS.md`、baseline、ADR、INDEX。
- Why: 新源文件必须进入生产/测试工程；公共契约和架构权威不能保持“两项能力”的旧叙述。
- Change Necessity: 仅新增源文件而不更新工程不会被构建；不更新文档会产生契约漂移。
- Impact / Compatibility: 明确新函数不是通用点云寻路，保留旧 API 输入契约。
- Verification:
  工程清单扫描、README 调用示例、生产依赖扫描。

### Task 5：完整构建和边界验证

- Files: 全部改动。
- Steps:
  1. Debug x64 Rebuild，运行全量测试和样例。
  2. Release x64 Rebuild，运行全量测试和样例。
  3. 执行生产禁用依赖扫描、`git diff --check`、CRLF 和项目清单检查。
  4. 审查真实点云对照、输入行序不变性和现有测试回归证据。
- Verification:
  所有命令退出 `0`；只允许私有现场测试按缺少环境变量明确跳过。

## Risks

- 正规方程与 NumPy `lstsq` 的浮点路径不同；使用带主元的长双精度三元方程并以真实
  点云逐点误差约束漂移。
- 原算法以 X 为展开轴；文档和诊断必须拒绝把它宣传为任意方向或自交线路寻路。
- `interval_m` 大于线路长度时仍应返回起点和实际终点。
- 公共诊断枚举为追加式变更；调用方若序列化枚举整数需要重新编译并复核。

## Retirement

- 不修改或退役现有 `reconstruct`、`HorizontalReconstructor` 或 Python 成果文件。
- 不引入生产 fallback、旧代码副本或测试参考链接。
- Python 实现只作为本次迁移的外部数值参考；C++ owner 通过后是库内唯一生产实现。

## Execution Readiness View

- Intent Lock: 新增 C++ 点云拟合采样 API，复现本会话 Python 行为。
- Scope Fence: 仅纯数值静态库、测试、工程和契约文档；不做 CAD、文件解析或通用寻路。
- Baseline Lock: 米制、C++17 标准库、现有 owner 和诊断模式。
- Approved Behavior: 行序无关、X 分箱、鲁棒 Y/Z 拟合、XY 弧长采样、保留终点。
- Owner / Contract Constraints: 新 `PointCloudFitter` 独占点云拟合；旧 owner 不变。
- Compatibility Boundary: 追加式 API；现有函数行为不变。
- Retirement Boundary: 无 fallback、无旧 owner 替换。
- Task Batches: 契约 → owner → 真实对照 → 文档工程 → 双配置验证。
- Test Obligations: 合成、失败、洗牌、真实参考、全量回归。
- Review Gates: 公共契约审查、数值对照、生产边界扫描。
- Drift / Rewind Rules: 若需要通用寻路或改变 `reconstruct` 输入契约，停止并重新设计。
- Evidence Required Before Completion: Debug/Release 构建、全量测试、样例、真实逐点对照、
  禁用依赖扫描和 CRLF 检查。
- Advisory Boundary: method-pack execution guidance only; not GateDecision,
  PolicySnapshot, or completion authority.
