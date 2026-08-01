# AlignmentReconstruct Agent 开发指南

## 1. 适用范围与目标

本文件约束 `AlignmentReconstruct` 根目录及其全部子目录中的开发工作。项目是独立的 Visual Studio 2026、C++17、x64 动态库，生产代码只能依赖 C++17 标准库。不得引入 AutoCAD、ObjectARX、MFC、MDB、Excel、COM 或旧工程运行时依赖。

库的公共能力有三项：

1. `fit_point_cloud`：从行序无关、沿 X 方向单值展开的 XYZ 点云拟合中心线，并按
   XY 水平弧长等间隔输出三维点。
2. `reconstruct`：从沿线路前进方向排序的 XYZ 测点重构平面和纵断面。
3. `calculate_adjustments`：用完整平纵方案逐点计算投影里程、拨量、设计高程和抬落道量。

CAD 绘图、文件解析、数据库访问、任意拓扑点云寻路和旧插件集成都不属于本库。

## 2. 开始修改前的必读顺序

先检查工作区，保留已有修改；本目录可能尚未初始化 Git，不得擅自初始化仓库、提交或清理文件。

按任务范围阅读以下权威资料：

1. `README.md`：调用方契约、单位、正负号、错误处理和构建方式。
2. `docs/aegis/baseline/2026-07-13-revised-baseline.md`：当前产品与架构基线。
3. `docs/aegis/specs/2026-07-13-alignment-reconstruct-design.md`：完整设计；涉及算法迁移或兼容性时必须阅读第 3 节“High-value legacy seams”。
4. `docs/aegis/adr/ADR-0001-pure-cpp-alignment-owners.md`：唯一所有者和依赖方向。
5. `docs/aegis/adr/ADR-0002-point-cloud-fit-owner.md`：点云拟合 owner、输入几何和采样边界。
6. `include/alignment_reconstruct/` 与本次变更对应的 `src/` owner。
7. `docs/verification/2026-07-13-build-and-parity.md`：已有证据、容差和未覆盖边界。

不要把历史验证记录当作当前构建结果；完成前必须重新执行与变更风险相称的验证。

## 3. 不可改变的公共语义

- 坐标：公共数值 API 固定为 `x = N（北坐标/纬距）`、`y = E（东坐标/经距）`、`z = H（高程）`。
- CAD 边界：旧 DWG 代码常使用 `CAD X = E`、`CAD Y = N`；坐标交换应由调用方适配层完成，不能改变本库契约。
- 单位：坐标、station、半径、缓长、高程、拨量和抬落道量的权威值均为米；毫米只允许作为派生便捷值。
- 拨量：沿里程增加方向观察，测点位于方案线右侧为正、左侧为负。
- 抬落道：`lift_drop_m = design_elevation_m - measured_z`；正值抬道，负值落道。
- 里程：拨量和设计高程必须共享 `HorizontalAlignment` 产生的同一个投影 `station_m`。
- `VerticalControlPoint::station_m` 与平面线路使用同一数值里程基准，不是点序号，也不是独立纵面累计长。
- 平面必须真实支持直线、进口 Euler 缓和曲线、圆曲线和出口 Euler 缓和曲线；纵面控制点的半径是物理竖曲线半径。
- `reconstruct` 不得静默排序或丢弃输入点；`fit_point_cloud` 按契约不使用输入行序，
  但只支持沿 X 方向单值展开且空间分箱充分的点云。
- 输入或几何失败必须返回稳定诊断；不得静默丢点、回退为折线或用硬编码结果掩盖失败。

任何公共字段、单位、符号、输入范围、诊断或默认参数变更，都属于契约变更：先核对基线和设计，补充测试，并同步 `README.md`。

## 4. 生产所有权与依赖方向

- `include/alignment_reconstruct/`：唯一公共契约。
- `HorizontalReconstructor`：唯一平面重构 owner。
- `VerticalReconstructor`：唯一纵断面重构 owner。
- `HorizontalAlignment`：唯一平面建线、按里程求点和最近投影 owner，也是 station 的唯一权威来源。
- `VerticalAlignment`：唯一纵面校验和按里程求设计高程 owner。
- `AlignmentAdjustmentCalculator`：只编排平面投影与纵面求值，不得复制投影、符号、缓和曲线或竖曲线公式。
- `PointCloudFitter`：唯一点云空间分箱、鲁棒局部拟合和 XY 弧长等间隔采样 owner；
  不得复制参数化线路建线、投影或纵面求值公式。
- `tests/legacy_reference/`：仅用于测试数值对照，绝不能出现在生产 DLL 中。
- `include/alignment_reconstruct/api.h`：DLL 可见性的唯一 owner；只有
  `fit_point_cloud`、`reconstruct`、`calculate_adjustments` 使用导出宏，禁止导出
  `detail::*` 或新增自动导出路径。

依赖应从公共自由函数流向内部编排 owner，再流向 alignment 和 typed 几何线元。不要新增第二套投影、高程求值或重构实现，也不要建立通用算法“大杂烩”文件。

## 5. 旧项目研究规则

旧工程默认位于同级目录 `../主控`，只作为行为证据。设计规格第 3 节给出的入口是导航起点，不是阅读边界；必要时必须继续追踪调用者、被调函数、参数来源、全局配置和实际可达分支。

高价值入口：

| 目的 | 旧代码入口 |
| --- | --- |
| 平面“直曲自动分段”活跃入口 | `QXBoLiangCalDlg.cpp:70-157`，`CQXBoLiangCalDlg::OnBnClickedButtonZdfd` |
| 平面“生成平面线位”活跃入口 | `QXBoLiangCalDlg.cpp:159-258`，`CQXBoLiangCalDlg::OnBnClickedButtonNhht` |
| 点序、累计距离、曲率和初始分类 | `CeDianNiHe.cpp:848-1366`：`Sort`、`JMSurPtLC`、`CalCurvature`、`MarkProperty`、`Dynamic_Adjust_Property_SetAdjust` |
| 直曲稳定化和反向曲线处理 | `CeDianNiHe.cpp:1373-1838`：`FormLineSeg`、`IDdentifyLineOrCurve`、`DealFanxiangQX`、`CalFJD`、`CheckCurve` |
| 直线拟合和相交判定 | `CeDianNiHe.cpp:2279-2431`：`IsIntersectionBetween`、`FitLine`、`FootPoint` |
| 圆/缓和三态识别 | `CeDianNiHe.cpp:2451-2882`：`IDSpiral`、`FormLineSeg2`、`AtLeastOneCurvPt`、`AtLeastOneLinePt` |
| 全线参数输出和拨量优化 | `CeDianNiHe.cpp:2974-4278`：`AllQXNH2`、`YNH2`、`CalJDZB`、`CalL0`、`YouHuaBL`、`CalBL`、Powell/Brent 系列 |
| 缓和边界摆动稳定化 | `CeDianNiHe.cpp:4692-4917`，`SwingAlgorithm` |
| JD 方案生成 typed 线路的旧行为 | `RHA.cpp:3348-4003` |
| 直线/圆/缓和线元与投影 | `BAS_CALC_FUN.cpp:767-1165`、`:2279-2897` |
| 拨量及正负号 | `RHA.cpp:26041-26188` |
| 纵断面重构业务入口 | `ZDMSJ.cpp:941-962`：`ZDMSJ::FitSlope`；`RebuildDlg.cpp:93-368`：`CRebuildDlg::OnBnClickedOk` |
| 纵面样本和参数来源 | `RebuildDlg.cpp:93-167`：从当前轨面线形成 `Cml/H` 样本；`:590-662`：规范半径、最小坡长、限坡和坡差；`:179-367`：固定阈值或候选阈值及半径归整 |
| 纵断面活跃拟合链 | `ReBuildGMX.cpp:841-1680`：`CReBuildGMX::FitSlope` 当前未注释的“方法三、原始”；`:2797-3669`：分类、直线拟合、坡段合并、半径和取整帮助函数 |
| 纵面规范整饰 | `ReBuildGMX.cpp:1795-2380`：限坡、最大坡差、竖缓避让和短坡合并 |
| 纵面设计高程和竖曲线 | `BAS_CALC_FUN.cpp:10269-10576` |
| 平纵联合调用 | `ZDMSJ.cpp:10949-10968` |
| 抬落道及符号 | `DrawSlope.cpp:6337-6367`、`:3306-3376` |

平面迁移必须先从上述两个按钮处理函数确认当前实际调用顺序，再进入 `CCeDianNiHe` 的被调函数；同名 `_SW`、`New`、`opti`、`SwingAlgorithm_SinglePI` 等未被这两个入口调用的实现不能取代活跃路径。`Checkline` 虽被调用但函数体为空，不得从注释代码臆造行为。

纵面迁移的权威路径固定为用户实际运行并产出对照坡度表的 `ZDMSJ::FitSlope` → `CRebuildDlg::OnBnClickedOk` → `CReBuildGMX::FitSlope`。必须保留其前后超过 15 m 的二阶差商取点、结果绝对值乘 100、固定 `0.001` 默认阈值、最小坡长交点约束、全部内部变坡点里程取整、带坡角余弦和前后限位的边界震荡，以及 `30000 m` 半径硬上限。`CH_V_Cal` 的 `_SW` 路径可用于理解三维测点如何形成投影里程，但不能覆盖本次真实运行结果所证明的纵面算法行为。

参考优先级为：真实运行结果和参数 > 活跃业务调用链 > 活跃算法帮助函数 > 显示/统计代码。注释掉的历史实现只能作为线索。不得直接搬入 `CCeDianNiHe`、`CReBuildGMX`、`CH_V_Cal`、`RHA`、`CString`、`CArray`、`AcGe`、CAD/UI/数据库代码、全局状态或十列裸数组模型。

迁移或修正算法时，记录并验证“旧参考函数 → 新 owner → 对照夹具 → 数值容差”。目标是行为一致，不要求代码逐字一致。

## 6. 代码和项目文件约定

- 使用 C++17、RAII、值类型和显式单位后缀（如 `_m`、`_inverse_m`）；公共头文件只包含标准库头文件。
- 保持 `namespace alignment_reconstruct`，内部实现放在最窄的 owner 中；不要扩大公共 API 来方便测试。
- 保持 `/W4` 和警告即错误；不要通过关闭警告绕过问题。
- 新增、删除或移动源文件时，同步更新对应 `.vcxproj` 和 `.vcxproj.filters`；测试与样例工程也一样。
- 保持维护文本为 CRLF，避免无关格式化和大范围重排。
- `build/` 与 `.scratch/` 是生成/临时区域，不是源代码或设计权威，不要手工修补其中产物。
- 不要在源码、测试、README 或项目文件中写入原始 XLSX/MDB 的本机绝对路径。

## 7. 测试与夹具规则

每次行为变更至少增加一个能在修改前失败、修改后通过的针对性测试。优先覆盖：

- 直线、圆曲线、进出口不等长缓和曲线和线元连接点；
- 左右拨量符号、线路端点外投影和同里程不变量；
- 凸/凹竖曲线、零半径折坡、纵面覆盖范围和抬落道符号；
- 平纵重构闭环，以及修改重构交点后再次调用 `calculate_adjustments`；
- 点云输入行序不变性、空间支持度、XY 弧长采样端点和 Python 迁移数值对照；
- 非法输入的稳定 `StatusCode`、`DiagnosticCode` 和 `input_index`。

`tests/fixtures/*.csv` 是私有现场证据，已由 `.gitignore` 排除，严禁提交、推送或放入公开发布包。公开仓库只保留 `tests/fixtures/README.md`；没有私有夹具时相关测试必须明确跳过，不能伪装为通过。普通测试不得依赖 ACE、Office、外部数据库或本机路径。只有在有明确权威来源和审查理由时才更新本地夹具；现场兼容性结论必须逐字段比较并报告最大误差、RMS（适用时）和容差，不能以“构建成功”代替。

## 8. 构建与完成检查

所有命令使用 Git Bash。在根目录执行：

```bash
MSYS2_ARG_CONV_EXCL='*' MSBuild.exe \
  AlignmentReconstruct.sln /m /t:Rebuild \
  /p:Configuration=Debug /p:Platform=x64 /v:minimal

./build/Debug/AlignmentReconstruct.Tests.exe
./build/Debug/AlignmentReconstruct.Sample.exe

MSYS2_ARG_CONV_EXCL='*' MSBuild.exe \
  AlignmentReconstruct.sln /m /t:Rebuild \
  /p:Configuration=Release /p:Platform=x64 /v:minimal

./build/Release/AlignmentReconstruct.Tests.exe
./build/Release/AlignmentReconstruct.Sample.exe
```

若 `MSBuild.exe` 不在 `PATH`，使用本机 Visual Studio 2026 的 x64 MSBuild 绝对路径执行命令，不要降级工具集或改写项目配置。

生产边界扫描：

```bash
rg -n -i \
  'afx|AcDb|AcGe|ObjectARX|CString|ads_|aced|XLDataBase|Excel|legacy_reference' \
  include src AlignmentReconstruct.vcxproj
```

预期无命中。交付前还要确认：Debug/Release 测试均退出 0、样例退出 0、
`dumpbin /exports` 只有三个公共入口、测试和样例依赖 `AlignmentReconstruct.dll`、
新文件已进入正确项目、生产 DLL 未链接测试对象、相关文档已同步、未引入绝对测试
数据路径。只报告本次实际执行过的验证；无法执行的检查要明确说明原因和剩余风险。
