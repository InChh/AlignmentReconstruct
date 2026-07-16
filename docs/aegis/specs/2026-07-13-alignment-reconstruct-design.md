# AlignmentReconstruct 完整设计规格

日期：`2026-07-13`  
状态：`已实现；2026-07-16 平面活跃路径完成毫米级对齐`
架构审查：`需要`

## 1. 目标

在 `D:\Programming\C++\Projects\既有线\AlignmentReconstruct` 建立独立的 Visual Studio 2026、C++17、x64 静态库项目，从既有 ObjectARX 工程中抽取纯数值算法，对外提供两项能力：

1. 输入按线路前进方向排序的 XYZ 测点，重构平面线形和纵断面线形。
2. 输入 XYZ 测点以及完整线路方案（平面方案和纵断面方案），逐点计算拨量与抬落道量。

新项目不得依赖 ObjectARX、AutoCAD、MFC、MDB、Excel 或旧工程的运行时状态。

## 2. 现有项目中的真实业务链

实现 Agent 必须以当前项目中的实际调用链和活跃函数为行为依据，不能只根据类型名或界面文字猜测算法：

- “全线坐标拟合”对话框的平面流程由两个按钮组成：`CQXBoLiangCalDlg::OnBnClickedButtonZdfd` 先完成直曲自动分段，`CQXBoLiangCalDlg::OnBnClickedButtonNhht` 再执行 `FormLineSeg` → `IDSpiral` → `SwingAlgorithm` → `AllQXNH2`，这是平面重构的 A 级入口。
- `OnBnClickedButtonZdfd` 的活跃数值顺序是 `Sort` → `JMSurPtLC` → `CalCurvature` → `MarkProperty` → `IDdentifyLineOrCurve` → `Dynamic_Adjust_Property_SetAdjust(5)` → `DealFanxiangQX` → `CalFJD` → `CheckCurve` → `Checkline` → `Dynamic_Adjust_Property_SetAdjust(6)`；其中 `Checkline` 当前为空函数。
- `RHA::CalRectification` 先将测点 XY 投影到平面线路，得到投影里程、投影点和带符号拨量：`RHA.cpp:26041-26188`。
- `ZDMSJ.cpp:10961-10968` 使用同一投影里程调用纵断面计算，形成拨量与抬落道量的联合结果。
- `CDrawSlope::CalTLDL` 按 `设计高程 - 实测高程` 计算抬落道量，正值为抬道、负值为落道，并在旧界面链路中换算为毫米：`DrawSlope.cpp:6339-6367`。
- `BAS_CALC_FUN::MLToH_Bas` 明确“考虑了竖曲线”计算设计高程：`BAS_CALC_FUN.cpp:10269-10576`。
- `PDTAB` 的 `Rshu` 是变坡点竖曲线半径：`Struct.h:1482-1524`。
- 纵断面重构会拟合并写回 `Rshu`：`CH_V_Cal.cpp:3183-3461`、`CH_V_Cal.cpp:4444-4542`。

这些代码共同确定本项目的公共契约：纵断面重构必须输出竖曲线半径，第二项能力必须接收完整平纵方案并联合计算拨量和抬落道量。

## 3. High-value legacy seams（高价值旧代码入口）

所有旧代码路径均相对于：

`D:\Programming\C++\Projects\既有线\主控`

### 3.1 总流程和数据契约

| 目的 | 必读代码 | Agent 应提取的行为 | 不应带入新库的内容 |
| --- | --- | --- | --- |
| 平面直曲自动分段入口 | `QXBoLiangCalDlg.cpp:70-157`，`CQXBoLiangCalDlg::OnBnClickedButtonZdfd` | 按活跃顺序读取/整理测点、累计距离、计算曲率、直曲稳定化、反向曲线处理和短曲线检查 | 文件读取、Excel 回写、绘图、进度条、消息框 |
| 平面线位生成入口 | `QXBoLiangCalDlg.cpp:159-258`，`CQXBoLiangCalDlg::OnBnClickedButtonNhht` | 依据当前 `SelSurPt.Propert` 重建直线段；有曲线时执行 `IDSpiral` 和 `SwingAlgorithm`，最后无条件执行 `AllQXNH2` 输出方案 | 对话框选项、CAD 文档锁、实体创建和显示 |
| 三维测点及联合结果字段 | `Struct.h:508-561`，`SSurPt` | N/E、投影里程、拨量、实测高程、设计高程、抬落道量之间的关系 | `CString`、线路名、现场里程冠号、界面限制字段 |
| 平面交点输出 | `Struct.h:678-732`，`JDarrayStruct` | 交点 N/E、`R`、`l1`、`l2`；起终点半径和缓长为零 | MDB/绘图/改建属性及字符串字段 |
| 平面线元里程 | `Struct.h:563-675`，`QXBStruct`/`QXBCmlStruct` | ZH/HY/YH/HZ 里程、转角、切线长、曲线长、左右转 | 现场里程字符串和图形字段 |
| 纵断面控制点 | `Struct.h:1482-1524`，`PDTAB` | `Cml`、`Level`、`Rshu`、`RLen`、`degree`、`length` 的语义和关系 | 线路名、冠号、数据库和设计控制附属字段 |
| 纵面测点 | `Struct.h:4713-4745`，`GMXPt` | 测点统一里程、高程、二阶差商、直曲属性、拟合高程和抬落道量 | UI 限制和现场里程附属字段 |

### 3.2 平面重构主链

| 阶段 | 必读代码 | Agent 实现指引 |
| --- | --- | --- |
| 点序与累计距离 | `QXBoLiangCalDlg.cpp:70-157` → `CeDianNiHe.cpp:848-963`：`Sort`、`JMSurPtLC` | 旧界面会整理点序；公共契约要求调用方按线路方向给点，因此新库保留顺序、重复点和累计距离校验，不复制文件/UI 自动寻路 |
| 曲率和初始属性 | `CeDianNiHe.cpp:964-1226`：`CalCurvature`、`GetCirByThreePointOnArc`、`MarkProperty` | 保留 15 m 前后搜索、三点切向/圆心状态、曲率和 `0.00008 1/m` 初始阈值的计算顺序 |
| 直曲稳定化总顺序 | `QXBoLiangCalDlg.cpp:117-143`；`CeDianNiHe.cpp:1310-1838` | 严格按 `IDdentifyLineOrCurve` → 动态阈值 5 → `DealFanxiangQX` → `CalFJD` → `CheckCurve` → 空的 `Checkline` → 动态阈值 6 执行；不要调用同名 `New` 或注释分支 |
| 直线段、拟合和交点有效性 | `CeDianNiHe.cpp:1373-1520`、`:2279-2431`：`FormLineSeg`、`IsIntersectionBetween`、`FitLine`、`FootPoint` | 保留连续区段形成、端点强制直线、相邻直线延长线判定和旧正交拟合的运算顺序；数组越界只做等价安全化，不改变有效输入结果 |
| 反向曲线和短曲线 | `CeDianNiHe.cpp:1591-1731`：`CheckCurve`、`CalFJD`、`DealFanxiangQX` | 保留曲率符号反转时的 `0.0008 1/m` 再分类，以及长度小于 20 m 曲线段的处理 |
| 缓和三态识别 | `QXBoLiangCalDlg.cpp:229-251` → `CeDianNiHe.cpp:2451-2882`：`IDSpiral`、`FormLineSeg2`、`AtLeastOneCurvPt`、`AtLeastOneLinePt` | 仅在 `CurvNum > 0` 时执行；保留 `Propert = 0/1/2` 的直线/圆/缓和边界调整、点数约束和 300 次活跃上限 |
| 缓和边界摆动 | `CeDianNiHe.cpp:4692-4917`：`SwingAlgorithm` | `IDSpiral` 后继续用当前圆拟合、交点、初始缓长和线路投影反复重分三态，保留 500 次上限 |
| 圆拟合、交点和初始缓长 | `CeDianNiHe.cpp:3504-3789`：`YNH2`、`CalJDZB`、`CalL0`、`CalL02` | 保留圆心先直接除以样本数再累积正规方程的浮点运算顺序、相邻拟合直线交点以及两端不等长缓和估计 |
| 全线输出与优化 | `CeDianNiHe.cpp:2974-3283`、`:3792-4278`：`AllQXNH2`、`YouHuaBL`、`CalBL`、`powell`、`linmin`、`mnbrak`、`brent` | 形成起点、中间交点、终点并输出 N/E、R、l1、l2；`AllQXNH2` 当前无条件调用 `YouHuaBL`，变量顺序固定为 `(l1, R, l2)`，目标覆盖 `SNO .. 下一直线 ENO`，Powell 容差 `1e-4`、最多 10 次，`mnbrak` 最多 10 次；默认不等缓长且不取整 |
| 曲线点少于 3 个 | `CeDianNiHe.cpp:3038-3106`：`AllQXNH2` 内的 `BCTTP`/`BCTPP` 活跃分支 | 这是稀疏曲线样本的可达分支；若迁移该边界，必须继续读取 `BCTTP`/`BCTPP` 的活跃纯数学逻辑并建立专门对照测试，不能用任意圆拟合替代 |
| 方案转 typed 线路 | `AllQXNH2` 中 `m_SJD`/`pRHA_Opti` 数据流；`RHA.cpp:3348-4003` | 只提取“交点结果可构造并被 `CalBL` 投影评价”的数值行为；不创建 `RHA`、CAD 实体或十列裸数组 owner |

### 3.3 平面建线、按里程求点与最近投影

| 阶段 | 必读代码 | Agent 实现指引 |
| --- | --- | --- |
| 交点参数生成曲线表和线元 | `RHA.cpp:3348-4003`：`RHA::getpxy_DATA` | 这是由 JD、R、l1、l2 构造直线/前缓/圆/后缓以及累计里程的主要行为权威。新库必须改为 typed elements，不能复制十列裸数组作为公共或内部主模型 |
| 曲线要素 | `BAS_CALC_FUN.cpp:2279-2350`：`getBPMTL` | 提取转角、半径、两端缓长到 T1/T2/L 等曲线要素的公式 |
| 线元长度 | `BAS_CALC_FUN.cpp:767-826`：`Get_INDATA_L` | 作为旧线元类型与长度解释的对照，只用于映射和 parity；新库由 typed element 自己拥有长度 |
| 线元局部里程求坐标 | `BAS_CALC_FUN.cpp:827-985`：`xlzbxy` | 提取直线、圆、前缓、后缓的坐标、切向和曲率推进公式；剥离显示参数和旧结构体 |
| 全线里程求坐标 | `BAS_CALC_FUN.cpp:987-1165`：`xlpoint_pz` | 保留端点夹取、线元选择和连接点容差语义；新库返回 typed result，不返回数字型线元代码 |
| 点到线路投影里程 | `BAS_CALC_FUN.cpp:2445-2897`：`DistToXL`、`PROJ_ML` | 提取候选线元筛选、直线/圆投影、缓和曲线二分搜索和最小距离择优；修复固定 100 候选数组为动态容器，但用 parity 测试证明结果未漂移 |
| 圆弧投影里程 | `BAS_CALC_FUN.cpp:4768` 起：`Circle_XY_to_ML` | 作为圆线元投影到累计里程的公式依据 |
| 拨量符号 | `RHA.cpp:26041-26188`：`CalRectification`、`RectificationSign` | 投影后以线路前进切向判断左右；公共契约固定为沿里程方向右正左负 |

### 3.4 纵断面重构主链

| 阶段 | 必读代码 | Agent 实现指引 |
| --- | --- | --- |
| 三维点转纵面样本 | `CH_V_Cal.cpp:2939-3055`：`OnInitZDMBasicData_SW`、`GetPDandDSC_SW` | 使用平面投影后的统一里程和测点 z 形成纵面样本；规范参数必须变为显式 options |
| 阈值搜索 | `CH_V_Cal.cpp:3056-3197`：`GetOptThreshold_SW`、`MlToGMXH_SW` | 保留二阶差商阈值候选的评价方式；绘图和全局对象不进入新库 |
| 纵面总流程 | `CH_V_Cal.cpp:3198-3465`：`FitSlope_SW` | 严格参考现有阶段顺序：二阶差商 → 属性分类 → 直坡段 → 变坡点 → 合并 → 竖曲线半径 → 规范限制 → 高程重算 |
| 二阶差商与属性 | `CH_V_Cal.cpp:3686-3788`：`CountEstateFall_SW`、`JudgePtProperty_SW`、`CreatePtArry_SW` | 支持非等间距 station；端点处理和阈值比较必须有确定性测试 |
| 直坡段形成和拟合 | `CH_V_Cal.cpp:3839-4005`：`FormLineAry_SW`、`FitSglLine_SW`、`_rtCalculateLine_SW` | 提取直线段样本范围、最小二乘斜率/截距和误差计算 |
| 变坡点形成 | `CH_V_Cal.cpp:4006-4194`：`FormSlopeAry_SW`、`IsPIbetwnLines_SW` | 相邻坡段延长线交点生成 `station/elevation`；保持起终点固定规则 |
| 坡段整饰 | `CH_V_Cal.cpp:4195-4444`：`UniteSmallGradeFall_SW`、`CalSglSlopeDegree_SW`、`UniteSmallSlope_SW`、`RoundBPDMl_SW` | 保留小坡度差和最小坡长合并、坡度/坡长重算及里程取整顺序 |
| 竖曲线半径 | `CH_V_Cal.cpp:4445-4592`：`SetRShu_SW`、`FitRShu_SW`、`GetSEPt_SW`、`Func_SW` | 输出 `vertical_curve_radius_m`；保留坡度代数差触发、半径迭代、最小/最大半径限制和取整顺序 |
| 竖缓重叠与坡度调整 | `CH_V_Cal.cpp:4593-5552`：`AdjustSlopeByHHQX_SW` 到 `CalLimitGrade_SW` | 仅抽取影响最终变坡点/半径的纯规则；桥隧站等外部规范数据不隐式读取，改为 options 或明确不支持诊断 |
| 纵面输出准备 | `CH_V_Cal.cpp:5553-5575`：`CpyXLNameToSlope_SW` | 线路名复制不是核心；最终生产输出只取 station、elevation、radius 及必要派生坡度 |

### 3.5 纵断面求高程和抬落道量

| 阶段 | 必读代码 | Agent 实现指引 |
| --- | --- | --- |
| 按里程求设计高程 | `BAS_CALC_FUN.cpp:10269-10484`：`MLToH_Bas` | 这是纵面线路评估的主要行为权威，必须考虑直坡段和竖曲线范围 |
| 竖曲线坐标 | `BAS_CALC_FUN.cpp:10514-10576`：`VerticleArc` | 第一版公共半径采用物理圆曲线半径，重点对照 `Rshu >= 100` 的活跃圆弧分支；`0` 表示无竖曲线 |
| 联合调用 | `ZDMSJ.cpp:10949-10968` | 先 `CalRectification` 得 `Projml`，再用同一 `Projml` 计算纵面设计高程和 TLDL，这是第二公共功能的数据流权威 |
| 单点抬落道 | `DrawSlope.cpp:6337-6367`：`CDrawSlope::CalTLDL` | `HFall = design_elevation - measured_elevation`，返回链路使用毫米；新库内部保留米并提供毫米换算 |
| 正负号语义 | `DrawSlope.cpp:3306-3376`：`CountMaxFall` | `HFall >= 0` 为抬道，`HFall < 0` 为落道 |
| 批量重构评价 | `CH_V_Cal.cpp:6151-6301`：两个 `CalTLDALL` | 用于确认纵面重构目标统计和厘米舍入历史行为；不要把 `fabs` 后的厘米整数当作新公共 API 的主结果 |
| 批量逐点输出 | `DrawSlope.cpp:6272-6335`：批量 `CalTLDL` | 用于对照 `Level`、`TLDL` 和统计量；新库的逐点权威值仍为未提前舍入的米制差值 |

### 3.6 参考优先级与禁止复制边界

实现时按以下优先级处理冲突：

1. **A级：活跃业务调用链**——平面以 `OnBnClickedButtonZdfd` 和 `OnBnClickedButtonNhht` 为入口；联合调整量以 `CalRectification`、`ZDMSJ` 联合调用、`MLToH_Bas`、单点 `CalTLDL` 为入口。这些代码决定公共行为。
2. **B级：活跃算法帮助函数**——上述表格列出的重构、建线、投影、竖曲线函数。用于复现数值步骤。
3. **C级：显示和统计代码**——Excel、CAD 绘制、厘米/毫米格式化、汇总表。只能解释展示单位，不能反向改变主数值契约。
4. **注释掉的旧实现**不得作为首选权威；只有活跃函数缺失公式且有测试证据时，才可作为调查线索。

严禁直接复制到生产库：`CCeDianNiHe`、`CH_V_Cal`、`RHA`、`CDrawSlope` 整类，`CString`/`CArray`/`AcGe` 类型，CAD 绘图/文档锁/进度条，MDB/Excel 访问，全局状态，以及十列裸线元数组作为新核心模型。

每完成一个算法切片，Agent 必须记录“参考函数 → 新所有者 → 对照夹具 → 容差”，避免只抽取函数名而丢失调用顺序和参数来源。

## 4. 方案比较与选择

### 4.1 方案 A：联合线路评估（采用）

平面线路负责投影和拨量，纵断面负责按同一里程求设计高程，最外层计算器只负责编排并形成逐点联合结果。

优点：与现有代码的真实数据流一致；投影里程只有一个权威来源；不会重复最近点搜索；便于验证平面、纵面和联合结果。

### 4.2 方案 B：拨量与抬落道量两个完全独立的公共函数（不采用）

调用方需要先自行取得里程，再把里程和高程传给另一个函数。

缺点：把关键的“同一投影里程”不变量推给调用方，容易出现平面里程和纵面里程不一致，也会泄漏内部步骤。

### 4.3 方案 C：直接求测点到三维线路的最短空间距离（不采用）

缺点：三维最短距离不是铁路拨量和抬落道量的定义。现有业务先在平面投影，再分别计算横向差和竖向差，不能用一个三维距离替代。

## 5. 项目结构

```text
AlignmentReconstruct/
  AlignmentReconstruct.sln
  README.md
  include/alignment_reconstruct/
    types.h
    reconstruction.h
    adjustment.h
  src/
    geometry/
    horizontal/
    vertical/
    alignment/
  tests/
    AlignmentReconstruct.Tests.vcxproj
    legacy_reference/
    fixtures/
  samples/
  docs/aegis/
```

解决方案包含：

- `AlignmentReconstruct`：C++17 x64 静态库，平台工具集 `v145`，只依赖 C++ 标准库。
- `AlignmentReconstruct.Tests`：C++17 x64 控制台测试程序，只链接静态库和测试专用参考实现。

必须支持 `Debug|x64` 和 `Release|x64`。

## 6. 公共数据契约

### 6.1 单位与方向

- 坐标、里程、半径、缓和曲线长、高程、拨量和抬落道量的主单位均为米。
- 公共结果可提供毫米便捷换算，但不把毫米作为内部权威单位。
- 输入重构测点按线路起点到终点排序；库不自动重排。
- 沿里程增加方向观察：右侧拨量为正，左侧拨量为负。
- `抬落道量 = 设计高程 - 测点 z`；正值表示抬道，负值表示落道。
- `station` 表示沿平面线路从 `start_station_m` 开始累计的统一里程，不是测点序号，也不是测点间原始折线距离。

### 6.2 核心类型

公共头文件提供语义明确的值类型，形状等价于：

```cpp
namespace alignment_reconstruct {

struct Point2D {
    double x;
    double y;
};

struct Point3D {
    double x;
    double y;
    double z;
};

enum class HorizontalControlPointKind {
    start,
    intersection,
    end
};

enum class HorizontalElementKind {
    straight,
    entry_spiral,
    circular,
    exit_spiral
};

struct HorizontalControlPoint {
    HorizontalControlPointKind kind;
    Point2D position;
    double radius_m;
    double entry_spiral_length_m;
    double exit_spiral_length_m;
};

struct VerticalControlPoint {
    double station_m;
    double elevation_m;
    double vertical_curve_radius_m;
};

struct AlignmentDefinition {
    double start_station_m;
    std::vector<HorizontalControlPoint> horizontal;
    std::vector<VerticalControlPoint> vertical;
};

struct PointAdjustment {
    std::size_t point_index;
    double station_m;
    Point2D projected_point;
    double design_elevation_m;
    double horizontal_offset_m;
    double lift_drop_m;
    HorizontalElementKind horizontal_element;

    double horizontal_offset_mm() const noexcept;
    double lift_drop_mm() const noexcept;
};

}
```

### 6.3 平面控制点语义

- 起点和终点的半径、进口缓和曲线长、出口缓和曲线长均为零。
- 中间交点保留独立的进口和出口缓和曲线长；不能合并成一个字段，因为现有数据允许二者不相等。
- 中间交点半径必须为正。

### 6.4 纵断面控制点语义

- `station_m`：变坡点统一里程。
- `elevation_m`：变坡点处两相邻坡段切线的交点高程。
- `vertical_curve_radius_m`：以该变坡点为中心控制竖曲线的物理半径；`0.0` 表示该点不设置竖曲线。
- 纵断面起点和终点的竖曲线半径固定为零。
- 第一版公共契约不暴露旧代码中“小于 100 时改作特殊抛物线参数”的重载编码；公共字段始终表示米制物理半径。旧工程中该特殊编码只作为兼容性调查项，不作为第一版受支持输入。

## 7. 公共操作

### 7.1 平纵断面重构

```cpp
ReconstructionResult reconstruct(
    const std::vector<Point3D>& ordered_points,
    const ReconstructionOptions& options = {});
```

输出：

- 有序平面控制点：起点、各交点、终点；
- 每个中间交点的曲线半径、进口缓和曲线长和出口缓和曲线长；
- 有序纵断面控制点：起点、各变坡点、终点；
- 每个纵断面控制点的里程、高程和竖曲线半径；
- 各输入点投影到重构平面线路后的里程；
- 状态和诊断信息。

`ReconstructionOptions` 显式承载旧工程原先来自界面、规范对象或全局变量的阈值与限制，包括平面直曲分段阈值、拟合/优化控制、纵面二阶差商阈值候选、最小坡长、坡度限制、坡度代数差限制、竖曲线最小半径、半径取整粒度、起始里程和里程单调容差。

默认值只代表独立几何重构的推荐配置。凡声称与旧行为一致的测试，必须显式填写所有影响结果的选项。

### 7.2 联合拨量和抬落道量计算

```cpp
AdjustmentCalculationResult calculate_adjustments(
    const std::vector<Point3D>& points,
    const AlignmentDefinition& alignment);
```

对每个输入点：

1. 将 `(x, y)` 投影到平面线路最近的有效线元。
2. 得到唯一的投影里程、投影坐标和线元类型。
3. 依据平面前进切向计算带符号拨量。
4. 在同一里程处求纵断面设计高程，计算 `设计高程 - z`。
5. 按输入顺序返回 `PointAdjustment`。

该函数同时输出拨量和抬落道量；第一版不再提供一个会重复建线、重复投影的独立生产级抬落道计算器。若后续确需按已知里程查询纵面高程，可在 `VerticalAlignment` 内部测试接口稳定后另行设计，不在本目标中预先扩张公共 API。

## 8. 算法与所有权边界

### 8.1 `HorizontalReconstructor`

唯一负责：

- 平面累计距离、切向和曲率计算；
- 直曲自动分段及稳定化；
- 直线正交最小二乘拟合；
- 圆曲线拟合和相邻切线交点；
- 进出口缓和曲线长估计；
- 半径和缓长的旧目标函数优化；
- 平面几何可行性检查。

### 8.2 `VerticalReconstructor`

唯一负责：

- 将水平投影后的 `(station, z)` 作为纵面样本；
- 非等间距二阶差商及直线/竖曲线样本分类；
- 各坡段最小二乘拟合；
- 相邻坡段延长线交点形成变坡点；
- 小坡度差和短坡段合并；
- 坡度、坡长、高程一致性重算；
- 竖曲线半径拟合、限制和取整。

### 8.3 `HorizontalAlignment`

唯一负责把平面控制点构造成直线、进口缓和曲线、圆曲线、出口缓和曲线和末端直线，并提供点到平面线路的最近投影。

### 8.4 `VerticalAlignment`

唯一负责验证纵断面控制点并按里程计算设计高程。竖曲线半径为零时按相邻直坡段求值；半径非零时按受支持的竖曲线模型求值。

### 8.5 `AlignmentAdjustmentCalculator`

只负责编排 `HorizontalAlignment` 与 `VerticalAlignment`：它不复制投影、缓和曲线、竖曲线或符号计算公式。

## 9. 重构数据流

### 9.1 平面

1. 校验有限数值、点数和连续重复点。
2. 按输入顺序建立临时累计距离。
3. 按旧工程对应行为计算切向与曲率。
4. 识别并稳定直线段与曲线段。
5. 拟合直线、圆曲线和缓和曲线参数。
6. 计算切线交点及左右转向。
7. 用抽取的旧目标函数优化半径和进出口缓长。
8. 验证相邻曲线不重叠且各线元长度有效。
9. 输出平面控制点。

### 9.2 纵断面

1. 将每个 XYZ 测点投影到已重构平面线路，取得统一里程。
2. 检查投影里程在容差内单调递增；不得静默排序或丢点。
3. 形成 `(station, z)` 样本并进行直坡段/竖曲线样本识别。
4. 拟合直坡段并求相邻坡段交点。
5. 按旧流程合并小坡度差和短坡段，重算坡度、高程和坡长。
6. 对需要竖曲线的中间变坡点拟合、限制并取整半径。
7. 输出包含 `station_m`、`elevation_m` 和 `vertical_curve_radius_m` 的控制点。

## 10. 线路评估细节

- 最近投影在所有相关平面线元上产生候选点，并选择平面距离最小者。
- 投影位于线路范围之外时，平面投影夹取到起点或终点。
- 精确落在线元连接点时，采用较小里程一侧线元，保证结果确定。
- 拨量符号由前进单位切向与“投影点指向测点”的向量叉积确定，并通过测试固定为右正左负。
- 纵断面里程覆盖范围必须覆盖整个平面线路范围；第一版不静默外推纵断面。
- 同一个 `station_m` 同时用于平面结果和纵面设计高程，调用方不能分别指定两套点里程。

## 11. 校验和错误处理

以下情况返回稳定诊断码，不作为正常控制流抛异常：

- 重构点数不足；
- 坐标、方案参数或选项不是有限数；
- 连续平面点重合；
- 重构后的投影里程反向超过容差；
- 平面控制点少于两个；
- 中间交点半径非正、缓和曲线长为负；
- 切线长、缓长或相邻曲线关系导致几何不可行；
- 纵断面控制点少于两个或里程不严格递增；
- 纵断面范围不能覆盖平面线路范围；
- 竖曲线半径为负，或者落入第一版不支持的旧特殊编码区间；
- 相邻竖曲线重叠；
- 坡段或拟合退化。

公共结果包含总体状态和诊断列表。诊断至少包含严重程度、稳定代码、相关输入索引（适用时）和可读信息。库不得静默删除输入点。

## 12. 验证设计

### 12.1 确定性功能测试

- 纯直线、零拨量、单一坡段、零抬落道量；
- 左右拨量符号；
- 设计高程高于测点时为正抬道、低于测点时为负落道；
- 单圆曲线，无缓和曲线；
- 等长和不等长进出口缓和曲线；
- 多交点及线元连接点投影；
- 多变坡点、无竖曲线和有竖曲线；
- 平曲线与竖曲线同时存在时，同一投影里程驱动两项调整量；
- 线路端点外投影；
- 米与毫米换算；
- 第 11 节各类校验错误。

### 12.2 重构闭环测试

用已知平纵方案生成确定性 XYZ 测点，执行重构后检查：

- 平面控制点数量、顺序、交点坐标、半径、进出口缓长；
- 纵断面控制点数量、顺序、里程、高程、竖曲线半径；
- 将重构结果再次送入 `calculate_adjustments` 后，设计线上样本的拨量和抬落道量接近零。

### 12.3 旧行为对照

`tests/legacy_reference` 只保留受支持契约所需的最小旧公式和状态，不包含 ARX/MFC 头文件，也不链接到生产静态库。生产路径与参考路径使用相同夹具并比较：

- 平面交点、半径和两个缓长；
- 纵面变坡点里程、高程和竖曲线半径；
- 各点投影里程、投影坐标和带符号拨量；
- 各点设计高程和带符号抬落道量。

每类量单独规定容差；失败信息必须包含夹具名、字段、期望值、实际值、绝对误差和容差。

### 12.4 独立性扫描

静态库工程、生产源文件和公共头文件不得出现：`afx`、`AcDb`、`AcGe`、`ObjectARX`、`CString`、`ads_`、`aced`、`XLDataBase` 或 Excel 自动化类型。

### 12.5 构建矩阵

使用：

- Visual Studio 2026：`D:\Programs\Microsoft Visual Studio\2026`
- x64 MSBuild：`D:\Programs\Microsoft Visual Studio\2026\MSBuild\Current\Bin\amd64\MSBuild.exe`
- 平台工具集：`v145`
- C++ 标准：`C++17`

必须完成：

- `Debug|x64` 全新构建与测试；
- `Release|x64` 重新构建与测试。

## 13. 兼容和范围边界

- 旧 ObjectARX 工程只作为行为证据，本目标不修改、删除或重定向旧代码。
- 新库不追求旧二进制 ABI、自定义实体、DWG、MDB 或界面兼容。
- 不处理无序点自动寻路。
- 不绘制 CAD 实体，不注册命令，不读写数据库或 Excel。
- 不在本目标中集成回旧插件。
- 不输出综合三维移动距离；拨量和抬落道量保持为两个有方向的物理量。

## 14. 架构完整性结论

- 不变量：每个测点只有一个平面投影里程，平面拨量与纵面设计高程必须共享该里程。
- 权威所有者：平面投影属于 `HorizontalAlignment`，纵面高程属于 `VerticalAlignment`，联合编排属于 `AlignmentAdjustmentCalculator`。
- 无重复责任：联合计算器不复制几何公式；纵断面重构输出能够完整回灌到线路定义。
- 结论：采用联合评估接口；竖曲线半径是公共纵面控制点的必要字段；第二功能必须同时输出拨量和抬落道量。

## 15. 成功证据与停止条件

只有同时满足下列条件才可声明完成：

1. `AlignmentReconstruct.sln`、静态库和测试工程位于指定目录。
2. 两个公共操作均完成真实实现。
3. 重构输出完整包含平面参数和纵面里程、高程、竖曲线半径。
4. 联合计算逐点输出投影里程、投影坐标、设计高程、拨量和抬落道量。
5. 确定性、闭环和旧行为对照测试通过。
6. `Debug|x64`、`Release|x64` 构建及测试通过。
7. 依赖扫描证明无 ARX/MFC 泄漏。
8. 聚焦架构与契约审查没有未解决漂移。

实现存在但证据不全时状态为 `needs-verification`；工具链或必需外部输入不可用时为 `blocked`；若继续需要修改旧插件或扩大公共契约，则为 `scope-exceeded`。

## 16. 实施顺序与交付物

实现 Agent 必须按以下顺序推进，前一阶段通过测试后再进入下一阶段：

1. 建立 `AlignmentReconstruct.sln`、静态库、测试工程和公共值类型；先完成无 ARX/MFC 依赖扫描。
2. 实现纯 C++ 基础几何、typed horizontal elements、`getBPMTL` 对照和按里程求点。
3. 实现 `HorizontalAlignment` 最近投影与拨量，先用人工已知线路夹具对照 `PROJ_ML`/`CalRectification`。
4. 实现 `VerticalAlignment` 和竖曲线按里程求高程，对照 `MLToH_Bas`/`VerticleArc`。
5. 实现 `calculate_adjustments`，证明拨量和抬落道量共享同一 station。
6. 实现平面重构主链，逐阶段对照分类、直线、圆、缓长和优化结果。
7. 实现纵断面重构主链，逐阶段对照二阶差商、坡段、变坡点、半径和整饰结果。
8. 加入闭环、旧行为对照、错误诊断和样例。
9. 完成 Debug/Release x64 新鲜构建、测试、依赖扫描和架构审查。

必须交付：

- 可被其他 C++17 工程链接的 `.lib` 和公共头文件；
- 演示两个公共能力的控制台 sample；
- 确定性夹具和测试可执行程序；
- `tests/legacy_reference` 最小参考实现及来源映射；
- 每项对照的容差说明；
- Debug/Release 构建与测试记录；
- 无 ARX/MFC 依赖扫描记录。

## 17. ADR 信号

本设计建立了持久公共契约、模块所有权、依赖方向、单位和符号约定、兼容边界及测试参考隔离。实现完成并验证后，应评估是否把这些已落地决策回填为 ADR；未实现的设计本身不直接成为已接受 ADR。
