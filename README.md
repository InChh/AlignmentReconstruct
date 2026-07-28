# AlignmentReconstruct

`AlignmentReconstruct` 是面向普通 C++ 调用方的铁路线路数值静态库。它接收调用方已经准备好的内存数据，不读取 DWG、MDB、Excel，也不依赖 AutoCAD、ObjectARX 或 MFC。

库提供两个公共入口：

| 调用方需求 | 使用入口 |
| --- | --- |
| 根据按线路方向排列的 XYZ 测点恢复平面和纵断面方案 | `reconstruct(...)` |
| 根据完整平纵方案计算测点的里程、拨量、设计高程和抬落道量 | `calculate_adjustments(...)` |

公共头文件位于 `include/alignment_reconstruct/`，静态库文件名为 `AlignmentReconstruct.lib`。

## 1. 获取代码

使用 Git 获取源码：

```bash
git clone https://github.com/InChh/AlignmentReconstruct.git
cd AlignmentReconstruct
```

如果不使用 Git，也可以在 [GitHub 仓库主页](https://github.com/InChh/AlignmentReconstruct) 选择 `Code -> Download ZIP`，解压后进入 `AlignmentReconstruct` 根目录。

## 2. 调用前需要知道的约束

- 平台：Windows x64。
- 工具集：Visual Studio 2026，项目文件使用 `v145`。
- 语言标准：C++17 或更高。
- 生产依赖：仅 C++17 标准库。
- 坐标约定：`x = N（北坐标/纬距）`，`y = E（东坐标/经距）`，`z = 高程`。
- 主单位：坐标、里程、半径、缓和曲线长、高程、拨量和抬落道量全部使用米。
- 二进制配置：调用工程必须与库保持相同的 x64、Debug/Release 和 MSVC 运行库配置。不要把 Debug 库链接到 Release 程序，反之亦然。
- 公共类型使用 `std::vector` 和 `std::string`，因此它是 MSVC C++ ABI，不是稳定的 C ABI。调用方更换不兼容工具集或运行库设置时，应重新编译本库。

静态库不需要初始化函数，也不需要复制 DLL。调用方负责文件读取、坐标转换、数据持久化和界面显示。

## 3. 构建静态库

### 3.1 使用 Visual Studio

1. 用 Visual Studio 2026 打开 `AlignmentReconstruct.sln`。
2. 选择 `Debug | x64` 或 `Release | x64`。
3. 生成项目 `AlignmentReconstruct`。

输出文件：

- Debug：`build/Debug/AlignmentReconstruct.lib`
- Release：`build/Release/AlignmentReconstruct.lib`

### 3.2 使用命令行

先确保 Visual Studio 的 `MSBuild.exe` 已加入当前 Git Bash 的 `PATH`，然后在库根目录执行：

```bash
MSYS2_ARG_CONV_EXCL='*' MSBuild.exe \
  AlignmentReconstruct.sln /m /t:Build \
  /p:Configuration=Release /p:Platform=x64
```

需要调试版本时，把 `Release` 改为 `Debug`。

## 4. 把库接入调用工程

### 4.1 推荐方式：在同一个解决方案中引用项目

1. 在调用方解决方案中选择“添加 → 现有项目”。
2. 添加 `AlignmentReconstruct.vcxproj`。
3. 在调用方 EXE/DLL 项目中添加对 `AlignmentReconstruct` 的项目引用。
4. 在调用方的“C/C++ → 常规 → 附加包含目录”中添加：

```text
<AlignmentReconstruct根目录>\include
```

项目引用负责确定构建顺序并链接静态库。调用方仍需显式配置公共头文件目录。

如果调用方本身也是静态库，最终生成 EXE 或 DLL 的项目仍必须链接 `AlignmentReconstruct.lib`。

### 4.2 使用预编译 `.lib`

在调用方 EXE/DLL 项目中配置：

| Visual Studio 属性 | Release 示例值 |
| --- | --- |
| C/C++ → 常规 → 附加包含目录 | `<AlignmentReconstruct根目录>\include` |
| 链接器 → 常规 → 附加库目录 | `<AlignmentReconstruct根目录>\build\Release` |
| 链接器 → 输入 → 附加依赖项 | `AlignmentReconstruct.lib` |
| C/C++ → 语言 → C++ 语言标准 | ISO C++17 或更高 |
| 配置管理器 → 平台 | x64 |

Debug 配置应改用 `build/Debug/AlignmentReconstruct.lib`。如果调用方使用 `/MT`、自定义迭代器调试级别或其他非默认运行库设置，应使用相同设置重新编译本库，避免 `LNK2038` 等 ABI 不匹配错误。

调用代码通常只需要：

```cpp
#include <alignment_reconstruct/reconstruction.h>
#include <alignment_reconstruct/adjustment.h>
```

## 5. 五分钟可运行示例

下面的程序构造一条 2 km 直线和一条 5‰ 纵坡，然后计算两个测点的拨量与抬落道量：

```cpp
#include <alignment_reconstruct/adjustment.h>

#include <iostream>
#include <vector>

int main() {
    using namespace alignment_reconstruct;

    AlignmentDefinition alignment;
    alignment.start_station_m = 1000.0;
    alignment.horizontal = {
        {HorizontalControlPointKind::start,
         {0.0, 0.0}, 0.0, 0.0, 0.0},
        {HorizontalControlPointKind::end,
         {2000.0, 0.0}, 0.0, 0.0, 0.0}
    };
    alignment.vertical = {
        {1000.0, 100.0, 0.0},
        {3000.0, 110.0, 0.0}
    };

    const std::vector<Point3D> measured_points{
        {500.0, 1.5, 102.45},
        {1500.0, -0.8, 107.55}
    };

    const AdjustmentCalculationResult result =
        calculate_adjustments(measured_points, alignment);

    if (!result.ok()) {
        for (const Diagnostic& diagnostic : result.diagnostics) {
            std::cerr << diagnostic.message << '\n';
        }
        return 1;
    }

    for (const PointAdjustment& point : result.points) {
        std::cout << "index=" << point.point_index
                  << ", station=" << point.station_m
                  << " m, offset=" << point.horizontal_offset_mm()
                  << " mm, lift/drop=" << point.lift_drop_mm()
                  << " mm\n";
    }
}
```

预期核心结果：

```text
index=0, station=1500 m, offset=1500 mm, lift/drop=50 mm
index=1, station=2500 m, offset=-800 mm, lift/drop=-50 mm
```

这也可以作为调用方完成链接后的第一项冒烟检查。

## 6. 坐标、里程和正负号

### 6.1 坐标与里程

| 字段 | 语义 |
| --- | --- |
| `Point3D::x` | 北坐标 `N`，单位 m |
| `Point3D::y` | 东坐标 `E`，单位 m |
| `Point3D::z` | 实测高程，单位 m |
| `AlignmentDefinition::start_station_m` | 平面线路起点里程，单位 m |
| `VerticalControlPoint::station_m` | 与平面线路共用的统一里程，单位 m |

库只处理数值里程，不解析或格式化 `K1413+456.789` 字符串。如果调用方把 `K1413+000` 表示为 `1413000.0 m`，则应把 `start_station_m` 设置为 `1413000.0`；所有输出里程都会沿用这一数值基准。

`VerticalControlPoint::station_m` 不是点序号，也不是输入测点折线累计长。它必须与平面线路计算出来的 station 使用同一基准。

### 6.2 拨量

`horizontal_offset_m` 表示测点相对输入方案线的位置偏差：

- `> 0`：沿里程增加方向观察，测点位于方案线右侧。
- `< 0`：测点位于方案线左侧。
- `== 0`：测点位于方案线上。

`horizontal_offset_mm()` 只是 `horizontal_offset_m * 1000` 的便捷值。

如果调用方业务中的“拨道指令”表示“把实测点移动到方案线”的动作方向，那么动作方向与上述位置偏差方向相反。库返回的是测点相对方案线的带符号位置偏差，业务层应明确是否需要转换。

### 6.3 抬落道量

```text
lift_drop_m = design_elevation_m - measured_z
```

- `> 0`：设计高程更高，需要抬道。
- `< 0`：设计高程更低，需要落道。
- `lift_drop_mm()`：米值乘以 1000。

拨量和设计高程始终使用同一个平面投影 `station_m`，不会各自计算两套里程。

## 7. API 一：根据 XYZ 测点重构线路

### 7.1 函数签名

```cpp
ReconstructionResult reconstruct(
    const std::vector<Point3D>& ordered_points,
    const ReconstructionOptions& options = {});
```

### 7.2 输入要求

`ordered_points` 必须满足：

1. 至少两个点。
2. 所有 `x/y/z` 都是有限数。
3. 点按线路起点到终点方向排列。
4. 相邻点的 XY 不能完全重合。
5. 投影到重构线路后的 station 必须严格递增。
6. 直线、缓和曲线、圆曲线和坡段应有足够采样点；库不会为无序点自动寻路，也不会静默删点或重排。

建议保留测量原始顺序，并让每种线形区段包含多个测点。采样明显稀疏时，应结合采样间距调整 `curvature_window_m` 和分类阈值，并用已知线路做回归验证。

如果只有 N/E、没有真实 z，可以使用常量 z 临时验证平面重构，但由此得到的纵断面没有业务意义，不能当作真实纵断面成果。

### 7.3 调用示例

```cpp
#include <alignment_reconstruct/reconstruction.h>

#include <iostream>
#include <vector>

using namespace alignment_reconstruct;

std::vector<Point3D> make_ordered_points() {
    std::vector<Point3D> points;
    for (int index = 0; index <= 100; ++index) {
        const double distance_m = 20.0 * static_cast<double>(index);
        points.push_back({
            distance_m,
            0.0,
            100.0 + 0.005 * distance_m
        });
    }
    return points;
}

int main() {
    ReconstructionOptions options;
    options.start_station_m = 1413000.0;

    const ReconstructionResult result =
        reconstruct(make_ordered_points(), options);

    if (!result.ok()) {
        for (const Diagnostic& diagnostic : result.diagnostics) {
            std::cerr << "input_index=" << diagnostic.input_index
                      << ", message=" << diagnostic.message << '\n';
        }
        return 1;
    }

    for (const HorizontalControlPoint& point : result.horizontal) {
        std::cout << "horizontal: x=" << point.position.x
                  << ", y=" << point.position.y
                  << ", R=" << point.radius_m
                  << ", Lin=" << point.entry_spiral_length_m
                  << ", Lout=" << point.exit_spiral_length_m << '\n';
    }

    for (const VerticalControlPoint& point : result.vertical) {
        std::cout << "vertical: station=" << point.station_m
                  << ", elevation=" << point.elevation_m
                  << ", Rv=" << point.vertical_curve_radius_m << '\n';
    }
}
```

### 7.4 输出字段

`ReconstructionResult`：

| 字段 | 说明 |
| --- | --- |
| `status` | 整体状态，`success` 表示成功 |
| `horizontal` | 平面起点、中间交点和终点 |
| `vertical` | 纵断面起点、变坡点和终点 |
| `projected_stations_m` | 每个输入点在重构平面线路上的投影里程；顺序和数量与输入一致 |
| `diagnostics` | 错误位置、错误码和可读信息 |

平面控制点：

| 字段 | 说明 |
| --- | --- |
| `kind` | `start`、`intersection` 或 `end` |
| `position` | 起点、交点或终点的平面坐标 |
| `radius_m` | 中间交点的圆曲线半径；起终点为 0 |
| `entry_spiral_length_m` | 进口 Euler 缓和曲线长；起终点为 0 |
| `exit_spiral_length_m` | 出口 Euler 缓和曲线长；起终点为 0 |

纵断面控制点：

| 字段 | 说明 |
| --- | --- |
| `station_m` | 统一里程 |
| `elevation_m` | 相邻直坡段延长线的交点高程 |
| `vertical_curve_radius_m` | 物理圆竖曲线半径；0 表示该变坡点不设竖曲线 |

## 8. API 二：计算拨量和抬落道量

### 8.1 函数签名

```cpp
AdjustmentCalculationResult calculate_adjustments(
    const std::vector<Point3D>& points,
    const AlignmentDefinition& alignment);
```

与 `reconstruct` 不同，`points` 不要求按 station 排序。每个点独立投影，输出顺序与输入顺序完全一致，`point_index` 是原输入下标。

### 8.2 构造平面方案

```cpp
AlignmentDefinition alignment;
alignment.start_station_m = 1413000.0;
alignment.horizontal = {
    {HorizontalControlPointKind::start,
     {500000.0, 3500000.0}, 0.0, 0.0, 0.0},
    {HorizontalControlPointKind::intersection,
     {501200.0, 3500000.0}, 600.0, 80.0, 70.0},
    {HorizontalControlPointKind::end,
     {501800.0, 3500900.0}, 0.0, 0.0, 0.0}
};
```

平面方案规则：

- 至少包含起点和终点。
- 第一个点应为 `start`，最后一个点应为 `end`，中间点应为 `intersection`。
- 起终点半径和缓和曲线长必须为 0。
- 中间交点 `radius_m` 必须大于 0。
- 进、出口缓和曲线长必须大于等于 0，允许不相等。
- 相邻控制点不能重合。
- 半径、转角和缓长必须能形成不重叠且连续的直线—缓和曲线—圆曲线—缓和曲线组合。

### 8.3 构造纵断面方案

```cpp
alignment.vertical = {
    {1413000.0, 100.0, 0.0},
    {1414000.0, 110.0, 10000.0},
    {1416000.0, 104.0, 0.0}
};
```

纵断面方案规则：

- 至少两个控制点，station 严格递增。
- 第一、最后一个控制点的竖曲线半径必须为 0。
- 中间点半径为 0 时表示折坡但不设竖曲线。
- 非零半径必须是大于等于 `100 m` 的物理半径。
- 半径必须填写实际物理半径，不支持用小于 100 的数值表示特殊编码。
- 竖曲线不能超出相邻坡段，也不能互相重叠。
- 纵断面起终里程必须覆盖完整平面线路范围，否则返回 `vertical_range_mismatch`。

### 8.4 读取计算结果

```cpp
const AdjustmentCalculationResult result =
    calculate_adjustments(measured_points, alignment);

if (!result.ok()) {
    // 读取 result.status 和 result.diagnostics
}

for (const PointAdjustment& point : result.points) {
    // point.station_m             平面投影里程
    // point.projected_point       方案线上的 XY 投影点
    // point.design_elevation_m    同一 station 的设计高程
    // point.horizontal_offset_m   右正左负拨量
    // point.lift_drop_m           设计高程 - 实测 z
    // point.horizontal_element    投影所在线元类型
}
```

`horizontal_element` 可能是：

- `straight`
- `entry_spiral`
- `circular`
- `exit_spiral`

### 8.5 把重构结果直接用于调整量计算

```cpp
ReconstructionOptions options;
options.start_station_m = 1413000.0;

const ReconstructionResult reconstructed = reconstruct(ordered_points, options);
if (!reconstructed.ok()) {
    // 处理诊断
}

AlignmentDefinition alignment;
alignment.start_station_m = options.start_station_m; // 不要遗漏
alignment.horizontal = reconstructed.horizontal;
alignment.vertical = reconstructed.vertical;

const AdjustmentCalculationResult adjustments =
    calculate_adjustments(points_to_check, alignment);
```

`ReconstructionResult` 本身不重复保存 `start_station_m`。调用方把重构结果装入 `AlignmentDefinition` 时，必须沿用当次 `ReconstructionOptions::start_station_m`；否则所有里程会整体错位。

### 8.6 修改重构方案后重新计算

调用方可以修改重构出来的交点、半径或缓长，再调用 `calculate_adjustments`：

```cpp
for (HorizontalControlPoint& point : alignment.horizontal) {
    if (point.kind == HorizontalControlPointKind::intersection) {
        point.position.y += 1.0; // 示例：交点向东移动 1 m
        break;
    }
}

const AdjustmentCalculationResult changed =
    calculate_adjustments(original_measured_points, alignment);
```

修改平面交点可能改变线路总长。此时必须确认纵断面 station 仍覆盖修改后的完整平面范围，并根据业务需要同步调整纵断面方案；否则接口会返回 `vertical_range_mismatch`。不要为了绕过该错误而随意伪造变坡点。

## 9. 重构参数 `ReconstructionOptions`

默认纵断面参数对应参考项目“拟合坡度参数设置”中当前既有线圆曲线方案。线路采用不同技术标准时，调用方应在调用 `reconstruct` 前覆盖相应字段。

| 参数 | 默认值 | 单位/说明 |
| --- | ---: | --- |
| `start_station_m` | `0` | 线路起点里程，m |
| `curvature_window_m` | `15` | 平面切向和曲率分析窗口，m |
| `curvature_threshold_inverse_m` | `0.00008` | 直/曲分类阈值，`1/m`；降低会把更多样本判为曲线 |
| `minimum_curve_sample_count` | `3` | 一个圆曲线分类段必须保留的最少样本数 |
| `maximum_spiral_classification_iterations` | `300` | 缓和曲线分类稳定化最大迭代次数 |
| `horizontal_optimization_tolerance` | `1e-4` | 半径和缓长 Powell 优化的相对收敛容差 |
| `horizontal_optimization_max_iterations` | `10` | 平面 Powell 优化最大迭代次数 |
| `vertical_second_difference_threshold_candidates` | `{0.001}` | 纵面二阶差商阈值。单元素表示固定阈值；传入多个候选时，库逐一完成重构并按参考项目的厘米级抬落道平方和选优 |
| `minimum_grade_length_m` | `350` | 小于该长度的短坡段会参与合并，m；0 表示不按长度合并 |
| `maximum_up_grade` | `25` | 最大上坡坡度，‰；0 表示不限制 |
| `maximum_down_grade` | `25` | 最大下坡坡度绝对值，‰；0 表示不限制 |
| `maximum_grade_difference_per_mille` | `7` | 相邻坡段最大坡差，‰；0 表示不限制 |
| `minimum_vertical_curve_grade_difference_per_mille` | `3` | 设置竖曲线的最小坡度代数差，‰；0 表示所有非零坡差均参与半径拟合 |
| `minimum_vertical_curve_radius_m` | `10000` | 竖曲线拟合初值和下限，m |
| `maximum_vertical_curve_radius_m` | `30000` | 竖曲线拟合上限，m；参考项目活跃算法的硬上限为 30000，不允许设置得更大 |
| `vertical_curve_radius_rounding_m` | `100` | 竖曲线半径向下归整步长，m；0 表示不归整 |
| `grade_rounding_per_mille` | `0.01` | 坡度归整步长，‰；0 表示不归整 |
| `station_rounding_m` | `10` | 内部变坡点里程归整步长，m；短坡二次合并和竖缓避让产生的新位置也使用该精度，0 表示不归整 |
| `station_monotonicity_tolerance_m` | `1e-6` | 判断投影里程反向的容差，m |

参数调整建议：

1. 先固定一组有权威结果的样本。
2. 一次只修改一类参数。
3. 同时比较控制点数量、交点坐标、半径、缓长、变坡点、拨量和抬落道量。
4. 不要只以“接口成功返回”作为参数合理的依据。

## 10. 错误处理

两个公共函数都通过结果对象返回状态：

| `StatusCode` | 含义 |
| --- | --- |
| `success` | 成功 |
| `invalid_input` | 输入字段、顺序、单位或方案范围不合法 |
| `infeasible_geometry` | 输入数据有限但无法构造合法平纵几何 |
| `numerical_failure` | 投影或数值求解失败 |

调用方应始终先检查 `result.ok()`，失败时遍历 `diagnostics`：

```cpp
template <class Result>
bool report_errors(const Result& result) {
    if (result.ok()) {
        return false;
    }
    for (const alignment_reconstruct::Diagnostic& diagnostic : result.diagnostics) {
        std::cerr << "code=" << static_cast<int>(diagnostic.code)
                  << ", input_index=" << diagnostic.input_index
                  << ", message=" << diagnostic.message << '\n';
    }
    return true;
}
```

常见 `DiagnosticCode`：

| 错误码 | 常见原因 |
| --- | --- |
| `too_few_points` | 重构点少于 2 个 |
| `non_finite_value` | 出现 NaN、无穷大或非法选项 |
| `coincident_consecutive_points` | 相邻 XY 或相邻平面控制点重合 |
| `non_monotonic_station` | 重构输入顺序导致投影里程不递增，或纵面 station 不递增 |
| `too_few_horizontal_control_points` | 平面方案少于 2 个控制点 |
| `invalid_horizontal_radius` | 起终点半径非 0，或中间交点半径不大于 0 |
| `invalid_spiral_length` | 缓长为负，或起终点缓长非 0 |
| `infeasible_horizontal_curve` | 曲线重叠、转角退化或几何不能闭合 |
| `too_few_vertical_control_points` | 纵面方案少于 2 个控制点 |
| `vertical_range_mismatch` | 纵面 station 范围没有覆盖完整平面线路 |
| `invalid_vertical_radius` | 端点半径非 0，或中间半径落在不支持范围 |
| `overlapping_vertical_curves` | 竖曲线超出相邻坡段或互相重叠 |
| `degenerate_fit` | 测点不足以形成稳定拟合，或拟合几何退化 |
| `projection_failed` | 测点无法稳定投影到平面线路 |

当错误不对应某一个输入点时，`Diagnostic::input_index` 为 `no_input_index`。

## 11. 常见接入问题

| 现象 | 优先检查 |
| --- | --- |
| `LNK2019`，找不到公共函数 | 是否链接 `AlignmentReconstruct.lib`，库目录和配置是否正确 |
| `LNK1112` 或 machine type 冲突 | 调用方是否误用了 Win32；本库只提供 x64 |
| `LNK2038` | Debug/Release、运行库、迭代器调试级别或工具集不一致 |
| 输出里程整体偏移 | 是否遗漏 `AlignmentDefinition::start_station_m` |
| 拨量正负与下游指令相反 | 区分“测点相对方案线的位置偏差”和“从测点移动到方案线的动作方向” |
| `vertical_range_mismatch` | 纵面起终 station 是否覆盖平面全长；修改交点后是否同步处理纵面 |
| `non_monotonic_station` | 重构测点是否按线路方向排列，是否有回头点或重复 XY |
| 曲线全部被判成直线 | 检查采样密度、坐标单位、`curvature_window_m` 和曲率阈值 |
| 纵面控制点过少或位置被归整 | 检查二阶差商阈值、`minimum_grade_length_m`、`station_rounding_m` 和坡度归整参数 |
| 输入只有 N/E | 可以验证平面，但没有真实 z 就不能得到有意义的纵断面或抬落道量 |

## 12. 运行测试和示例

构建整个解决方案后运行：

```bash
./build/Debug/AlignmentReconstruct.Tests.exe
./build/Debug/AlignmentReconstruct.Sample.exe
```

自动化测试覆盖：

- 直线、圆曲线、进出口 Euler 缓和曲线。
- 平面建线、按里程求点、最近投影和右正左负拨量。
- 圆竖曲线、变坡点、设计高程和抬落道量。
- 平纵重构闭环。
- 重构后修改交点，再使用 `calculate_adjustments` 计算原测点拨量。
- Debug/Release x64 构建。

完整可运行的双 API 示例位于 `samples/main.cpp`。

## 13. 常用文件位置与功能边界

```text
include/alignment_reconstruct/   公共头文件
samples/main.cpp                 完整调用示例
build/Debug/                     Debug 静态库、测试程序和示例程序
build/Release/                   Release 静态库、测试程序和示例程序
```

本库不提供：

- CAD 绘图、DWG 实体和 AutoCAD 命令。
- ARX/MFC 类型或运行时。
- Excel/MDB 文件读取。
- 无序测点自动寻路。
- 三维最短空间距离或综合移动量。
- 与 VS2012/v110 生成程序的二进制 ABI 兼容。
