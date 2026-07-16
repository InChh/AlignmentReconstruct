# 平面重构活跃旧路径毫米级迁移 - Reflection

## Outcome

本任务按已框定的停止条件完成：现场 22 个控制点数量和类型完全一致，坐标、半径、进口缓长和出口缓长四类最大误差均小于 `0.001 m`，Debug/Release、拨量、抬落道量和纵断面回归均通过。

## Key Judgment

平面算法权威不是 `CH_V_Cal` 的 `_SW` 副本，而是“全线坐标拟合”对话框两个按钮当前可达的 `CCeDianNiHe` 路径。先纠正权威入口，再按源调用顺序迁移，避免继续调试错误的近似 owner。

## Avoided Misfix

- 没有放宽 `0.001 m` 容差。
- 没有用 MDB 结果硬编码、夹具特征分支或第二套 fallback 掩盖误差。
- 没有把注释代码、`New`/`opti`/单交点实验路径当成当前行为。
- 没有保留 Nelder-Mead、局部目标样本或曲率排名启发式作为兼容后门。

## Complexity and Retirement

复杂度增加集中在 `HorizontalReconstructor` 的现有内部 classifier/optimizer owner，以及三个窄范围纯数学 helper；公共入口和公共结果类型未增加。旧近似 classifier/optimizer 已 delete-first 退役，生产符号扫描无残留，不存在双 owner。

## Residual Risk

本次证据没有覆盖少于 3 个圆样本时的 `BCTTP`/`BCTPP` 稀疏分支；后续若要支持该边界，应继续迁移旧类的活跃纯数学路径并建立专门 fixture，不能自行选择替代圆算法。

Method Pack 记录只保存决策和证据，不替代实际测试退出码。
