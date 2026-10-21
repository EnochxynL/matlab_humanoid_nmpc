# 全身参考的偏航速度索引修正

`g1_reference/TargetGenerator` 的中间目标原先用 `observation.x(35)`
与偏航速度命令取平均，再积分偏航角。58 维全身运动学状态中的
`x(33:35)` 实际为 `[yaw_dot; pitch_dot; roll_dot]`，因此应读取 `x(33)`。

这是从原仓库继承的索引错误。已核查本地 `wb_humanoid_mpc` 提交
`44c7bd8fbd75075cb2c78a61b7fcbb3edcbc86ef`：

- `WBAccelMpcRobotModel::getBaseComVelocity` 返回世界系线速度及 ZYX
  欧拉角导数，所以 C++ 的 `baseVel[3]` 是偏航导数，`baseVel[5]` 是滚转导数。
- `WBMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories`
  使用 `baseVel[5]` 计算 `averageVel(2)`。
- `TargetTrajectoriesCalculatorBase::integrateTargetBasePose` 又用
  `averageVel[2]` 积分偏航角，形成滚转到偏航的错误耦合。

本次仅将全身 reference 的该索引从 35 改为 33，并更新块内注释。
保持命令、滤波、步态模板、频率、目标窗口、代价及执行增益不变。

从实际模型块代码提取函数进行检查：初始偏航 0.2 rad、命令偏航速度为零，
第一段目标时长为 `0.7*1.1=0.77 s`。

| 实测运动 | 修正后的中间偏航目标 |
| --- | --- |
| 偏航导数 0.4 rad/s，滚转导数为零 | 0.354 rad |
| 滚转导数 0.4 rad/s，偏航导数为零 | 0.2 rad |

两项检查通过，证据位于 `runtime/evidence/wb_yaw_reference_fix/acceptance.json`。
完整行走测试结果位于 `runtime/walking_20261021_004008/`，
对比数据与图位于同一检查目录的 `walking_comparison.json/.png`。

当前配置还存在与效果判断有关的条件：`config/task.info` 中基座偏航角
阶段及终端权重 `Q(3,3)=Q_final(3,3)=0`（配置使用从零开始的索引）。
偏航角速度仍有代价，此处为零的是偏航角度跟踪权重。
全身足部代价跟踪地面法向、速度和加速度，其参数
并未使用目标状态中的偏航角。因此，修正参考偏航索引不一定会改变
当前配置下的最优解；需要把参考是否变化与策略是否变化分别检查。

## 2026-10-21 行走结果

4.8 s 多体行走通过：2401 个控制 tick、289 次求解，向前位移
0.379085 m，动作映射误差为零。人工观察状态尚未确认。

| 物理指标 | 修正前 | 修正后 |
| --- | --- | --- |
| 最大绝对偏航 | 14.1179° | 14.1179° |
| 最终偏航 | 12.8644° | 12.8644° |
| 横向位移 | 0.076258 m | 0.076258 m |

已发布及新生成参考的最大差异均为 0.525217 rad，仅中间和末尾节点的
偏航分量发生变化；说明实际仿真确实使用了修正后的目标生成器。
2401 tick 的策略/力矩以及整段物理轨迹均与修正前逐值一致。

因此，这个索引错误应当修复，但它不能解释当前配置下的偏航漂移。
本次未改变偏航代价权重、足部代价或其他控制参数。

参考回归检查通过：2401 个参考包原样传输、10 项非法输入拒绝、
偏航/滚转解耦，以及 2401 tick、289 次更新的策略回放均通过。
证据位于 `runtime/evidence/wb_yaw_reference_fix_regression/acceptance.json`。

Centroidal 的模型、频率、预测窗口等也与全身方案不同，因此不能仅凭两种
步态的观感差异，认定这个索引错误是全部原因。
