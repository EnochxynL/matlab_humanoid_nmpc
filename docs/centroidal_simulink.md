# Centroidal Simulink 参考、规划与执行

`models/g1_centroidal_planner.slx` 负责求解、策略查询与力矩计算，保留原有
全身 `g1_planner` 和 `g1_walking`。在已部署启用 `G1_BUILD_CENTROIDAL` 的
MEX、并配置编译器环境的 MATLAB 中运行：

```matlab
addpath('matlab','tools');
run_centroidal_walking(1,true); % 站立验证
run_centroidal_walking();       % 4.8 s 行走
```

`g1_centroidal_walking` 引用 `g1_centroidal_reference` 与
`g1_centroidal_planner`，与全身的 walking/reference/planner 命名一致。
按 2 ms 推进观测，在首次到达 80 Hz 截止时间的 tick 同步求解，
每 tick 查询策略并计算力矩，连接真实多体模型。
阶段性纯查询模型及其专用运行脚本已删除。

## 信号与模型流程

观测沿用 `observation74`，其中包括物理时间、基座位置、wxyz 四元数、
局部线/角速度、29 个关节的位置/速度和两个接触标记。
复用原有 `ObservationConversion` 与 `ObservationExtraction` 计算
广义位置/速度并检查姿态、关节映射和时间身份。

`CentroidalObservation` 通过同一 OCP MEX 内的模型计算：

```text
x = [A(q) v / m; q]
q = [基座世界位置3; ZYX欧拉角3; MPC关节角23]
v = [基座世界线速度3; ZYX欧拉角导数3; MPC关节速度23]
```

调用 OCS2 的 `updateCentroidalDynamics` 取得与当前模型类型对应的
动量矩阵；并非把基座速度直接放入状态。该转换需要原生模型，保留在
`G1Controller`，不修改上游 `humanoid_common_mpc` 或 centroidal 辅助函数。

模型依次安装参考、登记观测、判断求解截止时间、更新 MPC、查询策略。
输出 `Policy` 为 `[state35; input35; mode; queryTime]`，共 72 项；
`Torque23` 输出 23 个关节力矩；`PolicyAndID` 输出策略与力矩汇总。
输入末尾 23 项是关节速度，没有被解释成关节加速度。

## 显式参考协议

模型输入接收 428 项完整运动学参考包，内部转换为 `reference359`；
发布 tick 必须对应实际求解 tick。下表描述安装到 OCP MEX 的内部协议。
三个节点和事件数组均为固定容量，实际使用数量由包内字段指定。

| MATLAB 索引 | 内容 |
| --- | --- |
| 1:3 | tick、publish、目标节点数（1 或 3） |
| 4:6 | 三个目标时间 |
| 7:111 | 三个 35 维目标状态，按列展开 |
| 112:216 | 三个 35 维目标输入，按列展开 |
| 217 | 接触事件数 |
| 218:281 | 64 个事件时间 |
| 282:346 | 65 个接触模式 |
| 347 | 窗口外 terminal 标记 |
| 348:355 | 与加载配置一致的八个摆腿参数 |
| 356:359 | 四项滤波命令诊断值 |

模式 3 为双支撑。
改变输入数据时，应提供相应的参考及接触时间表。

## MATLAB 与 MEX 生命周期

```matlab
cfg = g1.initialize();
config = g1.plannerConfiguration(cfg.projectRoot, logDirectory, true, 'centroidal');
g1.plannerMex('init', config);
cleanup = onCleanup(@()g1.plannerMex('close'));
```

未提供 formulation 时仍选择 `whole_body`，其参考 428 项和策略 95 项
保持原协议。`dimensions` 返回当前问题的状态、输入和关节维度。
`convert_observation` 接收 `[q;v]`；全身模式原样返回。

centroidal 默认的只查询模式不初始化逆动力学 MEX。查询当前 tick 的策略后即可
推进下一 tick；调用 `inverse_dynamics` 明确报错。诊断仍由 MATLAB
写出通用 OCP 数据，不生成全身策略/力矩样本格式。

## 力矩执行与多体模型

`g1_centroidal_reference`、`g1_centroidal_planner` 和
`g1_centroidal_walking` 提供独立的命令参考、力矩执行和物理闭环。
原全身模型继续保留。

```matlab
run_centroidal_walking(1,true) % 站立
run_centroidal_walking()       % 4.8 s 行走命令
```

参考模型复用命令滤波和步态选择，按 80 Hz 发布参考，预测窗口为 1.2 s，
摆腿参数采用 centroidal task.info 的中点值 0。为复用既有总线，目标节点
在总线中仍采用 58 项完整运动学表示；执行模型将每个有效节点转换成
35 项质心动量状态，再安装 reference359。偏航积分使用偏航角导数。
因此这些中间节点不是交给 centroidal OCP 的状态布局。

执行模型将 MPC 的接触力与实测 `[q;v]` 交给独立逆动力学 MEX。
关节加速度显式计算为 `Kp_acc*(q_des-q)+Kd_acc*(qd_des-qd)`，
默认两组增益均为零；外层关节位置/速度跟踪增益为 1200/10。
这些值来自原项目 `CentroidalMpcMrtJointController.cpp`，集中放在
`config/centroidal/execution.json`，可在独立验证中调整。

原项目注明逆动力学动量计算存在疑问，且从零观测输入取得关节速度。
这里直接使用实测广义速度，避免丢失实测关节速度；不照搬其零输入
反推速度的路径，也不将策略中的关节速度误当作加速度。
尚未引入速度差分前馈或额外 WBC 优化器。

执行需在初始化配置中显式设置 `executionEnabled=true`。
`centroidal_inverse_dynamics(policy72, measured58, acceleration23)` 只接受
当前查询策略。驱动层的关节位置和速度分别来自策略的 13:35 和 48:70，
并使用既有名称映射扩展到 29 个关节。
动作输出为 `[state35;input35;torque23;mode;queryTime]`，共 95 项，
对应 MATLAB 诊断表 `centroidal_samples.csv`。

`run_centroidal_walking` 将实际多体状态、接触力和动作写入
`runtime/centroidal_时间戳/`，并独立核对动作映射、时钟、姿态与前进距离。
数值验证通过后仍需人工观察 Mechanics Explorer。

## 本次验证

- 完整原生构建、部署和新模型编译通过；模型无悬空信号端口。
- 早期纯查询验证：26 tick、5 次更新，35 维观测、72 维有限策略，
  重复查询逐值一致；零速度及纯平移的动量转换检查通过。
- centroidal 错误维度、求解前查询及逆动力学误用均被拒绝。
- 原生两种实例并存、诊断开关一致性和失败后重建检查通过。
- 全身阶段与诊断验证通过；2401 tick、289 次更新的全身 Simulink 回放
  策略/力矩与此前 Windows 基线逐值一致。

查询阶段证据位于 `runtime/evidence/centroidal_simulink/` 与
`runtime/evidence/centroidal_simulink_wb/`。

执行接入后，1 s 多体站立闭环通过：501 个控制 tick、81 次 MPC 更新，
高度 0.7811–0.8000 m，最大俯仰 1.9735°、滚转 0.0021°，动作映射误差为零。
证据：`runtime/centroidal_20261020_214829_362/verification.json`。

4.8 s 行走闭环也通过：2401 个控制 tick、385 次 MPC 更新，向前位移
0.3290 m，高度 0.7522–0.8000 m，最大俯仰 2.3818°、滚转 3.0844°，
动作映射误差为零。计划单支撑 1300 tick；实际左脚单支撑、右脚单支撑
分别有 2617、2637 个测量样本。用户于 2026-10-20 确认人工 PASS。
证据：`runtime/centroidal_20261020_215050_665/verification.json`。

执行阶段保护、重复调用、行/列向量输入、与独立逆动力学计算的一致性
及关闭后解锁检查通过。只查询模式回归通过；全身阶段/诊断验证和
2401 tick、289 次更新的完整回放通过，策略与力矩同此前 Windows 基线
逐值一致。保护与回归证据位于 `runtime/evidence/centroidal_execution/`，
全身回放位于 `runtime/evidence/centroidal_execution_wb/`。
