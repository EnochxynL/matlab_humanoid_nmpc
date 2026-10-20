# Centroidal Simulink 求解与策略查询

`models/g1_centroidal_planner.slx` 是独立的查询模型，保留原有
`g1_planner` 和 `g1_walking`。在已部署启用 `G1_BUILD_CENTROIDAL` 的
MEX、并配置编译器环境的 MATLAB 中运行：

```matlab
addpath('matlab','tools');
result = run_centroidal_query();
```

示例向模型输入 50 ms 的固定站立观测和显式双支撑参考，按 2 ms
推进观测，在首次到达 80 Hz 截止时间的 tick 同步求解，每 tick 查询一次。
它不连接多体执行模型，因此通过仅证明求解与查询链路可用。

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
`CentroidalState` 输出送入求解器的 35 项观测。
输入末尾 23 项是关节速度，没有被解释成关节加速度。

## 显式参考协议

参考输入目前直接接收 `reference359`，发布 tick 必须对应实际求解 tick。
此阶段尚未移植 `g1_reference` 的命令/目标生成器。
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

`run_centroidal_query` 展示站立包的构造。模式 3 为双支撑。
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

centroidal 模式不初始化逆动力学 MEX。查询当前 tick 的策略后即可
推进下一 tick；调用 `inverse_dynamics` 明确报错。诊断仍由 MATLAB
写出通用 OCP 数据，不生成全身策略/力矩样本格式。

后续需要另行接入 centroidal 命令目标生成与执行转换，再验证站立及行走。

## 本次验证

- 完整原生构建、部署和新模型编译通过；模型无悬空信号端口。
- `run_centroidal_query`：26 tick、5 次更新，35 维观测、72 维有限策略，
  重复查询逐值一致；零速度及纯平移的动量转换检查通过。
- centroidal 错误维度、求解前查询及逆动力学误用均被拒绝。
- 原生两种实例并存、诊断开关一致性和失败后重建检查通过。
- 全身阶段与诊断验证通过；2401 tick、289 次更新的全身 Simulink 回放
  策略/力矩与此前 Windows 基线逐值一致。

证据位于 `runtime/evidence/centroidal_simulink/` 与
`runtime/evidence/centroidal_simulink_wb/`。尚未运行 centroidal 多体闭环或行走。
