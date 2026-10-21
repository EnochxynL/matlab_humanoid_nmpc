# OCS2 MATLAB 直接绑定原型

当前 MATLAB/Simulink 执行链已切换到此绑定；`g1_ocp_mex` 保留用于独立 C++ 回放和对照。
目的是验证：MATLAB 能否直接编排问题接口、SQP 和 MRT，并减少中间容器。
`ocs2_binding_mex` 直接链接 humanoid 问题库和 OCS2，不调用 `G1Controller`、
`PacketManager` 或 `ExternalReferenceManager`；原生诊断计算由现有诊断类提供。

迁移后的调用链为 `Simulink → g1.plannerMex → g1.PlannerBackend → ocs2.Problem → MEX`。
`PlannerBackend.m` 负责时间/阶段协议、参考解包和校验、首次 reset 目标、策略缓存，
以及求解后的参考消费检查。逆动力学仍使用独立 MEX。
绑定新增状态布局转换、模型设置、参考快照与诊断接口；原生诊断计算复用现有诊断类。
以下原型复制统计保留为迁移前的对照证据。

## 当前操作如何映射

| 当前 Controller 操作 | 原生方法 | 我们额外增加的部分 |
| --- | --- | --- |
| `initialize` | 创建 `WBMpcInterface` / `CentroidalMpcInterface`；创建 `SqpMpc` 和 `MPC_MRT_Interface`；`setReferenceManager` | formulation 字符串、路径配置、诊断装配、单实例限制 |
| `dimensions` | `getMpcRobotModel().getStateDim/getInputDim/getJointDim` | MATLAB 返回数组布局 |
| `observationFromGeneralizedState` | 质心模型 `updateCentroidalDynamics`、`getCentroidalMomentumMatrix` | 58 维运动学输入协议、全身原样返回、`A(q)v/m` 的组装 |
| `setReference` | `TargetTrajectories`、`GaitSchedule::updateModeSchedule`、`setTargetTrajectories` | 428/359 包、发布标志、tick/session/sequence、节点容量、摆腿配置签名、参考消费历史 |
| `installOcpObservation` | 构造 `SystemObservation` | 首 tick 为 0、连续 tick、重复观测处理、阶段标记、默认输入为零/模式为 3 |
| `updateMpc` | `setCurrentObservation`、`resetMpcNode`、`advanceMpc`、`updatePolicy` | token 校验、request 幂等、首次 reset 目标构造、异常后的实例失效、诊断事件 |
| `queryPrepared` | `evaluatePolicy(time,state,...)` | 阶段顺序、固定 5 ms 提前量、同 tick 缓存、`Policy` 中间类型 |
| `tickIdentity/validateTick` | 无对应 OCS2 方法 | 我们的控制时间与阶段协议 |
| `drainDiagnostics` | 部分计算使用 OCP、模型和求解结果 | 表格缓冲、传输格式、诊断事件关联 |
| `shutdown` | 原生对象析构 | MEX 对象生命周期和锁管理 |

`ExternalReferenceManager::preSolverRun` 的原生有效工作是安装目标和接触时间表，
再调用原来的 `SwitchedModelReferenceManager::preSolverRun`，后者更新地面高度和摆腿参考。
原型在 MATLAB 中先安装目标和时间表，求解器直接注册原管理器；OCS2 仍会调用它。
参考是否被改写、发布和消费是否同 tick 等额外协议没有移植到原型。

## 原型接口

命名按原生对象分组：`reference_...`、`mrt_...`、`diagnostics_...`。
下划线连接对象访问路径，具体方法沿用上游小驼峰，例如
`reference_getGaitSchedule_updateModeSchedule` 对应
`reference->getGaitSchedule()->updateModeSchedule(...)`。
生命周期、组装和模型转换单独归为绑定辅助；`disableExtension` 和 `getSnapshot`
也是绑定辅助，注释中明确标出其组成操作，不冒充上游同名方法。

```matlab
problem = ocs2.Problem(config); % config 明确指定问题类型和三个配置路径
cleanup = onCleanup(@()delete(problem));
problem.createSolver();
problem.reference_getGaitSchedule_disableExtension(); % humanoid 外部完整接触时间表模式

problem.mrt_setCurrentObservation(time, state, input, measuredMode);
problem.mrt_resetMpcNode(initialTimes, initialStates, initialInputs); % 首次求解前
problem.reference_setTargetTrajectories(times, states, inputs);
problem.reference_getGaitSchedule_updateModeSchedule(paddedEvents, paddedModes);
problem.mrt_advanceMpc();
changed = problem.mrt_updatePolicy();
[desiredState, desiredInput, mode] = problem.mrt_evaluatePolicy(queryTime, state);
stats = problem.copyStats();
```

调度、reset 目标、末尾接触标记和 queryTime 由 MATLAB 显式决定，见
`tests/binding_replay.m`。原型不接收 tick token 或扁平参考包，也没有策略缓存。
目标矩阵的每一列是一个节点，不限制只能有 1 或 3 个节点。

目前一个 MATLAB `ocs2.Problem` 对应一个原生问题及其可选 SQP/MRT；
可以同时创建多个对象，每个对象只允许组装一个求解器。这里刻意没有暴露
任意跨 MEX 指针，也没有支持多个求解器共用一个可变参考管理器。
`uint64` 句柄查表，删除后不能再使用；每个活动对象锁住 MEX，全部删除后解锁。

原生边界仍检查类型、维度、有限性、无效句柄、查询前是否有策略。
此外拒绝缺失观测/目标/新接触时间表的求解，并在求解失败后要求重建对象。
这是防止原生库访问空数组或复用失败实例；不包含旧 tick 阶段协议。
尤其 `GaitSchedule` 每次读取都会移除末尾标记，故每次求解前必须重新提交完整时间表。

## 数据复制审计

下面按数值负载的完整物化计算。检查循环、`Eigen::Map`、`unique_ptr` 移动、
容器移动和指针交换均不计为复制。次数不等于分配次数，也不代表整个求解器的内存流量。

| 数据 | 现有执行链 | 直接绑定 |
| --- | --- | --- |
| 每次观测的状态 | MATLAB→MEX `std::vector`；→prepared Eigen；→MRT 观测，共 3 次 | MATLAB→临时 `SystemObservation` Eigen；→MRT，共 2 次 |
| 查询用测量状态 | 复用 prepared Eigen，无额外整向量复制 | MATLAB→`vector_t` 1 次；当前 OCS2 参数类型是 `const vector_t&`，不能声称 Map 后零复制 |
| 每次查询的状态/输入输出 | MRT 结果→`lastPolicy`；返回 `Policy` 值；→MATLAB，共 3 次 | MRT 结果→MATLAB，各 1 次 |
| 每次发布的有效目标节点 | 解码为 vector；→Eigen；取参考快照；复制到参考缓冲；复制到消费历史，共 5 次；首次解码还复制全部容量节点 | MATLAB→OCS2 Eigen 各 1 次，随后 move 到参考缓冲 |
| 目标的 OCS2 命令快照 | `MPC_MRT_Interface::copyToBuffer` 再复制 1 次 | 相同，不能消除 |
| 目标时间数组 | 固定 3 项解码、组装目标、参考快照、缓冲、消费历史 | 有效时间数组一次复制，move 到缓冲 |
| 接触事件和模式 | 固定容量解码、有效 schedule 构造、参考快照、padding 副本、GaitSchedule、消费历史 | MATLAB padding 后→schedule 一次；`updateModeSchedule` 内再复制一次 |
| 首次 reset 目标 | 构造和 `resetMpcNode(const TargetTrajectories&)` 的缓冲复制 | MATLAB 构造；绑定复制到目标，再由同一原生 reset 方法复制到缓冲 |
| 完整求解策略 | SQP→PrimalSolution 的轨迹/控制器结果物化 | 相同；`updatePolicy` 自身交换指针，不复制完整策略 |

两条路径中，`advanceMpc` 都将 MRT 观测复制到局部求解观测，并将初始化观测复制到命令快照。
这些是 OCS2 内部行为，不归入绑定层计数。SQP 工作区、插值、动力学计算的临时量和
策略提取内部复制没有被本原型逐次插桩，不能把上表当成全进程复制总数。

原型 `copyStats()` 返回每类 `[显式物化次数, 元素总数]`：targets 包含 reset，
schedules 包含事件和模式的类型转换，observations 包含状态和输入，queryStates
是查询输入物化，policyOutputs 是状态/输入写入 MATLAB。每个复制点旁直接累加，
回放根据输入长度独立核对总数。它不计 OCS2 内部副本、标量/句柄和 MATLAB 侧数组整理。

如果仍拼接 `[state;input;mode;queryTime]` 以兼容旧向量接口，MATLAB 会多一次数组组装。
直接使用独立输出或总线可避免该兼容步骤。原型自身不做该拼接。
另外，原型接收显式 observation input，故增加 MATLAB→临时 input 的复制；旧接口在
原生侧直接生成零 input。这里以更明确的原生 API 换取额外输入传输，不能忽略这项成本。

审计依据是项目当前 Conan 固定的 OCS2 源码：`MPC_MRT_Interface.cpp`、
`MRT_BASE.cpp`、`ReferenceManager.h`、`BufferedValue.h`，以及本项目的
`PlannerMex.cpp`、`G1Controller.cpp`、`PacketManager.cpp`、`ExternalReferenceManager.cpp`。

## 构建和验证

启用 `G1_BUILD_MATLAB` 时构建 `ocs2_binding_mex` 并安装到运行库目录，
不再需要原型专用开关。编译器仍由现有工具链选择。
现有模型不改端口，`g1.plannerMex` 内部改用直接绑定。

```matlab
addpath('matlab','tools','tests');
g1.initialize();
results = runtests('tests/Ocs2BindingTest.m');
assertSuccess(results);
```

全身测试使用已有 2401 tick 冻结观测和参考。质心测试使用同一运动学观测，
经既有转换得到质心状态，并将已有目标重新采样到 80 Hz 发布时刻、采用质心摆腿配置。
每种问题的旧 Controller 和原型接受完全相同的输入，逐值比较查询结果。
质心这个测试是接口等价性回放，不是之前真实质心行走日志的复现。
测试还覆盖重复查询、多实例、删除后的句柄、错误维度、非有限输入和求解前查询。
证据写入 `runtime/evidence/ocs2_binding/`。

本次验证结果：三个参数化/边界测试全部通过。两种问题均为 2401 tick，
全身 289 次、质心 385 次求解；两条执行链的策略逐值相同，最大差异为 0。
重复查询逐值相同，各类计数与按输入形状独立计算的预期完全一致。

以下为原型完整回放的显式复制统计，单元格是“物化次数 / 元素数”；
计数在重复查询测试之前读取，targets 包含首次 reset：

| 负载 | 全身 | 质心 |
| --- | --- | --- |
| 目标时间、状态、输入 | 2022 / 81404 | 2690 / 81792 |
| 接触事件、模式 | 578 / 6767 | 770 / 9011 |
| 观测状态、输入 | 4802 / 223293 | 4802 / 168070 |
| 查询输入状态 | 2401 / 139258 | 2401 / 84035 |
| 策略状态、输入写回 MATLAB | 4802 / 223293 | 4802 / 168070 |

回放循环同时运行两个求解器，没有独立测量各自耗时，因此没有性能提升结论。

迁移将时间、阶段和失败处理放在 MATLAB 中，原生层不再维护应用 tick 协议。
旧独立 C++ 实现暂时保留作对照，MATLAB/Simulink 不再通过它运行。

迁移版本已补齐诊断 API 和状态布局转换。为在 MATLAB 中保持参考消费检查，
每次求解后读回原生目标和接触时间表，因此新增了一次参考快照传输；
上面的原型计数不包含这次传输，也不包含诊断数据传输，不能直接作为生产路径总数。
目前尚未提供通用 OCP 构造；逆动力学保持独立，也未添加新的 Simulink 块。
目前不能称为完整的 OCS2 toolbox，也不能据此承诺求解速度提高。

## 执行链迁移验证

迁移后的 `g1.plannerMex` 已完成以下检查：

- 外部任意求解请求、重复阶段幂等、token/时钟检查、失败后拒绝旧策略和重新初始化通过。
- 诊断输出初始化失败会释放绑定和逆动力学实例；关闭诊断时不创建应用日志目录。
- 实际全身 planner 模型完整回放 2401 tick / 289 次求解，策略、力矩及动作总线正确；
  `planner_samples.csv` 与迁移前 Windows 基线逐值完全一致。
- 质心执行协调层完整回放 2401 tick / 385 次求解，策略与原绑定对照基线逐值完全一致。
- 两种算法分别通过 0.1 s 真实多体闭环，策略和力矩与迁移前相同，动作映射误差为零；
  全身闭环还逐值核对基座位置相同。未重新进行完整行走视觉验收。

运行 `validate_binding_migration`、`validate_centroidal_binding_migration`、
`validate_binding_walking` 可复核。证据位于 `runtime/evidence/binding_migration/`。
`G1Controller` 与旧 MEX 仍保留供独立 C++ 和回放对照使用，不在 MATLAB/Simulink 生产调用链中。

首批诊断表已迁到 MATLAB：`reference`、`gait_queries`、`timing_trace`，以及
`logging_load` 空表头和已有 `adapter_consumption` 的 schema。
绑定删除了只负责写行的 `diagnostics_trace`、`diagnostics_reference` 命令。
`timing_trace` 的会话相对时钟见 `diagnostic_clock.json`；逐表边界和验证记录见
[诊断表清单](diagnostics_inventory.md)。
