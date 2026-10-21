# 诊断表清点与 MATLAB 迁移边界

本清单针对当前 `ocs2_binding_mex` 执行链；旧 `G1Controller` 仅作为对照。
首次版本仅清点；当前清单同时记录已完成的直接生成表迁移。
所有应用 CSV、JSON、TXT、BIN 文件已经由 MATLAB 写出；
后续待迁移的是其余原生采样编排、组表、统计和部分计算调用。

控制器诊断共涉及 **19 种 CSV 文件名**。旧 C++ 执行器定义 16 种 schema；
当前绑定路径只保留其中 11 种全身原生 schema，另外 5 种已由 MATLAB 定义和生成，
再加上原本就在 MATLAB 的 `time_identity`、`planner_samples`、`centroidal_samples`。
全身路径通常输出 18 种；质心执行路径没有 `structural_id`，通常输出 17 种；
质心 query-only 不生成结果样本表，通常为 16 种。
`adapter_consumption` 在当前绑定路径完全由 MATLAB 定义和生成；
`logging_load` 在 MATLAB 中仅保留表头，没有数据行。
物理模型导出的 CSV 另列于下文，不计入这 19 种。

“需要补绑定”表示 MATLAB 编排和组表完全可以迁移，但当前绑定尚未提供必要数值数据。
这些建议 API 尚未实现，不是当前已经可调用的接口。

## 逐表清单

所有名字默认追加 `.csv`。S 表示每次求解，Q 表示每次首次策略查询，T 表示每个控制 tick。

| 表 | 当前来源 / 时机 | 数据与实际含义 | 迁移方式 |
| --- | --- | --- | --- |
| `time_identity` | `DiagnosticWriter.timeIdentity` / T | raw time、tick、canonical time、两者差、固定 session 和序号 | **已经在 MATLAB** |
| `planner_samples` | `plannerMex` / 全身逆动力学完成 | tick/raw/canonical + `[state58;input35;torque23;mode;queryTime]`，结果 118 项 | **已经在 MATLAB** |
| `centroidal_samples` | `plannerMex` / 质心逆动力学完成 | tick/raw/canonical + `[state35;input35;torque23;mode;queryTime]`，结果 95 项；query-only 不生成 | **已经在 MATLAB** |
| `adapter_consumption` | `PlannerBackend.verifyConsumption` / S | 发布序号、消费 tick/time、目标/事件数及误差；session=1、epoch=0，legacy 两列恒为 0 | **已在 MATLAB**，绑定路径不再定义原生空 schema |
| `reference` | `PlannerBackend.update` / S | 当前已过滤的 `vx,vy,height,yaw_rate` 和求解时间 | **已在 MATLAB**，不再经过写行专用绑定 |
| `timing_trace` | `PlannerBackend` 主流程与首次查询 / Q、S | `event,control_time_s,wall_monotonic_time_s` | **已在 MATLAB**；统一会话相对时钟，见计时说明 |
| `gait_queries` | `PlannerBackend.query` / Q | observation/query time、观测模式、MRT 返回的策略模式 | **已在 MATLAB**；重复查询不重复写行 |
| `policy_queries` | `TimingDiagnostics.update` / Q | observation/query time、策略所属求解观测时间、策略区间、两类年龄、wall time | **补少量只读数据后迁移**：策略区间与 `getCommand().mpcInitObservation_.time`；其余是 MATLAB 算术 |
| `policy_vectors` | `TimingDiagnostics.update` / Q，按标量展开 | 实测状态、名义轨迹插值状态、名义轨迹插值输入 | **补轨迹/名义插值数据后迁移**；不能直接用反馈策略输入代替 |
| `policy_knots` | `TimingDiagnostics.postSolverRun` / S，遍历全部节点 | 完整名义状态/输入轨迹，带求解时间、节点时间、kind、零基索引和值 | **补完整策略读取后迁移**；建议矩阵传输，MATLAB 展开为表 |
| `gait_schedule` | 同上 / S | 当前求解策略的事件时间和模式，按 kind/index/value 展开 | **补策略 mode schedule 后迁移**；不要用参考管理器的计划表冒充已发布策略表 |
| `logging_load` | MATLAB 在初始化时定义 | 列为 observation time/repeat/knot count/first state norm；没有写行调用 | **MATLAB 保留空表头兼容**，本次不删 |
| `solver_timing` | `TimingDiagnostics` 求解前/后同步模块 / S | 采样起止 wall time、区间长度、策略起止时间 | **必须先确定计时语义**；保留旧边界需极小的原生时间探针，改成 MATLAB 外层计时则应另命名 |
| `structural_solver` | `StructuralObserver.postSolverRun` / S | tick、累计/本次迭代、目标值、动力学/等式/不等式 SSE、不可用的计时字段和状态占位 | **补求解器 getter 后迁移**；差分、tick、NaN 和状态文字放 MATLAB |
| `structural_dynamics` | `StructuralObserver` 的 OCP 克隆 / S 的一个内部节点 | `linearApproximation` 的 f，每个分量一行；A/B 只有尺寸，**没有矩阵元素** | **保留原生计算 API，MATLAB 选择采样并组表** |
| `structural_costs` | 同一 OCP 克隆 / S | running、input soft、state soft 各项，三者合计，以及单列的 terminal 各项 | **保留原生项求值 API，MATLAB 组表/合计**；终端项不加到 running total |
| `structural_constraints` | 同一 OCP 克隆 / S | 当前只覆盖 state-input hard equality 和 input soft 包裹约束的原始残差；不活动项写 sentinel | **保留原生残差/活动性 API，MATLAB 展开和补 sentinel**；不是全部约束家族的清单 |
| `structural_id` | `StructuralObserver.update` / 全身 Q | 23 关节的 q/qd/qdd、用名义轨迹计算的 tau、两侧各 6 项 wrench 分散在前 6 行 | **模型 getter + 原生逆动力学/加速度计算 API，MATLAB 组表**；不能直接复用实际动作的力矩 |
| `foot_kinematics` | `FootDiagnostics.write` / S 所有节点 + Q 两种采样 | 两脚位姿、线速度、mode/support、摆腿高度约束、四角最低高度 | **保留 Pinocchio/模型/摆腿规划器数值 API，MATLAB 编排和组表**；详见下文 |

## 不能混用的数据语义

### 名义轨迹与反馈策略

`MRT_BASE::evaluatePolicy` 的输入输出中，输入来自
`controllerPtr_->computeInput(queryTime, measuredState)`，可能包含状态反馈。
当前诊断 `policy_vectors` 和 `structural_id` 使用的输入则是
`LinearInterpolation::interpolate(queryTime, policy.timeTrajectory_, policy.inputTrajectory_)`。
所以它们不一定等于 `PlannerBackend.policy` 内的输入，也不一定产生相同力矩。

迁移可选择导出名义轨迹让 MATLAB 插值，或提供薄的名义轨迹插值接口。
若在 MATLAB 实现插值，必须验证事件处重复时间、边界和外推行为与 OCS2 一致。
`modeAtTime` 同样不能未经验证就用任意 `interp1` 替代。

### 策略表与参考表

当前 `reference_getSnapshot` 返回的是参考管理器的目标和 mode schedule。
`gait_schedule` 则来自求解后 `PrimalSolution.modeSchedule_`，两者属于不同对象。
完整策略的取得应对应 `mrt.getPolicy()`，策略所属观测时间对应 `mrt.getCommand()`。

两种当前配置的 `solutionTimeWindow=-1`，MRT 取完整求解窗口。
未来若缩短该窗口，MRT 保存的策略与 `postSolverRun` 中的完整结果可能有不同覆盖范围；
迁移时应明确“求解结果”还是“已安装策略”，不要将两种采样源悄悄替换。

### OCP 诊断采样点与覆盖范围

每次求解只选一个内部节点做结构诊断，零基索引为
`k=min(floor(timeTrajectory.size()/2), inputTrajectory.size()-1)`。
动力学、运行代价和约束在此点计算；终端代价在最终节点计算。
诊断使用克隆的 OCP、Pinocchio 和机器人模型，并显式请求预计算，避免改写活动求解器工作区。
迁移不应把这些只读求值直接替换为对活动 OCP 的无保护调用。

`structural_solver` 的四项内部计时目前全部为 NaN，状态也是“策略已返回，但收敛枚举及内部计时不可用”的固定说明。
迁到 MATLAB 后保留这层“不知道”，不能用外层 `tic/toc` 补成 LQ/QP 等内部耗时。

### 脚部诊断的三类采样

- `policy_knot`：每个求解节点，使用该节点的名义状态和输入；末尾输入索引取有效末项。
- `policy_query`：query time 的名义状态和输入插值。
- `actual`：实测状态，但输入仍取 query time 的名义插值；sample time 是 observation time。

其中质心模型的广义速度还需要从状态和输入恢复，因此不是只给 MATLAB 一个 q 就能复现。
`z_reference` 来自当前摆腿规划器的 `getZpositionConstraint(leg,time)`。
`min_sole_z` 是固定四点 `(-.085,±.025,0)`、`(.085,±.03,0)` 变换后的最低世界 z，
当前地面取 z=0。这是诊断几何约定，迁移时应显式保留或配置，不能当作模型自动返回的足底尺寸。

## 计时范围核对

`SolverBase` 先更新 reference manager，再调用同步模块的 `preSolverRun`。
当前 `solver_timing.start` 因而不包含前面的参考管理器更新。
`TimingDiagnostics.postSolverRun` 先计算所有节点的脚部数据，再展开 gait schedule，然后才取 end：
**该时长包含这些诊断开销**；后面的 policy_knots 标量组表和之后的 StructuralObserver 求值则不在这个区间内。
它不是纯 SQP 数值时间，也不是整个 `advanceMpc` 的墙钟时间。

MATLAB 包围 `mrt_advanceMpc` 的计时包含更大的调用范围，包括其余原生诊断、MRT 策略缓冲和边界开销。
若改用它，应另设 `advance_mpc_wall_duration_s` 等清晰字段，不沿用旧字段制造可比性。
若要保留原始模块边界，可暂留原生时间探针；保留一个时间探针不要求保留原生表格管理器。

`timing_trace` 现在全部在 MATLAB 生成，`wall_monotonic_time_s` 是从
`PlannerBackend` 诊断初始化开始的 `toc` 秒数。`diagnostic_clock.json` 声明时间原点、单位和范围。
它不能与尚在原生生成的 `policy_queries`、`solver_timing` 的 `steady_clock` 时间戳直接相减。
`policy_consumed` 现在在原生查询（包括其原生诊断计算）完整返回后记录，
比旧实现中在 TimingDiagnostics 内、结构诊断之前记录的位置稍晚；事件控制时间和顺序保留。
旧独立 C++ 执行器继续使用原时钟与原有表生成逻辑。

## 表外产物

| 产物 | 当前含义 | 迁移边界 |
| --- | --- | --- |
| `structural_snapshot.bin` | 全身最新一次查询的 218 个 double，每次 drain 后覆盖写，不是时间序列 | MATLAB 拼接和输出；还需读出模型加速度、上次结构诊断 f、求解指标及名义轨迹值 |
| `loaded_configuration.json` | 原配置文本解析结果 + 实际加载设置、关节名和 OCP 项目录 | JSON 组装迁 MATLAB；原生实际设置/项目录需 getter，不能只用重新解析 config 替代 |
| `loaded_process_maps.txt` | 当前进程已加载模块/映射，不执行加载 | 可保留一个独立原生平台查询 API；与求解器实例无必然关系 |
| `reference_authority.txt` | MATLAB 写出的固定来源标记 | 已在 MATLAB |
| `diagnostic_clock.json` | `timing_trace` 的 MATLAB 会话时钟原点、单位及事件采样位置 | 已在 MATLAB；不改变其他原生计时表的时间原点 |
| `native_physics.csv` | Simulink 物理状态、驱动力矩、接触/足底 wrench | `g1log.exportPhysics` 已在 MATLAB，与原生诊断管理器无关 |
| `controller_state_native.csv` | 物理表去除驱动和接触列后的状态子集 | 已在 MATLAB；文件名中的 native 不代表 C++ 生成 |
| `held_action.csv` | 五个 29 维驱动动作字段 | 已在 MATLAB |
| 仿真 MAT、配置/验收 JSON、图和摘要 | 运行器及离线工具生成 | 已在 MATLAB/外围工具，本轮不改 |

218 项快照布局：1:6 为版本/观测时间/查询时间/策略区间/模式；7:64 为名义状态；
65:99 为名义输入；100:122 为名义力矩；123:151 为广义加速度；
152:159 为迭代数、代价、动力学 SSE、等式 SSE 和四个 NaN 内部计时；
160 为上次结构诊断采样时间；161:218 为该点 f。快照混合了当前查询和最近一次求解的诊断，不能全部标成同一采样时刻。

## 建议的原生数据接口边界

沿现有命名分组，可逐步补充以下能力；无需把所有 C++ 内部对象都暴露成 MATLAB 对象：

1. **策略与命令只读数据**：`mrt_getPolicy`、`mrt_getCommand`，一次求解读取一次并在 MATLAB 缓存。
2. **求解指标**：暴露 `mpc.getSolverPtr()->getPerformanceIndeces()`、`getNumIterations()`；MATLAB 记录 tick 和迭代差。
3. **OCP 只读求值**：保留克隆工作区，接收明确的 t/x/u，返回动力学近似、逐项代价、残差和活动性。
4. **模型只读求值**：返回广义坐标/速度/加速度、脚部位姿速度，以及必要的逆动力学结果。
5. **摆腿参考**：暴露 reference 内 swing planner 在指定时刻的高度约束。
6. **平台与实际加载设置**：独立读取进程模块、求解器设置和项目录。
7. **可选计时探针**：仅在确实需要旧模块边界的计时时保留；不继续承担组表和文件输出。

前三类原生数据不是“诊断表”：它们是可被 MATLAB 复用的库查询/数值 API。
采样点、采样频率、表列、长表展开、统计、缓存、快照布局和文件输出由 MATLAB 负责。
`enableDiagnostics` 最终可改为按需启用这些只读工作区，而非创建一个包揽全部采样的原生管理器。

建议顺序：先迁已有输入即可生成的表；再补策略与指标读取；再迁移 OCP/模型诊断编排；
最后处理计时口径和空表兼容。每步用同一冻结回放检查数据语义，墙钟时间不要求逐值一致。

## 核对来源

- `TimingDiagnostics.h`：9 个 schema、求解/查询采样位置、名义轨迹插值和计时范围。
- `StructuralDiagnostics.h`：5 个 schema、OCP 克隆求值、全身逆动力学和 218 项快照。
- `FootDiagnostics.h`：脚部 schema、Pinocchio 计算与四点足底几何。
- `Diagnostics.h`：可选对象装配与 adapter schema。
- `PlannerBackend.m`：当前绑定路径的参考消费记录。
- `plannerMex.m`、`DiagnosticWriter.m`：时间身份、两种结果样本及所有文件输出。
- `BindingMex.cpp`：目前实际暴露的数据范围和查询诊断触发点。
- `g1log/exportPhysics.m`：物理模型 CSV 的独立导出。
- 当前 Conan OCS2 的 `SolverBase.cpp`、`MPC_MRT_Interface.cpp`、`MRT_BASE.cpp`、`ModeSchedule.cpp`：回调顺序、策略窗口、反馈输入与模式查询。

源码清点已与 `runtime/evidence/binding_migration/model_replay/` 的实际文件名核对：
全身 18 个 CSV，`logging_load.csv` 为 54 字节、只有表头。

首批表迁移验证：`validate_matlab_diagnostic_tables` 已通过。
2401 tick / 289 次求解的 `reference`、`gait_queries`、`adapter_consumption`、
`time_identity`、`planner_samples` 与迁移前逐值一致；`timing_trace` 的事件名、
控制时间和顺序一致，新墙钟列有限且单调。`logging_load` 仍为空表。
原生绑定 batch 已不含五个 MATLAB 管理的 schema；新绑定和旧 C++ 控制器均编译通过。
证据位于 `runtime/evidence/matlab_diagnostic_tables/`。
