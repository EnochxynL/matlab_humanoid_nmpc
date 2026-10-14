# G1 Simulink + MEX 行走工程

使用已验收的 OCS2、Pinocchio、CppAD 控制器，通过同进程 MEX 接入 Simulink。OCP 直接使用参考工程的 C++ 业务实现，不再通过 MATLAB Coder 生成数学替代；不使用 MATLAB-native 实验求解器。运行时无需 ROS2 节点或消息通信。

```text
Simscape → StatePreparation → Command → ReferenceGait
    ↑                                ↓
JointDrive ← ActionGeneration ← OCS2PlannerMEX
```

## 打开与运行

```bash
bash scripts/build_mex.sh
bash scripts/start_matlab.sh
```

需要 MATLAB R2025b、Simulink、Simscape Multibody，以及 C/C++ 编译器、CMake、Pinocchio/Coal、URDFDOM、Boost、Eigen、absl 和 OpenMP。构建使用普通 CMake，不使用 ROS、ament、colcon 或消息生成。本机数值依赖恰好安装在 `/opt/ros/jazzy` 下；这只是 Pinocchio/Coal/URDFDOM 的安装位置，不需要 source ROS 环境。可用 `G1_NUMERICAL_PREFIX` 指定其他安装前缀。本机 Pinocchio 的 CMake 配置还会检查 Python/NumPy，脚本默认使用系统 Python，可用 `G1_BUILD_PYTHON` 覆盖。启动脚本为新 MATLAB 进程预加载匹配的系统库；已有生成缓存可复用。避免重复启动大型 MATLAB 实例。

MATLAB 中：

```matlab
open_project;         % 打开模型，不启动仿真
out = run_walking;    % 真实 0–4.8 秒行走并打开三维窗口
```

**请通过 `run_walking` 启动，不能在仅执行 `open_project` 后直接点击 Simulink Run。** `run_walking` 负责配置控制器环境、初始化 MEX、运行仿真和退出清理；直接 Run 会报 `Initialize MEX first`。

0.5 秒开始前进命令；MPC 60 Hz、策略查询 500 Hz，时间为 `double(tick)*0.002`，查询提前量 5 ms。完整输出、策略、物理 CSV、诊断与独立检查写入 `runtime/walking_*/`；当前运行状态为 `runtime/current_run.json`。启动与执行耗时分别记录。异常后可调用 `g1_ocp_mex('close')`。

## 文件按职责归类

| 位置 | 职责 |
|---|---|
| `models/g1_walking.slx` | 控制数据流、状态转换、动作生成、JointDrive、物理系统与接触测量 |
| `models/g1_reference.slx` | Reference/Gait 状态机、命令处理、目标及摆足参考 |
| `models/g1_data.sldd` | 总线、22 个固定参数和步态转换参数表 |
| `matlab/+g1/initialize.m` | 设置工程路径 |
| `matlab/+g1/loadConfiguration.m` | 读取机器人配置及仿真初始条件 |
| `tools/+g1log/` | 5 个离线日志工具：收集信号、转换矩阵、导出物理 CSV、写 CSV/JSON |
| `open_project.m`、`run_walking.m` | 打开工程与仿真编排入口 |
| `adapters/matlab/` | 六个原 MEX 命令、协议转换和 MATLAB 实例生命周期 |
| `cpp/include/g1_controller/` | 不暴露 MATLAB/ROS/OCS2 类型的公开数据类型 |
| `cpp/controller/` | 核心实例、状态缓存、求解线程与原调度 |
| `cpp/problem/humanoid_wb_mpc/src/dynamics/` | 原逆动力学辅助计算 |
| `cpp/data/reference/`、`time/` | 原参考消费与 canonical time |
| `cpp/data/diagnostics/` | 原控制链诊断 |
| `assets/`、`config/`、`data/` | 网格、URDF、任务与物理参数 |
| `cpp/problem/` | 原全身 OCP 数学和 composition root |
| `cpp/data/observation/` | 机器人观测状态 |
| `cpp/data/action/` | 关节动作数据 |
| `cpp/data/description/` | 机器人描述、关节映射及共用数值类型 |
| `cpp/extensions/` | 自定义罚函数、软约束所有权、权重与模式元数据、六维足端接口及旋转辅助函数 |
| `cpp/interfaces/` | 内部纯 C++ 观察接口 |
| `third_party/ocs2/` | 官方 OCS2 固定提交 submodule，原文件未改写 |
| `third_party/blasfeo/`、`third_party/hpipm/` | 原版固定提交 submodule 与许可证 |
| `cmake/NumericalLibraries.cmake` | 外部普通 CMake 适配，直接构建上述库与业务源码 |

在线升/降步态判断、步态参数查询、摆足参考、四元数转换和接触 wrench/CoP 计算均已嵌入模型。固定配置与总线由数据字典管理；模型工作区仅保留可覆盖的初始状态、重力和站立开关。模型及数据字典是源文件，不需要外部构图脚本才能打开。

日志导出、路径初始化和配置读取保留为文件，便于维护及批量运行。原 `math/`、`generated/` 和 `cpp/math/` 已移除；原数学源文件和生成包装可从 Git 历史恢复，本机另有 `build/cpp_restoration/before/matlab_math_sources.tar.gz` 归档。CppAD/CppADCodeGen 仍保留，用于参考 C++ 模型的自动微分，与 MATLAB Coder 无关。

## 验证与归档

整理前真实 4.8 秒行走证据位于 `runtime/inprocess_walking_20261010_163754/`。整理后已通过模型编译、22 个固定参数精确对照、16 个数学源文件原样保留检查、128 个嵌入内核案例，以及利用历史真实观测进行的 4.8 秒 Reference/Gait 重放；离散字段严格相等，连续字段使用 1e-12 绝对/相对容差。后者是参考模块重放，不是新一轮物理闭环或人工视觉验收。

验证摘要保存在 `source_manifest.json`，详细本机证据在 `build/organization_validation.json`。`build/`、`install/`、`runtime/`、`slprj/` 和 `cppad_code_gen/` 不提交。首个 Git 提交保留抽取工程与重构工程，第二个仅保留重构工程；后续整理保留这两个历史提交。

## 构建边界

构建产物位于 `build/standalone/lib/`，MATLAB 从这里加载 MEX。旧 `build/controller/`、`install/` 只作为本机历史产物保留，不参与新构建。默认串行编译，以限制内存。

`third_party/ocs2/` 中仍保留上游 ROS 接口、消息包及 catkin 构建文件，遵守本次“上游目录原样保留”的边界；当前普通 CMake 不执行这些文件，也不编译、链接这些 ROS 组件。项目自己的消息包、ROS 头文件依赖和 colcon 构建流程已移除。URDF 解析使用独立 URDFDOM。

这次结构调整尚不代表 Windows 构建通过：保留的 OCS2 CppAD 动态代码加载及部分诊断仍使用 Linux API，需要后续单独适配。

本次业务/依赖分离已完成独立普通 CMake 编译、422 个编译单元的 ROS 依赖检查、MEX 动态库依赖检查，以及清除 ROS 环境变量后的 tick0 离线查询。118 维输出与已验收的保存基线差异为零；未重新运行完整 4.8 秒物理闭环。验证摘要见 `source_manifest.json`，本机详细日志在 `build/ros_free_refactor/`。

控制链已显式拆为 `ObservationHandling → PlannerPolicy → InverseDynamics / ActionGeneration → JointDrive → PhysicalPlant`。`PolicyDiagnostics` 保留历史输出，并记录策略状态、输入、模式和查询时间。结构图、代码与数据流的实际边界及验证证据见 [数据流边界说明](docs/dataflow_boundaries.md)。

## 独立 C++ 核心

`G1_BUILD_CORE=ON` 建立 `libg1_controller.so`；`G1_BUILD_MATLAB=ON` 额外建立 `g1_ocp_mex.mexa64`。MEX 只链接核心，不直接编译 OCP 或 controller。

```bash
# 不查找 MATLAB、不编译 MEX
./scripts/build_core.sh
# 核心 + 原 MEX 接口
./scripts/build_mex.sh
# 核验固定来源的 OCS2 子目录快照
python3 scripts/verify_dependencies.py
```

公开入口为 `g1_controller::G1Controller`：`initialize`、`setReference`、`query`、`inverseDynamics`、`step`、`shutdown`。控制器声明与实现位于 `cpp/controller/G1Controller.h` 和 `G1Controller.cpp`；公用数据类型定义在 `cpp/include/g1_controller/Types.h`，依赖只有标准 C++。`config/controller_environment.json` 当前为空，不再选择 MATLAB/Shadow/Legacy 数学路径。standalone replay driver 可脱离 MATLAB 调用核心。

核心不包含 MATLAB SDK/MEX API，也不再编译或链接 MATLAB Coder 数学库。模型、协议、solver、权重和时间语义未改变。Pinocchio 等数值包当前仍安装于 `/opt/ros/jazzy`，这是发现安装包的前缀，不是 ROS 构建工具或通信依赖。

当前使用官方 OCS2 固定提交 `26386754b8bf31ab78b503971d0cbf4fdbcd7cb4`，由 Git submodule 管理，上游源码不修改。原工程依赖 fork 特有的业务罚函数、六维足端接口等通过 `cpp/extensions/` 保留，具体边界见 `docs/official_ocs2_transition.md`。

本阶段不进行 Windows 适配或 solver 优化。验收结果见 `docs/core_extraction_acceptance.json`；本机编译、无 MATLAB 配置、fixture 和重放证据位于 `build/core_extraction/`。

依赖初始化：`git submodule update --init --recursive`。第三方源码由 submodule 管理，版本见 `third_party/dependencies.lock.json`。

官方 OCS2 接入方式和验收见 `docs/official_ocs2_transition.md`。

## 恢复参考 C++ OCP

本次恢复范围、旧严格回放差异与新物理测试结果见 [恢复报告](docs/cpp_ocp_restoration.md)。上面的历史验证记录对应当时版本，不构成本次新路径逐位等价声明。

### 控制数据流边界

- `data/observation/ObservationConversion`：观测编码、机器人状态装填、OCP 坐标和关节顺序转换。
- `data/reference/ReferenceConversion`：参考容器与既有 OCP 参考包之间的格式转换。
- `controller/WBMpcController`：`updatePolicy` 判断更新时刻并等待必要的求解完成；
  `queryPolicy` 只查询已有策略，不触发求解；`inverseDynamics` 计算前馈力矩。
- `controller/G1Controller`：按顺序协调上述阶段，保留 tick/策略身份、缓存和生命周期门禁。

`models/g1_planner.slx` 显式连接参考装填、观测准备、MPC 更新资格判断、
MPC 更新、策略查询和逆动力学。`g1_walking` 通过模型引用接入，动作生成与 JointDrive 不变。
更新判断采用核心传入的 MPC 周期；观测仍使用真实物理 tick，不被 snap 到名义 deadline。

新增 MEX 阶段接口：`prepare_observation(o74) -> tick3`、
`update_mpc(tick3, request) -> tick3`、`query_policy(tick3) -> p95`。
`tick3=[control_tick;canonical_time;mpc_period]` 是阶段执行依赖及身份凭据，不能改写。
初始化、工作线程、策略缓存和关闭仍由核心管理；旧 `query`、`step` 等协议保留。

独立模型回归入口：`tests/validate_planner_dataflow.m`（使用本机保存的 fixture 与重构前 C++ 输出）。

### 观测数据流与独立数值内核

`g1_planner/ObservationConversion` 使用原生 Selector 完成字段提取和 29→23 关节重排，
使用 Mux 组装58维 OCP 状态。三个 MATLAB Function 节点调用独立、无状态的
`g1_observation_mex`：四元数转 Euler ZYX、局部线速度转世界系、局部角速度转 Euler 导数。
这些敏感计算继续使用已验证的 C++ 数学；不在 MATLAB 中维护另一套公式。

小 MEX 的实现位于 `cpp/data/observation/ObservationKernels.h/.cpp`，适配器为
`adapters/matlab/ObservationMex.cpp`。它不初始化控制器，不链接 OCS2、Pinocchio 或控制核心。
仅构建此内核可使用：

```bash
cmake -S cpp/data/observation -B build/observation \
  -DMatlab_ROOT_DIR=/usr/local/MATLAB/R2025b \
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
cmake --build build/observation --parallel 1
```

设置 `-DG1_BUILD_MATLAB=OFF` 可仅构建纯 C++ 静态库，不需要 MATLAB。
主工程的 `scripts/build_mex.sh` 会同时构建规划器和观测两个 MEX。

新规划链通过 `prepare_ocp_observation(raw74,state58)` 将已转换观测交给核心。
核心仍执行时间、有限性、安全范围和关节映射检查；不重新计算观测坐标。
纯 MATLAB 敏感公式候选的严格 FAIL 证据保留在忽略目录 `build/observation_dataflow/`。

观测迁移后的删除整理：调度器只接收已装填的 OCS2 观测，不再重复装填 RobotState 或生成 JointAction；未使用的旧 MRT 控制器和墙钟异步循环已移除。旧 MEX 观测入口仍保留必要的 `ObservationConversion` / `RobotState` 兼容实现，因此 `cpp/data/observation` 尚不能整体删除。

`ObservationPrepare` 已拆成两个顶层节点：`ObservationExtraction` 调用独立、无状态的 `g1_observation_extract_mex(raw74, x58, jointIndices23)`，检查观测及复制字段并输出 `[tick; canonical_time; raw_time]`；`ObservationInstall` 调用 `g1_ocp_mex('install_observation', x58, identity3)`，检查调用连续性并登记观测。原有 MEX 入口仍保留兼容。可运行 `addpath tests; validate_observation_split` 检查独立提取、拒绝非法输入及完整策略回放；本次未重跑物理行走。
