# G1 Simulink + MEX 行走工程

使用已验收的 OCS2、Pinocchio、CppAD 控制器，通过同进程 MEX 接入 Simulink。保留 16 个已验收的 MATLAB 数学边界，不使用 MATLAB-native 实验求解器。运行时无需 ROS2 节点或消息通信。

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
| `math/` | 16 个已验收数学函数的 MATLAB 源定义，对应现有生成库 |
| `tools/+g1log/` | 5 个离线日志工具：收集信号、转换矩阵、导出物理 CSV、写 CSV/JSON |
| `open_project.m`、`run_walking.m` | 打开工程与仿真编排入口 |
| `cpp/mex/` | MEX 生命周期及信号接入 |
| `cpp/controller/` | OCS2 求解、策略查询与逆动力学 |
| `cpp/reference/`、`cpp/time/` | 参考接入与确定性控制时间 |
| `cpp/math/`、`generated/` | 已验收数学适配及生成源码 |
| `cpp/diagnostics/` | 控制链诊断 |
| `assets/`、`config/`、`data/` | 网格、URDF、任务与物理参数 |
| `cpp/whole_body/` | 全身 NMPC 业务数学：动力学、接触、代价、约束、参考管理和初始化 |
| `cpp/robot/` | 机器人状态、动作、模型描述和控制器基础类型 |
| `cpp/interfaces/` | 不依赖 ROS 的策略观察接口 |
| `third_party/ocs2_ros2/` | 保留的 OCS2 派生版本源码快照，原文件未改写 |
| `third_party/blasfeo/`、`third_party/hpipm/` | 底层数值库源码及许可证 |
| `cmake/NumericalLibraries.cmake` | 外部普通 CMake 适配，直接构建上述库与业务源码 |

在线升/降步态判断、步态参数查询、摆足参考、四元数转换和接触 wrench/CoP 计算均已嵌入模型。固定配置与总线由数据字典管理；模型工作区仅保留可覆盖的初始状态、重力和站立开关。模型及数据字典是源文件，不需要外部构图脚本才能打开。

日志导出、路径初始化和配置读取保留为文件，便于维护及批量运行。`math/` 是生成数学库的源定义，不是另一个在线求解器。

## 验证与归档

整理前真实 4.8 秒行走证据位于 `runtime/inprocess_walking_20261010_163754/`。整理后已通过模型编译、22 个固定参数精确对照、16 个数学源文件原样保留检查、128 个嵌入内核案例，以及利用历史真实观测进行的 4.8 秒 Reference/Gait 重放；离散字段严格相等，连续字段使用 1e-12 绝对/相对容差。后者是参考模块重放，不是新一轮物理闭环或人工视觉验收。

验证摘要保存在 `source_manifest.json`，详细本机证据在 `build/organization_validation.json`。`build/`、`install/`、`runtime/`、`slprj/` 和 `cppad_code_gen/` 不提交。首个 Git 提交保留抽取工程与重构工程，第二个仅保留重构工程；后续整理保留这两个历史提交。

## 构建边界

构建产物位于 `build/standalone/lib/`，MATLAB 从这里加载 MEX。旧 `build/controller/`、`install/` 只作为本机历史产物保留，不参与新构建。默认串行编译，以限制内存。

`third_party/ocs2_ros2/` 中仍保留上游 ROS 接口、消息包及 ament 构建文件，遵守本次“OCS2 暂时不动”的边界；当前普通 CMake 不执行这些文件，也不编译、链接这些 ROS 组件。项目自己的消息包、ROS 头文件依赖和 colcon 构建流程已移除。URDF 解析使用独立 URDFDOM。

这次结构调整尚不代表 Windows 构建通过：保留的 OCS2 CppAD 动态代码加载及部分诊断仍使用 Linux API，需要后续单独适配。

本次业务/依赖分离已完成独立普通 CMake 编译、422 个编译单元的 ROS 依赖检查、MEX 动态库依赖检查，以及清除 ROS 环境变量后的 tick0 离线查询。118 维输出与已验收的保存基线差异为零；未重新运行完整 4.8 秒物理闭环。验证摘要见 `source_manifest.json`，本机详细日志在 `build/ros_free_refactor/`。

控制链可视化重构的接口合同与当前完成边界见 [数据流边界说明](docs/dataflow_boundaries.md)。策略查询/逆动力学接口已拆分验证，模型仍使用兼容 `step`；未完成新块图验收。
