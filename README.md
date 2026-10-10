# G1 Simulink + MEX 行走工程

控制器来自最后一个已验收的 4.8 s 行走基线：`7565b42006be8b0f8c7b21f441f50c7b773431d9`，稳定控制器源码为 `8123d4f5b15ee4b1056e90ff2415ea362c493d37`。保留 OCS2 SQP、Pinocchio、CppAD 和已验收的16个 MATLAB 数学边界，未引入后续 MATLAB-native 求解器实验。

```text
Simscape → StatePreparation → Command → ReferenceGait
    ↑                                ↓
JointDrive ← ActionGeneration ← OCS2PlannerMEX
```

Command、关节映射、目标位置/速度、PD 增益和动作总线由普通 Simulink 数据流块实现。状态转换与 Reference/Gait 算法保存在模型内的 MATLAB Function 块。SQP、策略查询和逆动力学留在 C++，通过同一进程的 MEX 调用；运行时没有 ROS2 节点、消息交换或参考文件 IPC。

## 构建和运行

需要本机 MATLAB R2025b、Simulink、Simscape Multibody、ROS2 Jazzy 构建环境、Pinocchio、C++/CMake/colcon、Boost、Eigen、absl 和 GLPK。ROS2/ament 仍是底层库的构建依赖，不再需要 MATLAB ROS 消息绑定。

```bash
bash scripts/build_mex.sh
bash scripts/start_matlab.sh
```

脚本串行构建，生成文件位于 `build/` 和 `install/`。本机 Pinocchio/Coal 与 MATLAB 自带的 C++/OctoMap 库版本不同，MATLAB 启动脚本仅为新进程预加载匹配的系统库，不修改安装。启动前保存并关闭已有 MATLAB，避免重复大型实例。

MATLAB 中：

```matlab
out = run_inprocess_walking;
```

入口运行真实 0–4.8 s Simscape 行走，0.5 s 开始前进命令，并打开 Mechanics Explorer。MPC 为60 Hz、策略查询为500 Hz，canonical time 为 `double(tick)*0.002`，查询提前量5 ms。模型、Reference/Gait、JointDrive 和控制数学沿用成功基线。

证据写入 `runtime/inprocess_walking_*/`：完整仿真输出、物理/动作/策略 CSV、时间身份、求解器诊断及独立检查结果。首次 AD 内核生成可能很慢，启动耗时与物理仿真执行耗时分别记录；生成缓存会供后续运行复用。异常后可调用 `g1_ocp_mex('close')`。

## 源码入口

| 位置 | 职责 |
|---|---|
| `models/g1_inprocess_walking.slx` | 完整控制数据流、原 PhysicalPlant 与 JointDrive |
| `models/g1_inprocess_reference.slx` | Reference/Gait 数学及状态机 |
| `build_inprocess_model.m` | 从当前重构模型中的物理/参考模块重建主控制图；无需旧 ROS 模型 |
| `run_inprocess_walking.m`、`open_inprocess.m` | 仿真、记录、检查与打开模型 |
| `matlab/` | 总线/物理配置、少量物理回调、记录工具及已生成数学的 MATLAB 原始定义 |
| `cpp/mex/` | MEX 生命周期、当前观测及直接参考信号接入 |
| `cpp/controller/` | OCS2 调度、策略查询与逆动力学 |
| `cpp/reference/`、`cpp/time/` | 参考接入与确定性时间身份 |
| `cpp/math/`、`generated/` | 已验收数学适配及其生成源码 |
| `cpp/diagnostics/` | 求解器、足端和控制链记录 |
| `third_party/` | 底层数值库必要构建源码；保留许可证 |
| `assets/`、`config/`、`data/` | 机器人网格、URDF、原任务/参考/步态与物理参数 |

仅保留重构入口。旧 ROS2 模型、启动脚本、消息包、通信桥及其专用 MATLAB 接口已移除，完整旧源码可从首个 Git 提交恢复。两个模型是源文件；Reference/Gait 的主要数学已经嵌入模型；状态机仍使用4个短 MATLAB 函数（升/降步态判断、步态参数表和摆足语义），保留这些实际依赖。`matlab/` 中保留的数学定义对应 MEX 使用的生成数学库，不是另一个在线求解器。

`build/`、`install/`、`runtime/`、`slprj/` 和 `cppad_code_gen/` 不提交。历史行走 PASS 与本次重构验收分别记录；本次结果以运行目录的 `verification.json` 和人工视觉结论为准。
