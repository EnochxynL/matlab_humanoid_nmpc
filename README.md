# G1：最后一次成功行走的精简源码

## 新入口：Simulink 数据流 + 进程内 MEX

```bash
bash scripts/build_mex.sh
bash scripts/start_matlab.sh
```

```matlab
cd('/media/enoch/DISK/MY_WORK/contribute/matlab_humanoid_nmpc_1');
build_inprocess_model; % 修改模型构建源码后才需要重新生成
out = run_inprocess_walking;
```

打开 `models/g1_inprocess_walking.slx` 可以直接看到完整控制链：

```text
Simscape → StatePreparation → Command → ReferenceGait
    ↑                                ↓
JointDrive ← ActionGeneration ← Planner (g1_ocp_mex)
```

`Command`、关节映射、目标位置/速度、PD 增益和动作总线使用普通数据流块；状态转换及 Reference/Gait 数学嵌入 MATLAB Function 块。原物理接触和姿态转换仍使用少量 MATLAB 函数。`Planner` 在同一 MATLAB 进程调用 OCS2/Pinocchio/CppAD：SQP、策略查询及逆动力学保留 C++，不重新实现求解器。

运行时不启动 ROS2 节点，不收发 ROS2 消息，不通过参考文件交换数据。ROS2/ament 仍是已有 C++ 库的构建依赖，因此不是完全去除 ROS2 安装依赖。新入口无需 `build_messages`，无需启动 `scripts/controller.py`。原 ROS2 模型与启动方式保留在下面，作为备用。

MPC 为60 Hz、策略/动作查询为500 Hz，物理 tick 为2 ms，查询提前量为5 ms；Reference/Gait、JointDrive 和控制数学保持原成功版本。`run_inprocess_walking` 使用真实 Simscape 推进0–4.8 s，并保存物理、动作、策略及时间日志到 `runtime/inprocess_walking_*/`。它检查状态有限性、基座高度/倾角、动作映射，并保留 Mechanics Explorer 回放。首次 AD 初始化与在线执行耗时分开记录。

`build_mex.sh` 串行构建并启用 `-fPIC`。生成文件留在 `build/`、`install/`，不会替代源码。MEX 生命周期由运行入口管理；异常后可调用 `g1_ocp_mex('close')`，然后重新运行。新模型的本次编译与行走验收结果以对应运行目录的 `verification.json` 为准，历史成功不代表重构自动通过。

本机 Jazzy Pinocchio/Coal 与 MATLAB R2025b 自带的 C++/OctoMap 库版本不同，已有普通 MATLAB 会话不能直接加载扩展。`start_matlab.sh` 为新会话预加载匹配的系统依赖，避免同名库冲突，不修改系统库或 MATLAB 安装。启动前先保存并关闭原 MATLAB 会话，避免多个大型实例。脚本打开模型并共享会话，不自动开始仿真。

这是已经完成 **4.8 s Simscape 行走并通过人工视觉验收**的 `pre-rewrite-baseline` 提取版。

- 基线提交：`7565b42006be8b0f8c7b21f441f50c7b773431d9`。
- 稳定控制器源提交：`8123d4f5b15ee4b1056e90ff2415ea362c493d37`。
- 控制链：**OCS2 SQP + Pinocchio + CppAD**，保留当时已验收的16个 MATLAB 数学单元。Reference/Gait 在 Simulink 中生成，JointDrive 在 Simscape 中执行。
- 原有轻微偏行保留。这里没有加入后续 MATLAB-native、CasADi、fmincon、稀疏 SQP 或 RTI 实验。

本目录只保留运行与重新构建需要的源码、数学生成源码、模型、机器人网格和配置。源码提取不包含旧实验数据、视频、审计报告、改进记录或对原工程的软链接。构建和运行后产生的文件分别位于 `build/`、`install/`、`runtime/`，这些目录不属于源码。原工程仍保留在原位置。

| 目录/文件 | 用途 |
|---|---|
| `matlab/` | 总线、物理配置、关节驱动与 ROS2 桥、Reference/Gait、已验收数学函数 |
| `models/` | 已验收的 G1 整机 Simscape/Simulink 模型 |
| `cpp/` | 成功基线的控制器、策略查询、逆动力学、时间接口及数学接入 |
| `generated/` | 已验收 MATLAB 数学的 C++/头文件，可直接构建 |
| `third_party/` | OCS2、机器人接口、BLASFEO、HPIPM 的必要构建依赖源码 |
| `ros2_ws/src/` | 自定义消息定义与硬件通信桥源码 |
| `assets/`、`config/`、`data/` | 网格、URDF、任务/参考/步态和物理参数 |
| `CMakeLists.txt`、`scripts/` | 本目录自己的构建和启动入口 |

## 首次构建

沿用本机已安装的 MATLAB R2025b、Simscape Multibody、ROS Toolbox、ROS2 Jazzy、C++/colcon、Pinocchio、Boost、Eigen、absl、GLPK 等依赖。生成 ROS2 MATLAB 绑定需要相应工具箱和编译器。

终端：

```bash
cd /media/enoch/DISK/MY_WORK/contribute/matlab_humanoid_nmpc_1
bash scripts/build.sh
```

脚本顺序构建依赖与控制器，使用本目录的 `build/` 和 `install/`，不读取原工程的构建目录。BLASFEO/HPIPM 使用已抽取的固定源码，无需重新拉取仓库。

MATLAB（首次）：

```matlab
cd('/media/enoch/DISK/MY_WORK/contribute/matlab_humanoid_nmpc_1');
build_messages;
```

建议新 MATLAB 会话运行此项目，避免旧工程中同名模型、类或 ROS 消息 MEX 仍驻留。不会自动关闭其他工程。

## 运行行走并打开三维视图

先在终端启动 C++ 控制器：

```bash
python3 scripts/controller.py start walking
```

等到 `READY` 后，在 MATLAB 中执行：

```matlab
cd('/media/enoch/DISK/MY_WORK/contribute/matlab_humanoid_nmpc_1');
out = run_walking;
```

真实物理时域是0–4.8 s，前进命令在0.5 s 开始，保留原参考和接触切换。模型窗口与 Mechanics Explorer 会打开，完成后保留回放。结果写入独立的 `runtime/walking_.../`。首次 CppAD 初始化可能重新生成和编译内核，启动等待上限600 s；需要至少3.5 GiB可用内存。

完成或取消后停止本项目的控制器：

```bash
python3 scripts/controller.py stop
```

该命令只针对本项目记录且核验过的控制器 PID，不终止 MATLAB 或其他程序。若要先站立，使用 `start standing`，随后同样调用 `run_walking`，此时运行1 s站立。

## 本次提取核验

已核对：C++ `simscape` 控制函数保持原样，MATLAB 运行数学和生成数学源码未改变；Simscape 序列化内容只改变了网格文件路径，物理拓扑和参数不变。必要路径已改为本目录资源，包装中去掉了未使用的 MuJoCo/静态重放入口。来源及包装调整见 `source_manifest.json`。

已通过依赖解析、六个数学库编译、控制器编译语法及本地依赖完整性检查。**本次未在新目录重新运行4.8 s行走**；历史成功验收和这次提取核验是两件不同的事。C++依赖解析检查暂时使用已有安装环境，完整从源码构建及 MATLAB 消息绑定生成应按上述首次构建步骤执行。

## 目录管理

业务源码在 `matlab/`、`cpp/`，第三方源码集中在 `third_party/`，数学生成源码在 `generated/`。第三方测试、示例、基准和说明文档已移除，许可证保留。CppAD 的 `example/cppad_eigen.hpp` 和 `example/eigen_plugin.hpp` 是实际编译依赖，保留原路径以兼容其头文件引用。第三方包不启用测试构建。

`build/`、`install/`、`runtime/` 都是生成目录，不应提交到源码版本库；当前构建严格使用单进程，避免内存压力。

## C++ 源码结构

```text
cpp/
├── mex/                 新入口，管理进程内规划器、参考包和策略输出
├── app/                 备用 ROS2 程序入口 main.cpp
├── controller/          当前实际使用的 WBMpcController.cpp/.h
│   └── legacy/          原时间归一化控制器；保留兼容库，不是另一个在线控制器
├── time/                物理 tick、canonical control time
├── reference/           Simulink Reference/Gait 接入及保留的参考管理接口
├── math/                已验收数学适配、几何计算图与代价/约束接入
└── diagnostics/         足端、时间、策略与控制器结构观测
```

`cpp/app/main.cpp` 是阅读入口。当前动作生成在 `cpp/controller/WBMpcController.cpp`，其类名仍为 `WaveBControlTimeWBMpcController`，以避免修改既有执行接口。`math/` 调用 `generated/` 中的数学源码，底层求解器和机器人算法来自 `third_party/`；ROS2 通信桥在 `ros2_ws/src/ros2_robot_hw_interface/`。

动态库目标名称保留，以兼容备用启动脚本和原运行配置。当前主入口改为 `build_mex.sh` 和 `run_inprocess_walking`，原入口不再是默认运行路径。
