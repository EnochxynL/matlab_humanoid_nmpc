# G1 Simulink + OCS2 MEX 行走工程

Simulink/Simscape 实现观测转换、Reference/Gait、动作生成和物理系统，通过同进程 Planner MEX 调用原 C++ OCS2、Pinocchio、CppAD、HPIPM 数值内核。运行和构建均不需要 ROS2 节点、catkin 或 colcon。

## 目录边界

| 位置 | 职责 |
|---|---|
| `native/` | 独立 CMake 子项目：C++ 控制器、Planner MEX、Conan 配方与原生测试 |
| `models/g1_walking.slx` | 物理闭环、动作生成、JointDrive 与诊断 |
| `models/g1_planner.slx` | 五个内联转换/检查块与分阶段 Planner MEX 调用 |
| `models/g1_reference.slx`、`models/g1_data.sldd` | Simulink Reference/Gait、固定参数与总线 |
| `matlab/+g1/` | 路径初始化、机器人配置读取 |
| `config/`、`assets/`、`data/` | 应用运行配置、URDF、网格与机器人数据 |
| `tests/`、`tools/` | MATLAB 模型验证及日志工具 |
| `open_project.m`、`run_walking.m` | 模型打开与仿真编排 |

原生子项目内部目录结构与构建说明见 [native/README.md](native/README.md)。CMake 源目录是 `native/`；仓库根目录不再是 CMake 工程。运行时的工程根目录仍是本目录，控制器从这里读取 `config/task.info`、`config/reference.info` 和 `assets/g1_29dof.urdf`。

## 构建与运行

克隆后直接按平台执行构建脚本；依赖由 Conan 配方获取固定上游提交或校验过的发行包，无需初始化 Git 子模块。

Windows 使用 PATH 中的 MSYS2 UCRT64 GCC 和 Conan，项目不下载或维护编译器。当前验证版本为 GCC 16.2.0；先进入相应编译器环境，再执行：

```powershell
.\native\scripts\build_windows.ps1 -Stage All
```

Linux 使用 Conan 管理数值依赖，再由 CMake 构建；需先准备对应编译器的 Conan profile：

```sh
bash native/scripts/build_mex.sh
bash scripts/start_matlab.sh
```

MATLAB R2025b、Simulink、Simscape Multibody 中运行：

```matlab
open_project;       % 只打开模型
out = run_walking;  % 初始化 MEX，运行原 4.8 s 物理闭环并清理
```

MATLAB 也应从相同编译器环境启动。Windows 可使用 `addpath tools/windows; out=run_windows_walking`，将 Simulink 生成文件隔离到 `build/matlab/<工具链标记>/`。不要仅打开模型后直接点击 Run；控制器生命周期由 runner 管理。

构建中间产物位于 `native/build/`；MATLAB 使用部署目录：Windows `install/windows/lib/`，Linux `install/linux/lib/`。Conan 管理 OCS2、HPIPM、BLASFEO 及其依赖，详情见[依赖与产物边界](docs/dependency_artifact_layout.md)。MPC 60 Hz、策略查询 500 Hz，canonical time 为 `double(tick)*0.002`，查询提前量 5 ms。日志和物理证据写入 `runtime/walking_*/`，状态文件为 `runtime/current_run.json`。

## 验证与历史

- [系统 UCRT64 GCC 迁移与验证](docs/ucrt64_toolchain_migration.md)
- [Windows 迁移检查点](docs/windows_checkpoint_20261017.md)
- [转换内核内联及行走验证](docs/native_matlab_adapters.md)
- [原生子项目整理与旧 gait 调用链审计](docs/native_subproject_migration.md)

旧构建缓存和验收证据保留在被忽略的 `native/build/`、`runtime/` 中；历史源码位置以对应提交为准。Conan 配方只构建所需数值模块，不构建上游 ROS 示例。项目 OCP 公式、任务参数、物理模型及 MEX 协议保持不变。
