# 独立控制核心：第一阶段架构分析

分析日期：2026-10-13。工程：`matlab_humanoid_nmpc_1`。起点 HEAD：`b333dde35a37f9fa11e059cd76fbf7c306d25104`。

本轮只分析和制定迁移方案。不移动生产源码，不替换依赖，不编译或运行控制器，不修改 Simulink、配置、OCP、求解器算法。用户的未跟踪文件 `cpp.zip` 保留不动。

## 1. 当前状态和实际依赖方向

当前路径为 Simulink → MATLAB Function → `g1_ocp_mex` → C++ → OCS2/Pinocchio/CppAD/HPIPM。ROS 通信组件不在该运行路径中。

当前已有普通 CMake，`scripts/build_mex.sh` 不调用 colcon、不 source ROS setup；`cmake/NumericalLibraries.cmake` 手工建立数值库目标，不执行快照中的 ament 文件。此前编译/动态链接审计记录也没有 ROS 通信依赖。因此下一阶段不是从零建立 CMake，而是把现有 CMake 改成可独立构建和安装核心的目标结构。

但目前还不是独立核心库：

- 顶层 `find_package(Matlab REQUIRED)`；即使只想构建数值控制，也要求 MATLAB 安装。
- `WBMpcController.cpp`、`GeneratedFootCost.cpp`、`GeometryInstallBridge.cpp` 直接属于 MEX target，而非独立控制库。
- `cpp/mex/PlannerContext.h` 实际组装 OCP、初始化 SQP、接入参考管理器和诊断，属于应用核心，不属于 MATLAB 数据桥。
- `PlannerMex.cpp` 还承担物理安全检查、tick 处理、机器人状态装配、首策略启动、策略/动作缓存和日志。这些领域逻辑应进核心；MEX 保留字符串命令分派、数组转换、句柄生命周期和异常转换。
- 当前 Pinocchio/URDFDOM 数值包由 `/opt/ros/jazzy` 前缀发现。它不是 ROS 通信依赖，但仍属于 ROS 分发的依赖安装来源。下一阶段要用独立依赖配置替代这个默认前缀。

证据：`CMakeLists.txt`、`cmake/NumericalLibraries.cmake`、`cpp/mex/PlannerMex.cpp`、`cpp/mex/PlannerContext.h`、`scripts/build_mex.sh`、`source_manifest.json`。

## 2. 目录分类和归属

| 当前位置 | 分类 | 迁移后的归属 |
|---|---|---|
| `third_party/ocs2_ros2/ocs2_core, ocs2_oc, ocs2_mpc, ocs2_sqp, ...` | third-party 数值库 | 固定官方 OCS2 commit；保留 upstream 源码，项目外部 CMake 包装只构建所需模块 |
| `third_party/ocs2_ros2/hpipm_catkin/` | third-party OCS2→HPIPM 接口 | 保留官方对应接口源码；建立普通 CMake target，名称中的 catkin 不代表必须使用 catkin |
| `third_party/ocs2_ros2/ocs2_thirdparty/` | 嵌套 third-party，含 CppAD/CppADCodeGen、IIT 等 | 固定具体版本和许可证，不误归为 G1 算法；明确由 OCS2 管理还是外部 provider 管理 |
| `third_party/hpipm/`, `third_party/blasfeo/` | third-party 数值库 | 固定版本、构建参数、目标架构 |
| 系统 Pinocchio/URDFDOM/Eigen/Boost/Abseil 等 | 外部 third-party 依赖 | 使用 standalone provider 或安装包配置；不依赖 ROS 包发现 |
| `cpp/whole_body/humanoid_common_mpc/` | robot-specific application | 核心的 `whole_body_ocp`：参考消费、步态语义、摆脚计算、代价、约束、初始化和机器人辅助数学 |
| `cpp/whole_body/humanoid_wb_mpc/` | robot-specific application | 核心的 G1 whole-body OCP、预计算、动力学及 OCP 组装 |
| `cpp/robot/` | application 支持 | 核心模型与观测/动作类型、URDF 模型管理 |
| `cpp/controller/` | application 控制运行时 | 核心的调度、MRT 查询、求解线程和反馈策略管理 |
| `cpp/math/` | application 数学适配 | 核心 OCP 适配层；`Generated*` 多数是手写适配类，不能当作自动生成的 third-party 文件 |
| `math/`, `generated/` | application 数学源及其生成产物 | 已验收 MATLAB 数学的原始定义和 C++ 内核；保留，不借重构删除已验收数学 |
| `cpp/time/`, `cpp/reference/`, `cpp/interfaces/` | application 协议和核心服务 | 核心的 canonical time、参考消费和观察者接口；IO 协议解码与业务参考适配分开 |
| `cpp/diagnostics/` | application 诊断 | 核心诊断接口及平台实现；保留可追溯数据，平台功能不得污染公共 API |
| `cpp/mex/PlannerContext.h` | application composition root，当前位置不正确 | 核心私有的 ControllerRuntime / OCP factory |
| `cpp/mex/PlannerMex.cpp` 中的 mxArray/mex API | MATLAB bridge | `adapters/matlab`，只转换和调用核心 |
| `matlab/`, `.slx`, runner | MATLAB/Simulink 前端 | 保持模型及调用合同不变；核心构建不依赖它们 |
| OCS2 快照中的 ROS 消息/节点/ament 元数据 | 非核心集成源码 | 不构建、不链接；未来 ROS adapter 单独依赖 ROS |

URDF、权重、gait 和参考配置是 G1 应用数据，不是第三方库的一部分。Pinocchio 对 Coal/OctoMap 等的传递依赖也要记录，但它们不应被写成我们自己的业务代码。

## 3. 官方 OCS2 的定点兼容性检查

正式来源为 [leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2)，不是给当前 fork 改名字。官方仓库同时包含 ROS 集成和数值模块；选择官方来源不等于编译 ROS 集成。

本轮从官方 main 读取到的 commit 为 `26386754b8bf31ab78b503971d0cbf4fdbcd7cb4`。这是**分析候选，不是已批准替换的依赖版本**。当前 fork 来源记录为 `manumerous/ocs2_ros2`，原 clone SHA 为 `bf9ae46f3c6dc751348d8002c03aa2f4a7cc17e7`。

按源码内容比较，忽略既有 CRLF/LF 差别：

| 文件 | 当前快照与候选官方源码 |
|---|---|
| `MRT_BASE.cpp` | 相同 |
| `SqpSettings.h` / `SqpSettings.cpp` | 相同 |
| `HpipmInterface.cpp` | 相同 |
| `CppAdInterface.cpp` | 相同 |
| `SqpSolver.cpp` | 去掉新增 `getBenchmarks()`、额外 `<fstream>` 和空白后，整个文件相同 |
| `ocs2_core/CMakeLists.txt` | 不同：fork 使用 ament，官方所查文件使用 catkin |

注意官方 SQP 实际路径是 `ocs2_sqp/ocs2_sqp/`，HPIPM 接口在 `ocs2_sqp/hpipm_catkin/`；不能沿用 fork 的扁平路径直接替换。最初按旧路径读取的 404 已在分析中纠正，不能将其解释为官方缺少 SQP。

现有 `cpp/diagnostics/StructuralObserver.h` 调用 fork 的 `getBenchmarks()`。这是已证实的 API 差异。应优先通过官方公开计时/统计接口保留可获得的信息；不支持的单轮内部时间标为 UNKNOWN，不能用累计平均值冒充，也不为保留诊断 getter 偷改 solver。

以上是少量关键文件的定点检查，不是完整兼容性证明。正式替换前仍要检查 solver 头文件、集合/penalty API、HPIPM 配套版本、Pinocchio/CppAD 接口、编译宏和生成代码路径。若发现算法差异，选择可验证兼容的官方历史 commit，或报告阻断；不能为消除差异修改 OCP 或复制 fork solver 数学到应用中。

原始检查、官方文件 SHA 和 diff 保存于 `build/core_architecture_audit/`；摘要见 `controller_core_architecture_audit.json`。

## 4. 建议的核心 API 和目标结构

结构是目标方案，尚未创建或移动这些目录：

```text
 g1_controller_core/
   include/g1_controller/       # 配置、观测、参考、策略、公开 Controller API
   src/
     whole_body_ocp/            # 应用 OCP 定义、组装与已验收数学适配
     controller/                # SQP 调用、调度、MRT、线程和缓存
     inverse_dynamics/          # 原逆动力学的独立调用边界
     robot_model/
     reference/
     time/
     diagnostics/
     platform/                  # 动态加载/路径/可选平台诊断
 adapters/
   matlab/                     # g1_ocp_mex
   ros2/                       # 未来可选，不在本轮实现
 third_party/
   ocs2/                       # 官方固定版本，只读
   pinocchio/                  # pinned provider/版本记录，不强制复制所有源码
   cppad/                      # 包含或明确配套 CppADCodeGen
   hpipm/
   blasfeo/
 cmake/dependencies/            # 独立 build wrapper 和版本锁
 models/, matlab/, math/, config/, assets/
```

`g1_controller_core` 是核心 CMake target，`OUTPUT_NAME g1_controller`，产生用户要求的 `.so` / `.dll`。不需要为了目录好看立刻为每个子目录创建独立共享库；逻辑模块可以先组成一个核心库，third-party 依赖按需链接。

建议公开 API 用明确的 G1 类型，不暴露 `mxArray`、ROS 消息或 OCS2 内部对象；通过 PImpl 隐藏 OCS2/Pinocchio。包含：

- `initialize(config)`：配置、机器人和 OCP 初始化，显式定义失败清理。
- `setReference(referencePacket)`：语义参考、tick、接触计划和摆脚参数。
- `query(observation)`：原有更新调度、策略查询和缓存。
- `inverseDynamics(policy)`：当前策略身份检查和原力矩计算。
- `step(observation)`：原 query+ID 兼容路径。
- `shutdown()`：有序停止本实例的线程、关闭资源。

核心拥有 OCP、求解器、参考管理器、机器人状态、tick/session、策略缓存和求解线程。MEX 持有一个核心实例，只处理六个原命令，转换 74/428/95/118/23 维布局、MEX 锁/退出回调以及错误映射。把数值输入长度放在协议布局定义中，核心使用有语义的观测/参考类型。物理门禁、canonical time 和策略身份属于核心合同，应对未来 ROS adapter 同样生效。

保留已有 `init/reference/query/inverse_dynamics/step/close` 命令、输入输出形状、关节顺序、错误 ID 和有效输入上的状态机行为。不能以架构重构为名新增动作裁剪、改变 contact observation、重新排序数学运算或暗改初始化。

生命周期区别：核心负责自己的资源与有序停止；MEX 负责 MATLAB 所持实例的创建、销毁和 mexLock。这两者不是重复的生命周期管理。

## 5. 纯 CMake 与无 MATLAB 核心的真实障碍

构建方向：

```text
 MATLAB discovery → g1_ocp_mex → g1_controller_core → 官方 OCS2/Pinocchio/AD/QP
 ROS discovery    → 可选 ROS adapter → 同一个核心 API
```

`G1_BUILD_MATLAB_MEX=OFF` 时不执行 `find_package(Matlab)`。数值依赖不借 ROS 的 find_package 环境；官方带 catkin 的 CMake 不直接执行，由仓库外的纯 CMake wrapper 建立固定数值目标。避免 third-party PUBLIC include 反向暴露项目 `cpp/`。MEX 只链接核心，不直接编译控制器或数学适配实现。

**MATLAB 生成数学仍有编译期耦合。** 当前多个 `generated/.../rtwtypes.h` 包含 MATLAB SDK 的 `tmwtypes.h`，顶层给这些库加入 `${Matlab_INCLUDE_DIRS}`。代表性 `libtask5c_mathematics.so` 的动态依赖没有 MATLAB runtime，但这不能证明无 MATLAB SDK 构建。

因此纯核心门禁必须包含：保留同一份已验收生成数学，提供经许可可分发的 standalone 类型/支持文件或只调整 Coder 的 standalone 输出配置；核对数据类型宽度、布尔表示和 ABI，并做原数学/导数及全序列回归。不能直接手写第二份数学替代它，也不能因方便而让核心一直要求 MATLAB 安装。

Windows 需公开符号导出（例如 `GenerateExportHeader`）、一致的编译器和 CRT、目标级平台选项及依赖部署。公共 C++ API 如跨 DLL 使用 STL，必须约定同一工具链/CRT；若以后需要跨工具链，再加 opaque-handle C ABI，不提前搭建额外封装。

目标文件名不等于单文件部署：`.so` / `.dll` 仍可能依赖数值库和 AD 内核，安装清单必须完整。

## 6. Windows 不是自动获得的能力

当前已确认：

1. 应用诊断包含 `execinfo.h` / `backtrace`；`PlannerContext` 读取 `/proc/self/maps`。可放到平台诊断实现，Windows 用等价可选诊断，不能改变控制数学。
2. 当前快照和所查官方版本的 `CppAdInterface.cpp` 都实例化 `GccCompiler` 和 `LinuxDynamicLib`。CppADCodeGen 的默认 GCC 路径也为 `/usr/bin/gcc`。
3. Linux RPATH、共享库后缀、启动时 `LD_PRELOAD` 和已有 AD `.so` 缓存不能搬到 Windows 使用。
4. Pinocchio、HPIPM、BLASFEO、Boost 和 C++20 线程功能需要在选定 Windows 工具链实测；当前没有 Windows 构建证据。

应先调查所选官方版本是否提供 compiler/loader 注入点，再考虑外部平台 backend 或由相同 AD 定义预生成内核，并分别构建、加载 Linux/Windows 原生产物。离线生成不改变算法，但需验证相同函数、导数及运行合同，并不能仅靠生成文件就证明完整 AD 集成可编译。

**如果没有可用公开替换接口，外部 adapter 不足以解决这个问题。** 在“官方源码保持原样”的边界下必须明确列为阻断，给出需要的平台补丁/上游支持范围后再决定；不偷偷修改 third-party，不移植成另一套求解器，不宣布 Windows PASS。WSL 也不能作为原生 `.dll` / `.mexw64` 的验收结果。

## 7. 第二阶段的迁移顺序和门禁

| 步骤 | 改动 | 验收 |
|---|---|---|
| A. 固定现状 | 保存当前配置、依赖版本、生成内核、模型 SHA、观测/参考/输出 | 明确实际 runtime 和未跟踪用户文件；不覆盖历史证据 |
| B. 提取核心 | 暂保留当前依赖，用独立 target 包含 composition root、controller、数学适配；MEX 只调 API | standalone 核心与 MEX 接口全序列输出严格相等，原 tick/事件/线程合同不变 |
| C. 去 SDK 耦合 | 将生成数学支持文件独立化，MATLAB discovery 变成 MEX 可选项 | 无 MATLAB/ROS 环境构建核心并运行 C++ replay；不能只是 MEX 编译通过 |
| D. 换官方 OCS2 | 固定兼容官方 commit 与 AD/QP 版本，改外部构建 provider 和应用 API 适配 | 原 solver 调用序列、配置、策略与动作回归；不能同时升级所有数值依赖 |
| E. Windows 平台 | 处理真实 compiler/loader、符号导出、路径、类型 ABI 和运行依赖 | 原生 Windows 编译核心和 MEX，验证初始化、AD 生成/加载、query、ID、关闭和完整 replay |
| F. 集成 | 原 Simulink 模型接回同名 MEX，配置保持原样 | 必要的真实闭环回归与人工视觉检查；本轮分析不启动它们 |

临时保留 fork 只是隔离 A/B 的中间步骤，不是最终交付。最终依赖必须来自官方，fork 保存在基线归档中。

验收应分开记录：

- 构建：核心不需要 MATLAB、ROS 包管理或 ROS runtime；MEX 构建才需要 MATLAB SDK。
- 接口：六个命令及原布局完全保持，两个 walking 模型和 reference 模型不修改。
- 行为：Linux 核心提取优先严格零差异；官方替换和跨平台若非零，先解释并按预先冻结的数值合同验收。不得看到误差后追改容差。
- 离散合同：tick、session、关节索引、模式、事件身份、MPC 更新次数和策略发布/拒绝状态严格一致。
- 数值：完整 policy state/input、wrench、q_des、qd_des、tau_ff，以及必要的原 OCP 目标和残差。跨编译器不承诺 bit equality，数值等价仍需量化证明。
- 求解：保留迭代预算、warm start、RK4/预测网格、投影、正则化、line search、反馈设置、query lead 和原基座求解。计时 API 的改变不得影响求解。
- 平台：Linux 和 Windows 分开标记 PASS；Linux 不能代表 Windows。

第二阶段最终应得到 `libg1_controller.so` + `g1_ocp_mex.mexa64`，以及原生 Windows 的 `g1_controller.dll` + `g1_ocp_mex.mexw64`。目前没有开始这些实现或验收。

## 8. 本轮结论

业务代码已经从 third_party 拆出，但库边界尚未建立。最优先的架构动作是把误放在 `cpp/mex` 中的核心组装和运行状态提取出来，建立真正的 `g1_controller_core`；其次独立化 MATLAB 生成数学的类型支持。

官方 OCS2 候选在已查关键数值文件上接近当前版本，有利于保持算法。但额外 benchmark API、完整依赖兼容性与 Windows AD 后端仍需处理。不能把“官方来源”“不依赖 ROS”“支持 Windows”当成同一个已解决的问题。
