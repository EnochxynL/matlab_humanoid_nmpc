# Windows Codex 交接：G1 Simulink + 原生 C++ OCS2

> Windows 执行更新（2026-10-17）：见 [本次检查点](windows_checkpoint_20261017.md)。MinGW 8.1 + Conan 核心、四个 MEX、AD 生成/重载、2401 tick 模型回放及真实 4.8 s 行走仿真的自动检查均已通过。行走前进约 0.379 m，最大俯仰约 0.60°，动作映射误差为 0；人工动画验收已由用户确认 PASS。与 Linux 回放仍有最大约 3.50e-4 的数值差异，按用户授权保留报告并完成行走，不声称逐位一致；未改控制逻辑。下文“尚未实施”为原始交接时状态。

## 先读这里

本分支 `port/windows` 从 `master` 的
`f64f2053a06f6b822baa4e9351a1c391bdcf83e0` 创建。
Linux 工程路径为 `/media/enoch/DISK/MY_WORK/contribute/matlab_humanoid_nmpc_1`。
**不要把另一个历史目录 `matlab_humanoid_nmpc_matlab_native` 当作本工程。**

本次仅创建分支、保存交接文档；尚未实施或验证 Windows 适配。
用户允许考虑 MSVC 或 MSYS2/MinGW，目前未确定工具链。
请在 Windows 上先读取本机环境和用户的新指令，不把本交接当作修改数学或进行长仿真的授权。

## 工程边界与用户偏好

真实路径：Simulink → MATLAB Function → MEX → C++ controller → OCS2/Pinocchio/CppAD/HPIPM。
不经过 ROS2 通信，构建也不使用 colcon/catkin。

- `cpp/controller`：G1Controller 公共入口；PlannerContext 装配/持有对象；PlannerExecutor 执行调度。
- `cpp/controller/reference`：ExternalReferenceManager 包装原参考管理器，向 OCS2 安装外部参考。
- `cpp/problem`：人形机器人 OCP 数学、动力学、约束等原 C++ 实现。
- `cpp/extensions`：本工程的通用扩展，命名空间 `humanoid::ocp_extensions`；不是官方 OCS2。
- `adapters/matlab`：一个 MEX 对应一个 cpp；数据转换/检查及生命周期桥接。
- `models`：已验收的 Simulink/Simscape、Reference/Gait、Planner 和 JointDrive。

用户希望框图明确展示数据流，并尽量降低 `cpp/` 核心的复杂度；不要求所有 C++ 都改成 MATLAB。
保留 `cpp/` 名称，不另造 `g1_controller_core/` 源码根目录。
相伴的 h/cpp 是用户偏好的阅读单位。不要大规模重新组织已整理的文件。
`adapters/matlab/CMakeLists.txt` 暂时保留；是否改为 mex 命令构建，进入 Windows 后再讨论。

**不修改 OCP 数学、OCS2 solver 行为、权重、物理参数、Reference/Gait、JointDrive、时间语义或 MEX 协议。**
不要重启已经终止的 Task 5C 逐模块数学替换、MATLAB-native NMPC 或浮点末位研究。
不以 Windows 迁移为由更新数值库版本或重写控制器。

## 依赖目前怎么管理

| 依赖 | 当前来源 |
|---|---|
| OCS2 | 官方 leggedrobotics/ocs2，Git submodule |
| BLASFEO、HPIPM | 官方 Git submodule |
| CppAD/CppADCodeGen | 当前 OCS2 的 ocs2_thirdparty/include 树 |
| Eigen、Boost、Abseil | Linux 系统预安装包，CMake find_package |
| Pinocchio、Coal、URDFDOM | Linux /opt/ros/jazzy 内的预安装数值包，CMake find_package |
| OpenMP/Threads | 编译器/系统提供 |
| MATLAB SDK | 本机 MATLAB；Linux 已测 R2025b |

准确子模块提交见 `third_party/dependencies.lock.json` 和父仓库 gitlinks。
截至分支起点：

- OCS2 `26386754b8bf31ab78b503971d0cbf4fdbcd7cb4`
- BLASFEO `ae6e2d1dea015862a09990b95905038a756ffc7d`
- HPIPM `255ffdf38d3a5e2c3285b29568ce65ae286e5faf`

克隆后执行 `git submodule update --init --recursive`。
这些源码不复制进父 Git，不直接修改上游 checkout。
`dependencies.lock.json` 目前没有覆盖系统预安装库；因此还不是完整可复现的依赖锁。
当前未使用 FetchContent、vcpkg 或 Conan。
`/opt/ros/jazzy` 只是数值库位置，Windows 不需要为此安装 ROS2。

根 CMake 查找安装包，并通过 `cmake/NumericalLibraries.cmake` 直接组装所需 OCS2 数值 targets，
不调用上游 ROS 构建。`cmake/PinocchioCompatibility.cmake` 在 build/ 中生成适配副本，
把 Pinocchio 3 的诊断字段 parent 改为 parentJoint；另通过编译选项补充 fstream。
这些适配应审计保留，不要误认为上游源码被魔改，也不要直接删掉。

## 已确认的 Windows 风险：先解决这些，再宣称能编译运行

1. **运行时 AD 编译/加载是主要边界。**
   `third_party/ocs2/ocs2_core/src/automatic_differentation/CppAdInterface.cpp`
   明确使用 `CppAD::cg::GccCompiler`、`LinuxDynamicLib`，以及 `-shared`/`-rdynamic`。
   不是装齐包或把 .so 改名 .dll 就能解决。先调查当前 CppADCodeGen 的 Windows 能力及生成库 ABI。
   保持上游 checkout 原样，必要的平台适配方案先说明范围；不改 AD 公式。
2. **一套一致的工具链。** MEX、核心和跨边界传递 C++ 对象的依赖必须 ABI/CRT 一致。
   不混用 MSVC C++ 库与 MinGW C++ 库。MATLAB 支持的 MinGW 不等于任意 MSYS2 GCC。
   查目标 MATLAB release 的官方编译器支持表后再选择。
3. **DLL 导出及安装布局。** 核心当前缺少明确 export/import 宏；审计各共享库导出、
   Windows import library、RUNTIME/ARCHIVE 安装规则、多配置输出目录和 DLL 搜索路径。
4. **Linux 专用启动/取证。** scripts/start_matlab.sh 使用 LD_PRELOAD、g++、Linux OctoMap/libgomp 路径；
   PlannerContext.cpp 的 /proc/self/maps 仅用于依赖取证，需要平台分支。
   不照搬 LD_PRELOAD 到 Windows，不对全系统 PATH 做破坏性修改。
5. **编译选项。** 根脚本固定 Release `-O1 -DNDEBUG`；GNU 链接/RPATH 部分已有 UNIX 条件保护，
   但仍应全面审计 flags、测试中的 setenv、运行时路径和生成库目录。
6. **动态库冲突/缓存。** 不复用 Linux build/、.so、.mexa64、AD 二进制缓存。
   Windows 独立构建目录和缓存目录；核对 MATLAB 实际加载的 DLL，不只检查磁盘上的文件。
7. `.gitignore` 目前忽略 *.mexa64，但尚未补 Windows *.mexw64 等产物规则。

## Windows 建议的依赖方案（尚未实施）

继续固定 OCS2/HPIPM/BLASFEO 子模块。对其余安装库选择一个统一包管理环境并锁定版本。
若选 MSVC，可先评估 vcpkg manifest + registry baseline；Pinocchio 的目标版本、功能和传递依赖
必须实测确认，不能假定全部存在。若选 MSYS2，也要先确认 MATLAB、C++20 jthread 和运行时 AD 支持。
不要同时混装数个生态的同名 DLL。

CMake 继续描述构建；包管理器获取依赖；CMakePresets 描述平台和工具链。
保留 G1_BUILD_CORE=ON、G1_BUILD_MATLAB=ON/OFF，使无 MATLAB 环境可构建核心。
目标为 g1_controller.dll、g1_ocp_mex.mexw64，以及三个独立数值 MEX。
必要依赖 DLL 可以并存，当前并不要求压成一个二进制文件。

## 现有 MEX 和数值验收

- g1_ocp_mex：init / close / install_reference / install_observation / update_mpc /
  query_policy / inverse_dynamics。精确输入输出以 PlannerMex.cpp 为准。
- g1_observation_mex：三个敏感坐标转换，独立于控制器。
- g1_observation_extract_mex：观测检查与 [tick,canonical_time,raw_time] 提取。
- g1_reference_extract_mex：reference428 检查并原样传递。

时间身份为整数物理 tick；物理步对应 .002 s，canonical=double(tick)*.002；
60 Hz MPC 的 deadline 调度与更高频 policy query 分开。不得以 wall clock 替代控制时间。
不要把这个控制采样周期误称为所有 Simscape 内部积分步长。

严格逐位门禁仍有效。Windows 不同工具链不保证与 Linux 逐位一致；如果出现差异，
保存首个差异、量级和工具链证据，停在边界报告，不自行放宽门禁或追逐运算顺序。
用户明确允许数值零差异时不跑昂贵行走测试。

最近的 C Function 隔离试验在 f64f205：
models/experiments/g1_cfunction_trial.slx 可由 Simulink 编译块内公式，不用 CMake，
但 2441 组测试 maxabs=4.4408920985006262e-16，1230 个 double 位模式不同，严格 FAIL。
正式模型未替换。用户要求出现差异就停止，以后再分析；**不要继续该试验**。

## 建议执行顺序

1. 检查 Windows MATLAB release、mex -setup、编译器/架构、Git submodules 和磁盘资源。
2. 冻结依赖来源/版本和工具链；建立独立 build/windows 配置，先编译三个小 MEX。
3. MATLAB OFF 编译核心；平台诊断与 DLL 导出问题单独处理，不碰数学。
4. 验证运行时 AD 内核首次生成及再次加载；两者都成功才算核心初始化可用。
5. 编译 Planner MEX，初始化/关闭与协议冒烟测试；检查加载库真实来源。
6. 独立数学对照、固定输入完整策略回放；有差异即保存失败并报告。
7. 仅在数值门禁通过且用户确认后安排真实物理/视觉验收，不自动开 4.8 s 仿真。

## Linux 证据与携带限制

最新通用扩展重命名提交 bfa064e 已通过 fresh-process 2401 ticks / 289 MPC updates，
四份 CSV 与已验收恢复原 C++ 实现的基线逐字节一致。
原始 4.8 s 物理行走已人工 PASS：除原有偏行外正常。

**build/ 和 runtime/ 被 ignore，不随 clone 带到 Windows。** 请先确认下列文件是否已另行复制，
没有就报告缺少 fixture，不虚构已有数据，也不要为补数据擅自启动长仿真：

- build/core_extraction/fixture.bin：2401 行，每行 620 doubles。
- build/core_extraction/fixture.bin.states.bin：2401 行，每行 58 doubles。
- build/core_extraction/fixture.mat。
- build/observation_hybrid/conversion_values.mat：2441 组转换输入和冻结值。
- build/directory_validation/core_replay/ 内 planner_samples.csv、policy_knots.csv、
  gait_queries.csv、adapter_consumption.csv：严格回放基准。
- build/ocp_extension_naming/acceptance.json：最近 fresh-process 验收。
- 若需要视觉参考：runtime/walking_20261014_195254；它不是本次 Windows 已完成实验。

注意：fixture.bin 尾部118维来自更早的 generated-math 历史版本；驱动 measure 模式可输出
max_abs≈0.000101241 / different_bits=195928，这不是当前严格门禁基准。
**实际门禁是上述四份恢复原 C++ 基线 CSV。** 不要把历史 fixture mismatch 当作新平台故障或 PASS。

tests/core_replay.cpp 为 standalone 公共 API 回放入口；
tests/validate_single_file_mex.m、validate_planner_dataflow.m 等为 MATLAB 数值验证入口。
Windows/Linux 浮点或文本换行差异需区分记录；不能未经授权改写验收标准。

交付时清楚区分：配置成功、编译成功、初始化/AD 成功、固定输入数值通过、真实物理通过。
保留失败和 checkpoint；不删除 Linux 已验收成果，不提交大型生成物。
