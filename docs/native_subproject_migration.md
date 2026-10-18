# 原生子项目整理与 gait 残留审计

2026-10-18，用户确认将所有原生代码及构建归入 `native/`，并要求先查完整旧 gait 调用链，再成组删除。

## 已核查的调用关系

当前链路为 Simulink Reference/Gait → 428 元素参考包 → Planner MEX → `ExternalReferenceManager` → 原 `SwitchedModelReferenceManager`。`PlannerContext` 只安装此消费适配器，不构造旧命令生成器。

旧链为 `ProceduralMpcMotionManager` → `getGaitMap(gaitFile)` → `loadModeSequenceTemplate`，并通过 `GaitScheduleUpdater` 执行升/降步态；速度命令、滤波和目标轨迹生成另由 `WalkingVelocityCommand`、`BreakFrequencyAlphaFilter`、`TargetTrajectoriesCalculatorBase`、`WBMpcTargetTrajectoriesCalculator` 提供。仓库调用搜索确认这些类没有当前运行实例，只有相互引用和无效 include/CMake 条目。模型 XML 中也没有 gait.info 文件引用。

因此一起删除：

- 根 `config/gait.info` 与原 `cpp/problem/humanoid_common_mpc/config/command/gait.info`。
- `ProceduralMpcMotionManager`、`GaitScheduleUpdater` 的头文件和实现。
- `WalkingVelocityCommand`、`BreakFrequencyAlphaFilter` 头文件。
- 两个旧目标轨迹计算器的头文件和实现。
- 仅用于旧命令配置的 `getGaitMap` 函数及声明、`PlannerConfig::gait` 和两个无效 include。
- 上述实现的 CMake 编译项。

保留 `Gait`、`GaitSchedule`、`ModeSequenceTemplate`、`loadModeSequenceTemplate`、`loadModeSchedule`、`SwitchedModelReferenceManager` 和摆足规划。`WBMpcInterface` 仍通过 `GaitSchedule::loadGaitSchedule(config/reference.info,...)` 初始化，成本、约束、预计算仍使用这些对象。不能因在线步态由 Simulink 决定就删除 OCP 的模式/摆足消费逻辑。

## 目录与构建边界

`native/` 包含 `CMakeLists.txt`、`cpp/`、`adapters/`、`cmake/`、`third_party/`、`conan/`、原生 `scripts/` 与 C++ `tests/`。根目录保留 MATLAB/Simulink 应用、运行配置、资产、MATLAB 测试和仿真编排。

三个 Git 子模块通过 `git mv` 迁移，并更新 `.gitmodules`；名称、远端与固定提交不变。依赖锁内 `source_directory` 相对 `native/`，由 `native/scripts/verify_dependencies.py` 检查。HPIPM 既有 `experimental/andrea/notes/syscop.bib` 修改逐字节保留，不属于本轮更改。

新构建使用 Windows `build/windows/native-core` / `native-platform`、Linux `build/native-linux`，避免复用旧源码根目录缓存。Conan 缓存、编译器与 AD 缓存位置不变。MATLAB 初始化选择新的运行目录；运行配置仍从传入的应用根目录读取。

## 验证

- Windows 新目录完整构建通过（412 个构建步骤），36 个二进制运行依赖扫描通过。
- 平台 AD 生成/新进程重载 2/2 通过；真实 OCS2 AD 探针创建和重载均得到 value=15、Jacobian=8、Hessian=2。
- 原生公开 API 回放完成 2401 tick / 289 次更新，`planner_samples.csv` 与移动前 `build/windows/core-replay` 逐字节一致。
- 实际 MATLAB 模型回放完成 2401 tick / 289 次更新，阶段与动作检查通过，`planner_samples.csv` 与整理前内联模型回放逐字节一致。耗时 21.5158742 s，历史 Linux 基线最大差仍为 0.0010613718186487375，未新增数值差异。
- 已加载模块记录确认 MEX 与 `g1_controller.dll` 来自新 `build/windows/native-core/lib`，而非旧构建目录。
- 三个子模块固定提交、两个干净上游工作区及 HPIPM 原有修改文件哈希核对通过。严格的 `verify_dependencies.py` 仍会拒绝已有 HPIPM 工作区修改，本轮未削弱其检查。
- Conan 锁文件保持原内容，构建入口不再在已有锁文件时重写其导出时间戳。PowerShell 脚本语法与 Git 空白检查通过。
- Windows 已执行完整构建与运行验证；Linux 构建入口路径已更新，本机未实际执行 Linux 构建。

证据位于 `build/native_subproject_migration/`，包括 `submodules.json`、`layout.json`、`native_replay_comparison.json`、`model_replay_comparison.json` 及 `model_replay/acceptance.json`。本轮未新增物理行走；旧行走人工 PASS 为历史证据，不自动作为本轮新验证。
