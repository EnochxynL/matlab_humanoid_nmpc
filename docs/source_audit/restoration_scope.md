# 原源码与 MATLAB 替代路径核对

本轮只读调查，不修改生产代码、不编译、不切换运行权限。当前未提交的文件整理和中文注释亦纳入快照。源文件清单及 SHA 见 inventory.json，逐文件差异为本机审计中间产物，见 `build/cpp_restoration/source_audit_diffs/`（不提交 Git）；文本差异先统一 CRLF/LF，避免换行噪声。

## 结论与范围

对原仓库 humanoid_nmpc/humanoid_wb_mpc、humanoid_common_mpc 和 robot_runtime/robot_core、robot_model 逐文件按映射路径核对。两个 MPC 包的生产 .h/.cpp 没有整文件缺失。缺失路径不是历史删除证明，也不排除保留文件内部存在改动。

此前将“实现没有完整收纳在 whole_body”与“原实现缺失”混用，不准确。MATLAB 替代主要通过外部包装、抽取 OCP term 再注册和独立状态准备路径实现；不是原生产文件大批删除。

## 执行路径替代与原实现

| 当前替代 | 可恢复的原实现 | 接入位置 |
|---|---|---|
| GeneratedZeroWrench | common/constraint/ZeroWrenchConstraint.cpp | PlannerContext 中 install |
| GeneratedFriction | common/constraint/FrictionForceConeConstraint.cpp | installFriction |
| GeneratedPenalties | 原依赖 fork 的罚函数；当前 cpp/extensions/G1*Penalty | installPenalties |
| GeneratedJointLimits | common/constraint/JointLimitsSoftConstraint.cpp | installJointLimits |
| GeneratedTerminal/Running | common/cost/StateInputQuadraticCost.cpp、工厂与 OCS2 quadratic cost | installQuadraticCosts |
| generatedNominalState | common/reference_manager/SwitchedModelReferenceManager.cpp | 二次代价包装 |
| generatedNominalInput / GeneratedInitializer | common/pinocchio_model/DynamicsHelperFunctions.cpp、initialization/WeightCompInitializer.cpp | 代价、初始化器及控制器补偿路径 |
| prepareState | wb/src/mrt/WBMpcMrtJointController.cpp 的 updateMpcState | 当前 cpp/controller/WBMpcController.cpp 独立实现分支 |
| GeneratedPreComputation / GeneratedFootConstraint | common/HumanoidPreComputation.cpp、wb 预计算和足端加速度/摆腿约束 | wavec::install |
| GeneratedMoment / FusedMomentGraph | common/constraint/ContactMomentXYConstraintCppAd.cpp | installMomentBridge |
| GeneratedCollisionCost / CollisionGraph | common/constraint/FootCollisionConstraint.cpp 和 owning soft cost | installCollisionBridge |
| GeneratedFootCost / FootCostGraph | wb/cost/EndEffectorDynamicsFootCost.cpp 和辅助代码 | installFootCosts |

表中 common=humanoid_common_mpc/src，wb=humanoid_wb_mpc；这是恢复入口映射，不表示每个包装等同替换一整个原类。generated/ 是替代数学的产物，外层 math/ 是其 MATLAB 源码。原 fork 罚函数不在 wb_humanoid_mpc 源码树内，属于其依赖，不能从此仓库复制出不存在的文件。

## 不能以整目录覆盖方式还原

生产文件差异包含官方 OCS2 适配：G1 扩展类型、const AD 回调、参考管理 initMode 接口以及应用侧观测缓存；另有 URDFDOM include 路径、关节索引安全修复、求解线程退出修复和新增中文注释。这些不应随 MATLAB 数学撤回一并删除。

当前实际调度器是 cpp/controller/WBMpcController.cpp，不是原包内 mrt 文件。直接复制原 mrt 文件不会撤回当前状态准备与确定性调度路径。

建议恢复顺序：
1. 保留当前工作区和已验收回放证据，建立独立恢复检查点。
2. 让 PlannerContext 使用基础 OCP 和原初始化器，撤销生成数学安装链；控制器状态准备和补偿输入恢复为原 C++ 数学路径。
3. 保留外部 Simulink Reference/Gait、canonical time、拆分 query/ID、官方 OCS2 兼容扩展和已批准安全修复。
4. 确认运行路径不再调用 generated 数学后，再移除数学库链接、迁移权限和证据包装；不要仅关环境变量就宣布完成结构恢复。
5. 文件整理结束后统一构建，独立核心/MEX 同观测回放与必要物理回归验证；数学恢复未验证前不得称为无损迁移完成。

本轮不对修改文件逐行作数学等价认证；差异已完整保存，后续恢复应逐项保留兼容调整、撤销 MATLAB 替代，不能宣称所有 modified 文件只含接口变动。

## 实际缺失路径清单

- `humanoid_nmpc/humanoid_wb_mpc/CMakeLists.txt`
- `humanoid_nmpc/humanoid_wb_mpc/package.xml`
- `humanoid_nmpc/humanoid_common_mpc/CMakeLists.txt`
- `humanoid_nmpc/humanoid_common_mpc/package.xml`
- `humanoid_nmpc/humanoid_common_mpc/test/testEndEffectorKinematicsCostElement.cpp`
- `robot_runtime/robot_core/CMakeLists.txt`
- `robot_runtime/robot_core/package.xml`
- `robot_runtime/robot_model/CMakeLists.txt`
- `robot_runtime/robot_model/package.xml`
- `robot_runtime/robot_model/test/testRobotDescription.cpp`

## 数量

- humanoid_nmpc/humanoid_wb_mpc: {'missing': 2, 'line_endings_only': 21, 'modified': 14}
- humanoid_nmpc/humanoid_common_mpc: {'missing': 3, 'line_endings_only': 47, 'modified': 24}
- robot_runtime/robot_core: {'missing': 2, 'line_endings_only': 3}
- robot_runtime/robot_model: {'missing': 3, 'line_endings_only': 8, 'modified': 1}
