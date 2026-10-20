# Centroidal MPC 移植

第一阶段建立独立的 centroidal 模型与 OCP 构建目标，通过原生程序验证
模型构造、SQP 求解和 MRT 策略查询。当前全身 MPC 与两个 MATLAB MEX
继续使用原有接口；本阶段不接入 centroidal 行走或力矩执行链。

来源为 `wb_humanoid_mpc` 提交
`44c7bd8fbd75075cb2c78a61b7fcbb3edcbc86ef` 的
`humanoid_nmpc/humanoid_centroidal_mpc`，以及
`robot_models/unitree_g1/g1_centroidal_mpc/config`。
导入文件保留原有 BSD 许可声明。

## 代码边界

| 位置/目标 | 内容 |
| --- | --- |
| `model/humanoid_centroidal_mpc` / `humanoid_centroidal_model` | 质心状态、输入和 Pinocchio 映射 |
| `problem/humanoid_centroidal_mpc` / `humanoid_centroidal_mpc` | 动力学、代价、约束、初始化器及 OCP 装配 |
| `extensions/PinocchioEndEffectorKinematicsCppAd.*` / `humanoid_pinocchio_kinematics` | 通过通用映射计算脚位置、姿态、线速度、角速度及其导数 |
| `config/centroidal` | G1 centroidal 专用 task/reference，不覆盖全身配置 |
| `native/tests/centroidal_smoke.cpp` | 直接使用模型、SQP 和 MRT 的站立观测验证入口 |

centroidal OCP 可以依赖 `humanoid_common_mpc`、共享模型和 OCS2，
不能依赖 `humanoid_wb_mpc`、`g1_controller_core` 或 `g1_dynamics_core`。
CMake 的传递依赖检查和源码 include 检查继续约束这一边界。

6D 运动学实现取自 [manumerous/ocs2_ros2 的固定提交](https://github.com/manumerous/ocs2_ros2/tree/bf9ae46f3c6dc751348d8002c03aa2f4a7cc17e7)，
与现有应用扩展来源一致。它迁入 `humanoid::ocp_extensions`，不替换官方
OCS2 同名类型，也不修改 Conan 缓存中的依赖源码。

## 兼容适配

删除未使用的 `ProceduralMpcMotionManager` include，不导入 ROS、命令
生成器或原线程执行器。约束启用方法不再声称覆盖官方不存在的虚函数。
Gauss–Newton 代价遵循官方 `const` 接口，Pinocchio 的内部计算缓存标为
`mutable`；项注册/名称查询沿用现有 `TermRegistry` 适配。

官方 centroidal 工厂通过 `getBodyId` 查找接触，但当前模型添加的是
`FIXED_JOINT` 接触 frame。新接口在创建模型信息后使用 `getFrameId`
修正接触索引并检查有效性，保留原接触顺序和几何，避免无效索引进入 AD。

代价公式保留来源行为。例如 `ICPCost` 当前实际使用 COM 位置，含速度的
capture-point 表达式在来源中被注释，本次不借移植改变这个算法。

## 构建与验证入口

在已有 Conan/CMake 构建目录中启用：

```powershell
cmake -S native -B native/build/windows/gcc16-ucrt64/core -DG1_BUILD_CENTROIDAL=ON -DG1_BUILD_TESTS=ON
cmake --build native/build/windows/gcc16-ucrt64/core --target g1_centroidal_smoke --parallel 1
```

使用项目已有的 GCC/PATH、依赖 DLL 搜索路径和 `G1_AD_CACHE_ROOT`。
首次运行会在独立的 `cppad_centroidal_mpc_g1` 缓存中生成 AD 库。

```powershell
native/build/windows/gcc16-ucrt64/core/lib/g1_centroidal_smoke.exe $PWD.Path runtime/evidence/centroidal_ocp
python native/scripts/verify_layers.py
```

程序使用原始配置中的 23 个关节、35 维状态、35 维输入和 80 Hz MPC
更新频率，完成三次求解及六次策略查询，检查维度、模式和数值有限性。
观测固定在标称站立状态，因此这不是闭环稳定性或行走测试，也不表示
三次 SQP 更新已经达到最优解。证据写入 `acceptance.json` 和 `policy.csv`。

本阶段验证结果：完整构建通过；首次 AD 生成及独立进程缓存重载均通过
三次求解、六次查询；重载没有重新编译 AD 库，两次 `policy.csv` 逐值一致。
证据位于 `runtime/evidence/centroidal_ocp/`。现有全身控制器的 2401 tick
原生回放比较统计与迁移前一致。源码边界检查通过，规划与逆动力学之间
没有交叉 include；centroidal 与全身 OCP 的传递链接隔离也通过 CMake 检查。

## 第二种实现揭示的接口差异

可共用的是 SQP/MRT 生命周期、同步求解和查询流程；需随问题变化的是
状态/输入维度与含义、观测转换、目标生成、初始目标处理和部分诊断。
centroidal 与全身 MPC 的输入虽然同为 35 维，末尾分别表示关节速度和
关节加速度，不能把它们交给同一个未经转换的逆动力学入口。

第二阶段的共用原生接入见 [共用运行接口](shared_mpc_runtime.md)。
centroidal 的观测/参考和执行链仍待接入；原执行器中的逆动力学 TODO
仍需独立验证。
