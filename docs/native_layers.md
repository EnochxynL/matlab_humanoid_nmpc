# 原生代码的 model / problem / dynamics 边界

三层位于 `native/src/g1_controller/`，保留原来的包名、命名空间和
`include/src` 布局。按实际内容选择文件，不为凑齐包名建立空目录。

| 层 | 内容 | 项目内依赖 |
| --- | --- | --- |
| `model` | 配置、状态输入映射、接触几何、Pinocchio 建模、共用运动学和动力学数学 | 无 problem/dynamics 依赖 |
| `problem` | 代价、约束、参考与摆腿、初始化器、OCP 装配及 AD 动力学接入 | model |
| `dynamics` | 独立逆动力学入口及其模型实例生命周期 | model |

`problem` 和 `dynamics` 禁止互相包含或链接。两者可以使用共同的 OCS2
模块；共享类型与算法不要求共享模型实例。

`model/humanoid_common_mpc` 接收原 common/contact/pinocchio_model、
`MotionPhaseDefinition.h` 及对应源文件。
`model/humanoid_wb_mpc` 接收全身状态输入映射和 `DynamicsHelperFunctions`。
其中力矩算法既供独立逆动力学使用，也供规划代价和原生诊断使用，
因此属于共享模型层，不能放到 dynamics 后再让 problem 引用它。

`problem` 保留两套原包的规划问题代码。`dynamics/humanoid_wb_mpc`
保存独立的 `InverseDynamics` 类；当前没有 common 包专属的逆动力学入口，
故不创建空的 `dynamics/humanoid_common_mpc`。公共维度与力矩类型位于
`native/include/g1_controller/ModelTypes.h`，逆动力学头文件不再引用
`G1Controller.h`。

构建关系：

```text
g1_ocp_mex → g1_controller.dll → humanoid_wb_mpc → humanoid_common_mpc
                                            ↘        ↓
                                              humanoid_model
                                                    ↑
g1_inverse_dynamics_mex → g1_dynamics.dll ─────────────┘
```

`humanoid_model` 是静态库。两个项目 DLL 各自链接所需数学代码，各自创建
ModelSettings、Pinocchio 和状态输入映射对象，不共享求解器实例。
独立逆动力学项目目标不链接 humanoid_common_mpc、humanoid_wb_mpc 或
g1_controller_core。

运行 `python native/scripts/verify_layers.py` 检查直接和间接 include 边界。
配置时的 `cmake/VerifyLayers.cmake` 同时禁止项目目标之间出现传递交叉链接；
违反边界会使 CMake 配置直接失败。
同时检查生成的链接命令和 DLL 导入，确认两个 MEX 不通过同一个项目 DLL
间接恢复耦合。数值验证继续使用原生 2401 tick 回放及现有 MATLAB 验证入口。

本次验证证据位于 `runtime/evidence/native_layers/`：原生回放统计与拆分前一致；
4 项独立逆动力学测试、阶段和诊断验证通过；完整 Simulink 回放完成
2401 tick、289 次求解，策略与力矩输出与拆分前逐值完全一致。
源码依赖检查覆盖 model 23 个文件、problem 71 个文件和 dynamics 2 个文件，
交叉依赖为零；正常链接图可配置，三种故意注入的错误链接均被拒绝。
本轮未重跑物理行走仿真。
