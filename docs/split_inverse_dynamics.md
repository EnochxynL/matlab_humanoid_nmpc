# 求解与逆动力学分为两个 MEX

`g1_ocp_mex` 创建 OCP、MPC 和 MRT，负责外部参考、观测、求解与策略查询。
`g1_inverse_dynamics_mex` 独立创建机器人模型，根据 58 维状态和 35 维输入
计算 23 维关节力矩，不创建求解器，也不引用另一个 MEX 的对象。
两者使用相同的 task/URDF 配置和原有力矩公式。

正常应用继续使用 `g1.plannerMex`。包装层负责两个 MEX 的初始化、失败回滚和关闭，
现有 Simulink 框图、端口、调度和 MATLAB 调用形式保持不变：

1. OCS2 MEX 返回 95 元素策略。
2. MATLAB 将其中 state/input 传给独立逆动力学 MEX。
3. MATLAB 调用 OCS2 的 `complete_action(policy95, torque23)`。
4. OCS2 核对当前策略身份，记录策略诊断和完成样本，允许进入下一个 tick。

`complete_action` 不计算力矩。它只接受有限数据和当前 tick 的原样策略；
重复提交还必须使用相同力矩，不重复记录诊断。应用诊断仍由 OCS2 MEX 管理，
文件输出仍由 MATLAB 管理。不跨 MEX 传递原生指针。

独立使用逆动力学：

```matlab
g1_inverse_dynamics_mex('init', struct('taskFile', taskFile, 'urdfFile', urdfFile));
torque = g1_inverse_dynamics_mex('compute', state58, input35);
g1_inverse_dynamics_mex('close');
```

原生 C++ 用户分别创建 `G1Controller` 和 `InverseDynamics`，通过
`InverseDynamics::compute()` 计算力矩，再调用 `G1Controller::completeAction()`。
两个模型分别持有数据；共享已编译的核心库不等于共享求解器实例。

验证入口为 `inverseDynamicsMexTest`、`validate_synchronous_backend`、
`validate_matlab_diagnostics`、`validate_diagnostics_separation`、
`validate_planner_dataflow('split_inverse_dynamics', true)` 和原生 `g1_core_replay`。
