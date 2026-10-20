# MATLAB 调度与同步 OCS2 后端

原 `PlannerExecutor` 的确定性模式把求解提交到工作线程后立即等待完成。
Simulink 已有 `g1_planner/MPCSchedule`，C++ 又维护 epoch、nextSlot 和更新资格，
形成了两套相同的调度状态。本次由模型中的调度器独立决定求解时刻。

## 职责

| 位置 | 职责 |
| --- | --- |
| `matlab/+g1/plannerConfiguration.m` | 选择 task、reference、URDF 和日志路径 |
| `g1_planner/MPCSchedule` | 控制 tick 对应的 MPC 更新请求 |
| `PlannerMex.cpp` | MATLAB 数据校验、命令分发和实例生命周期 |
| `G1Controller.cpp` | 观测、参考和策略身份绑定，以及阶段缓存 |
| `Ocs2Backend` | 原生 OCP、SQP MPC、参考和诊断对象的创建与生命周期；同步求解、策略安装/查询及 Pinocchio 逆动力学 |

初始化改为 `g1_ocp_mex('init', g1.plannerConfiguration(root, folder))`。
独立 C++ 调用者通过 `ControllerConfig` 显式提供相同的四个路径。
删除旧 `PlannerConfig` 的占位字段和隐式目录推断。

`update_mpc(token, true)` 在当前调用线程完成求解与策略安装，
`update_mpc(token, false)` 使用已有策略并登记当前观测。
首次更新必须求解；同 tick 重复调用不得改变 request，且不重复求解。
求解异常直接传回 MATLAB，后端进入失败状态，必须 close/init 后恢复。
保留阶段顺序、连续 tick、策略身份和有限性检查。

删除应用层工作线程、互斥观测副本、条件变量、请求完成计数、
内部 60 Hz 时钟及五秒线程等待超时。OCS2 自身的数值并行设置不变。
完整策略仍留在 C++；返回 MATLAB 的是当前查询得到的单个策略样本。
诊断仍接在 C++ 求解器回调上，避免搬运完整策略。

模型框图和 OCP 公式不变。`timing_trace.csv` 的更新事件时间戳现在记录
外部请求的实际观测时刻，不再代表 C++ 重新计算的名义截止时间。

## 验证

验证证据保存在忽略目录 `runtime/evidence/synchronous_backend/`。

- `before/`：原线程后端的完整模型回放。
- `external_requests/`：刻意在 2 ms 额外求解，在 18 ms 不求解，确认外部调度权；
  同时检查重复调用、配置失败、求解失败、失败后拒绝查询和关闭后重新初始化。
- `after/`：同步后端完整模型回放及原有阶段检查。

运行入口：

```matlab
addpath tests tools/windows
validate_synchronous_backend;
run_windows_planner_replay('synchronous_backend/after');
```

回放要求 2401 个控制 tick、289 次求解，模式、查询时间和动作数据流身份保持。
浮点差异按项目既有约定记录；模型回放不能替代实际物理行走验收。

2026-10-19 的 Windows GCC 16.2.0 验证：编译与上述专项检查通过；
修改前后完整模型回放的 2401 行样本完全相同，最大数值差异为零，
289 次求解的观测时间也完全相同。
独立 C++ 回放通过，2401 tick、289 次求解；与历史 Linux fixture 的最大差异
约为 0.00167215，与既有工具链迁移验证一致。

真实物理闭环 `runtime/walking_20261019_214147/` 完成 4.8 s：9601 个物理样本，
2401 个控制 tick、289 次求解，动作映射误差为零，前进约 0.379085 m，
最大俯仰约 0.599627°。用户已确认本次动画人工 PASS。
与修改前的 `runtime/walking_20261019_205616/` 对比，2401 行控制输出完全相同，
最大数值差异为零。
