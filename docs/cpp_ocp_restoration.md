# 恢复参考 C++ OCP 执行路径

## 实现范围

参考仓库 `wb_humanoid_mpc` 固定快照 `44c7bd8fbd75075cb2c78a61b7fcbb3edcbc86ef`；修改前工程 HEAD `14aea7472c98fa7973685594d5028e96fc584705`。

- `PlannerContext` 直接使用 `WBMpcInterface::getOptimalControlProblem()` 和 `getInitializer()` 创建 `SqpMpc`，移除整条生成数学安装链与调用权威证据对象。
- 当前控制器的状态准备恢复原 `WBMpcMrtJointController::updateMpcState` C++ 函数体；忽略空白、注释后与参考代码逐字一致。重力补偿直接调用 `weightCompensatingInput`。
- 二次代价、罚函数、关节限制、零 wrench、摩擦、足底力矩、碰撞、足端跟踪和预计算重新使用原业务对象的 C++ 执行路径。
- 删除 `cpp/math/` 包装、外层 `math/` MATLAB 数学源和 `generated/` MATLAB Coder 产物；移除六个数学库构建目标和链接依赖。关闭环境变量不再是恢复手段，生产代码中已没有 TASK5C 执行分支。
- CppAD / CppADCodeGen 仍保留：这是参考 C++ 代码本来的自动微分与动态内核生成技术，不是被移除的 MATLAB Coder 路线。

## 保留边界

保留参考包内所有生产 C++ 文件、官方 OCS2 固定版本、G1 兼容扩展、URDFDOM 适配、已批准的索引/线程安全修复。未整目录覆盖源码；未恢复 ROS 通信或 fork 依赖。

MEX 协议、拆分 query/ID、外部 Simulink Reference/Gait、canonical tick、5ms query lead、JointDrive、物理模型、权重及求解器参数均保持不变。`config/controller_environment.json` 为空容器，不再承载数学选择。

原源文件及完整差异见 `docs/source_audit/`。迁移前构建库和源码另存于本机 `build/cpp_restoration/before/`；历史 Git 提交仍可恢复。

## 构建与回放

串行增量构建核心、MEX 和 standalone replay 成功。OCS2、BLASFEO、HPIPM 三个 submodule 的 SHA 与源文件状态检查通过。核心/MEX 动态依赖没有六个旧生成数学库，没有缺失库。模型与任务、参考、步态配置 SHA 均未变。

历史真实观测回放覆盖2401物理tick、289次MPC更新。模式与查询时间逐次严格相等；MEX 拆分/组合结果严格一致，策略篡改、未完成tick和重复逆动力学检查通过。这里的回放不是新的物理闭环。

**历史严格零差异测试 FAIL**：首个非零输出出现在tick650（1.300s）。保留原失败日志，再以单独的 measure 模式完成全序列差异统计；measure 完成不表示原严格门禁通过。没有为消除舍入差异修改原公式或放宽生产控制/物理安全门禁。

| 信号 | 最大绝对差 | 发生时刻(s) |
|---|---:|---:|
| joint_position_target_rad | 3.0282983e-08 | 3.598 |
| joint_velocity_target_rad_s | 1.7304562e-06 | 3.598 |
| contact_wrench | 0.0001012406 | 3.584 |
| joint_acceleration | 8.5206416e-05 | 3.584 |
| feedforward_torque_Nm | 2.7200089e-05 | 3.432 |

最大差的不同字段有不同单位，不能把全向量最大差统一写成力矩误差。差异的详细数值保存在 `build/cpp_restoration/replay_measure/comparison.json`；旧严格失败在 `replay_split.log`。这次不研究优化器浮点执行轨迹，也不声称逐位无损。

## 真实物理测试

真实 Simscape 从正常初始化运行至4.8s，9601个物理输出采样、2401个控制tick、289次MPC更新，数值检查通过。动作组装误差为0，canonical time 最大偏差 8.88e-16s；策略查询提前量检查通过。

- 位置从 (0,0,0.8)m 到 (0.3790837004, 0.0762576873, 0.7692536536)m。
- 基座高度范围 [0.7366087813079591, 0.8]m；最大绝对 pitch 0.599626913°。
- 与旧真实行走轨迹逐时刻对照，最大 x/y/z 差分别为 6.3e-08/8.75e-09/1.56e-08m；最大 pitch 差 8e-07°。
- 策略接触模式、查询时间序列及实际左右足接触标志均与旧记录严格一致。左右足 Fz 最大差分别为 0.000361N、0.00143N。
- 全部导出物理字段有限。饱和计数为965个关节采样点，旧轨迹同为965；不声称执行器完全无饱和，也没有修改饱和逻辑。
- 核心初始化 1.583s；仿真执行 612.981s（包含 Simscape 初始化及仿真，不是单轮MPC耗时）。MCP等候上限600s造成客户端超时，但后台正常完成，结果文件与后续 stopped 状态已确认，没有重跑。

结果目录：`runtime/walking_20261014_142343/`。原始 MAT、CSV、独立检查、`physical_summary.png` 和历史对照图 `cpp_vs_generated_physics.png` 均已保存。Multibody 回放保留；**人工视觉结论 PENDING**，不继承历史 PASS。

资源抽样从物理 0.772s 起，抽样窗口最小可用内存 3.66GiB、MATLAB最大RSS 5.73GiB、swap最大 3.52GiB。未终止其他用户进程。

机器可读摘要：`docs/cpp_ocp_restoration_acceptance.json`。这是恢复参考 C++ 执行路径后的功能和物理证据，不将旧严格零差异 FAIL 改写为 PASS。
