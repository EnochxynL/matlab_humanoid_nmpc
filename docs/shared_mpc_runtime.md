# 两种 MPC 的共用原生运行接口

`G1Controller` 直接管理两种 OCP 的实例生命周期、参考接入、观测阶段、
同步 SQP 求解、MRT 查询、重复查询缓存及失败后重建要求。
没有重新引入 `Backend` 转发层，也没有把问题内部对象暴露到 MEX。

## 问题选择与数据

`ControllerConfig::formulation` 默认是 `MpcFormulation::WholeBody`。
启用 `G1_BUILD_CENTROIDAL` 后也可以选择 `Centroidal`；未编译该能力时
初始化明确报错。初始化处只负责选择具体 OCP，后续运行方法共用。

私有状态通过 OCS2 已有的 `RobotInterface` 拥有具体问题，并持有其模型、
MPC/SQP 设置和参考管理器。模板仅用于初始化和诊断的具体类型适配；
公共 API 不暴露 OCS2、Eigen 或模板。

| 数据 | 全身 MPC | Centroidal MPC |
| --- | --- | --- |
| `dimensions().state` | 58 | 35 |
| `dimensions().input` | 35 | 35 |
| `dimensions().joints` | 23 | 23 |
| 状态 | 广义位置与速度 | 归一化质心动量、基座位姿、关节角 |
| 输入末尾 | 关节加速度 | 关节速度 |
| MPC 周期 | 读取全身配置 | 读取 centroidal 配置，当前为 1/80 s |

`Reference::targetStates/targetInputs` 的节点以及 `Policy::state/input`
使用 `std::vector<double>`，长度必须匹配当前 OCP。参考节点和事件容量
仍保留现有传输协议的上界。观测接口接收动态长度的状态和 `[tick,time]`；
tick 从 0 连续推进，外部决定何时请求求解。

首次求解的占位目标按问题处理：保留原全身复位布局，centroidal 不再
按全身布局清零。正式目标和接触时间表仍由外部参考包安装。
参考包维度和摆腿参数分别对照当前模型维度与加载的配置检查。

## 诊断与执行边界

OCP 代价、约束、动力学、求解统计、策略与脚部运动学诊断共用。
全身专用的逆动力学诊断和 218 维快照保留在明确的类型分支内；
centroidal 不生成 `structural_id`，也不把其输入解释为加速度计算力矩。
求解诊断直接使用传入的 tick，不再按固定采样周期反推 tick。

重力补偿保留两个上游包原有的独立实现和质量缓存行为。此前尝试合并
辅助函数并移除静态缓存，已单独提交后恢复；共用运行接口不依赖该整理。

`InverseDynamics` 仍然是独立的全身模型接口，状态/输入保持 58/35。
它不依赖规划目标或共用控制器；原生回放适配器显式把全身策略复制到
逆动力学数组中。centroidal 的执行转换属于下一阶段。

## 兼容与验证

现有 MATLAB MEX 仍解码 428 元素全身参考、58 维观测，编码 95 元素
全身策略。有效输入上的传输协议和数值语义保持不变；本阶段不改模型。
原生 C++ API 的向量类型已改变，调用方需要重新编译。

`g1_shared_runtime_smoke` 只通过公共控制器接口验证两种实例并存、
centroidal 维度检查、参考包、真实求解、已有策略查询、重复调用、
失败后关闭重建，以及诊断开关下策略逐值一致。
`g1_centroidal_smoke` 保留直接 OCP 基线，使用原有 centroidal 重力补偿函数。

```powershell
cmake --build native/build/windows/gcc16-ucrt64/core --parallel 1
native/build/windows/gcc16-ucrt64/core/lib/g1_shared_runtime_smoke.exe $PWD.Path runtime/evidence/shared_runtime
python native/scripts/verify_layers.py
```

全身回归继续运行原生 2401 tick 回放和现有 MATLAB 阶段、诊断及模型回放。
这一阶段验证原生运行接入；centroidal 尚未接入 Simulink 或完成行走验收。

本次验证证据位于 `runtime/evidence/shared_runtime/`：

- 完整构建、依赖边界检查通过；关闭 centroidal 的独立 CMake 配置检查通过。
- `final/acceptance.json`：两种实例并存，centroidal 维度/阶段检查、失败重建
  通过；开关诊断时策略逐值一致。
- `direct_centroidal/policy.csv`：直接 OCP 的六次查询与第一阶段基线逐值一致；
  其中质量来源与多质量缓存检查属于恢复前的尝试版本，恢复后不再执行。
- 全身原生 2401 tick 回放统计与整理前一致；MATLAB 阶段和诊断验证通过，
  2401 tick、289 次更新的模型回放策略/力矩与整理前 Windows 基线逐值一致。

共用运行测试使用自行给出的双支撑事件网格，直接 OCP 基线使用默认
自动延展网格；二者不作为同输入的逐值相等门禁。初始策略样本的最大
观察差约为 `6.12e-10`，保留在导出的 CSV 中。
