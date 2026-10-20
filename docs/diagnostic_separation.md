# 运行与诊断边界

运行所需的时间、连续性与阶段检查不依赖日志对象。
`controller/CanonicalControlTime.h` 从原 `data/time/CanonicalControlTime.h`
提取纯运行合同，不再持有文件流；未使用的旧 `ControlTime.h` 同时删除。

`Ocs2Backend` 创建并持有 OCP、求解器、参考管理器和可选诊断对象，
直接向诊断对象提供求解事件和策略数据。`G1Controller` 通过后端公开的
`diagnostics_` 记录控制时间及提取诊断批次；策略诊断在查询时由后端采集。
策略与力矩的完整结果样本由 MATLAB 协调层汇总，关闭诊断时跳过这些记录。

诊断会话独占时间/样本 CSV、参考消费记录、配置与动态库信息导出，
并注册已有 `TimeLog` 和 `StructuralObserver`。
`ExternalReferenceManager` 继续执行原有参考校验和安装，仅通过回调报告消费结果。
所有原有校验在关闭诊断后仍执行。

## 开关与兼容性

```matlab
config = g1.plannerConfiguration(root, logDirectory);        % 默认开启
config = g1.plannerConfiguration(root, logDirectory, false); % 关闭应用诊断
g1_ocp_mex('init', config);
```

C++ 对应 `ControllerConfig::diagnosticsEnabled`，默认 true；MEX 要求该字段为
逻辑标量，省略字段时保留默认值。
关闭时不创建应用日志目录、诊断文件、额外 OCP 克隆或诊断观察器。
OCS2 自身的日志与 CppAD 代码缓存不属于应用诊断开关。
`run_walking` 继续开启诊断，因为其验收流程读取诊断日志。

本轮不迁移诊断计算到 MATLAB，也不改变模型框图或 OCP 公式。

## 验证入口

```matlab
addpath tests tools/windows
run_windows_planner_replay('diagnostic_separation/enabled');
validate_diagnostics_separation;
```

测试在关闭诊断后检查无目录生成、首次求解要求、查询阶段顺序、原始时间校验、
连续 tick 和策略身份检查，再执行完整 2401 tick 模型回放，比较开启诊断时的输出。
证据存入 `runtime/evidence/diagnostic_separation/`。

2026-10-19，Windows GCC 16.2.0 构建通过。开启诊断的完整回放与修改前
2401 行控制输出逐值相同、最大差异为零，289 次求解时刻完全相同。
关闭诊断的完整模型回放同样与开启时输出完全一致，且未创建指定日志目录；
上述运行校验检查通过。本轮未重新运行物理行走仿真。
