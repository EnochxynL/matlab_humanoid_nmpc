# MATLAB 管理诊断文件

后续双 MEX 拆分中，完整 `planner_samples` 的汇总也已迁入 MATLAB：
OCS2 查询时采集原生策略诊断，独立逆动力学返回力矩后由
`g1.plannerMex` 调用 `DiagnosticWriter.sample()`，不再将力矩传回 OCS2。

本轮移除 `controller/data/`。C++ 仅保存诊断对象并计算、采集数值；
MATLAB 管理文件、目录、CSV 编码和验收所需的存储格式。

| 位置 | 职责 |
| --- | --- |
| `controller/diagnostics/Diagnostics` | 引用现有模型/MPC，维护可选诊断对象 |
| `StructuralDiagnostics`、`FootDiagnostics` | 在原生模型上评估代价、约束、动力学和足端量 |
| `TimingDiagnostics` | 收集求解、策略和参考事件 |
| `DiagnosticBuffer` | 有界的结构化数据批次，不涉及文件 |
| `g1.plannerMex` | 正常运行入口，转发命令并在动作完成后取走批次 |
| `g1.DiagnosticWriter` | 写 CSV、JSON、动态库清单和最新结构快照 |

同一个 `g1_ocp_mex` 仍保管所有 C++ 对象；不把指针暴露给 MATLAB。
新增 `batch = g1_ocp_mex('diagnostics')`，对应原生 API
`G1Controller::drainDiagnostics()`。批次含表名、列名、数值/文本单元格、
最新 218 维快照和初始化元数据。取走后清空行与元数据，重复读取不会重复计算。
诊断关闭时返回空批次，也不创建额外诊断模型。

直接调用原生接口的消费者须定期取走数据；累计超过 250000 行时明确报错，
不静默丢弃或无限增长。常规 MATLAB 入口每个完成的控制 tick 自动取走数据。
诊断回调中不反向调用 MATLAB；MEX 只在普通调用边界传输批次。

```matlab
config = g1.plannerConfiguration(root, logDirectory);
g1.plannerMex('init', config);
% 模型中各阶段通过同一个包装函数调用。
g1.plannerMex('flush'); % 运行期间读取 CSV 前刷新文件
g1.plannerMex('close'); % 取走尾批次，释放原生对象并关闭文件
```

`run_walking`、模型五个 MEX 调用块和现有验证入口已采用包装函数。
模型仅修改调用函数名，调度、端口和连接保持原样。
CSV 文件名、列名和诊断计算沿用迁移前定义；文本字段按 CSV 规则转义，
数值以 17 位有效数字写出。文件写入失败会显式报错；关闭时即使收集失败，
仍清理原生会话。OCS2 自身日志和 AD 缓存不受应用诊断开关控制。

原 Windows 诊断文件替换辅助函数及其测试随之删除。
独立 C++ 回放程序自行保存测试结果，并定期取走诊断批次；原生库不再生成日志。

验证证据目录：`runtime/evidence/matlab_diagnostics/`。

2026-10-20，Windows GCC 16.2.0 编译通过；完整模型回放完成 2401 tick、
289 次求解，与迁移前控制输出完全一致。18 张诊断表的列名和行数相同，
其中 17 张表的非墙钟数据完全一致；求解性能统计最大尾差约 3.55e-15，
218 维快照最大尾差约 2.71e-20，NaN 位置保持一致，配置元数据相同。
专项测试验证原生代码不创建诊断目录、取批次幂等、关闭诊断返回空批次、
MATLAB 写入失败时释放会话，以及关闭诊断后的完整回放与运行校验。
本轮未重新运行物理行走仿真。
独立 C++ 回放完成 2401 tick，历史 fixture 的最大差异约 0.00167215，
与既有结果一致。模型的 13 个系统 XML 未变，只有五处脚本的入口函数名改变。

本机开启全量诊断的完整回放耗时约 66.8 s，迁移前约 35.5 s；
结构化批次跨 MEX 传输和 MATLAB 文件输出增加了开销。
这是诊断路径的成本，不改变控制时间或求解次数；需要轻量运行时可关闭诊断。
