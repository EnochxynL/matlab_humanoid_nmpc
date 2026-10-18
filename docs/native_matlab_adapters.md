# 原生 MATLAB 转换入口

2026-10-18：三个纯转换 MEX 替换为原生 MATLAB Function 块内代码，保留 Planner MEX 与 C++ 控制器。最初独立的五个 `.m` 实现已按用户要求内联并删除。

| 原入口 | 当前 MATLAB Function 块 |
|---|---|
| ObservationMex.cpp | ObservationConversion 内的 QuaternionToEulerZYX、LinearVelocityToWorld、AngularVelocityToEulerRates |
| ObservationExtractionMex.cpp | ObservationExtraction |
| ReferenceExtractionMex.cpp | ReferenceExtraction |

`models/g1_planner.slx` 的五个 MATLAB Function 块保存完整实现，没有转换 MEX 的 `coder.extrinsic` 或独立转换函数依赖。拓扑、端口和数据维度不变。参考仍为 428 元素、OCP 状态为 58 元素、时间身份为 3 元素。检查有限值、坐标/关节映射、时间、姿态、参考顺序与模式；不新增归一化、裁剪或控制器数学。

`tests/planner_block_functions.m` 从实际模型块读取代码，在 `build/block_function_tests` 下临时生成测试入口并自动清理。测试复用块内实现，不保留第二份算法。实际模型回放另外覆盖 Simulink 编译和连线。

原三个 C++ 文件及独立 CMake/Conan 构建入口已移除，构建只生成 `g1_ocp_mex`。原始版权声明保存在 `licenses/observation_conversion.txt`。旧二进制可能仍在被忽略的本地构建目录内，但当前模型与初始化不再引用它们。

## 独立函数版本验收历史

- `validate_matlab_adapters`：2441 组转换，最大绝对差 `8.881784197001252e-16`；2401 组观测身份检查、2401 组参考逐位检查和 18 组无效输入检查通过（含非法四元数导致的复数反三角结果拒绝）。
- `run_windows_planner_replay('native_matlab_migration')`：实际模型回放通过 2401 tick、289 次更新及阶段检查，模型动作与同次日志完全一致。与历史 Linux 基线最大差 `0.0010613718186487375`，按用户授权记录并继续物理验证；首次耗时 38.710475 s，最终复核 28.3829322 s。
- 原 4.8 s 行走完成，自动及用户人工验收均 PASS。证据目录 `runtime/walking_20261018_125655`，9601 个物理输出样本、2401 tick、289 次 MPC 更新，动作映射误差为 0，最大 raw/canonical 时间差 `8.881784197001252e-16 s`。
- 前进 `0.37908514840229135 m`，高度范围 `[0.73660879241046251, 0.8] m`，最大绝对俯仰 `0.59962726330955185°`。与 2026-10-17 MEX 转换版本相比，终点前进距离相差约 `1.50e-6 m`。摘要图显示高度稳定、左右足交替支撑，无倒地迹象。
- 缓存初始化耗时 `0.8479452 s`，物理仿真耗时 `614.1906798 s`。采用 MATLAB 单次 timer 在会话内执行，以避开连接工具的 600 s 调用超时；完成后 timer 已删除。
- 仅 Planner MEX 的 CMake 构建配置通过；增量构建无须重编核心。PowerShell 脚本语法检查通过。
- 五个原生 MATLAB 函数的 Code Analyzer 消息均为 0；最终函数检查和模型回放复核通过。

对照数据位于 `build/native_matlab_migration/adapters.json` 与 `model_replay/acceptance.json`。不以微小浮点差异作为停止条件；结构、时间、模式与有限值检查继续有效。

## 内联版本复核

块代码提取检查再次通过 2441 组转换、2401 组观测身份、2401 组参考逐位检查及 18 个无效输入；模型回放再次通过 2401 tick / 289 次更新，基线最大差仍为 `0.0010613718186487375`，耗时约 25.7432 s。

本轮证据保存在 `build/inline_matlab_migration`，回放的 `planner_samples.csv` 与独立函数版本逐字节一致。

内联版本行走完成，证据目录 `runtime/walking_20261018_131159`：4.8 s、9601 个物理输出样本、2401 tick、289 次更新，动作映射误差为 0，所有自动检查通过。前进 `0.37908514840229135 m`，高度范围 `[0.73660879241046251, 0.8] m`，最大绝对俯仰 `0.59962726330955185°`。

`planner_samples.csv`、`native_physics.csv`、`held_action.csv` 和 `controller_state_native.csv` 四份文件均与独立函数版行走逐字节一致，比较报告为 `build/inline_matlab_migration/walking_comparison.json`。物理摘要图已检查，高度、姿态和交替支撑无异常。缓存初始化耗时 0.9376867 s，仿真耗时 718.8702831 s（不据此作性能比较）。

测试临时入口全部清理，辅助脚本 Code Analyzer 检查无消息。上方用户人工 PASS 属于独立函数版本；本次内联版本的 `human_visual` 仍保留为 `PENDING`，不自动继承人工签署。
