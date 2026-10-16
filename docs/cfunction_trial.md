# 块内 C 数值内核试验

模型：`models/experiments/g1_cfunction_trial.slx`。

`Quaternion → QuaternionToEuler（C Function）→ Euler`。双击中间块可直接查看原四元数到 ZYX Euler 的公式。无外部算法源文件、无 CMake，由 Simulink 仿真目标编译。块内保留原来源的许可证。正式模型、MEX 和控制核心未修改。

## 实测

MATLAB R2025b，已配置 GCC/G++ 13。使用冻结的 2441 组观测（真实记录及补充姿态）：

| 对照 | 最大绝对误差 | 不同 double 位模式数 |
|---|---:|---:|
| 原 MEX 对冻结基线 | 0 | 0 |
| C Function 对原 MEX | 4.4408920985006262e-16 | 1230 / 7323 |

C Function 已成功编译及执行。默认编译选项及一次有界 `-O1 -ffp-contract=off` 检查得到相同差异；未证实具体编译器或数学库根因，不继续做末位精度诊断。模型保存默认编译选项，不引入 GNU 专用标志。

**严格逐位门禁 FAIL，不能替换正式观测转换。** 本试验仅证明块内公式及 Simulink 自动编译可行，未证明其余内核、完整策略或 Windows 编译可行。未运行物理仿真。

复现：运行 `tests/validate_cfunction_trial.m`；当前预期在保存比较结果后触发严格门禁断言。需要已有的 `build/observation_hybrid/conversion_values.mat` 和已编译原观测 MEX。

证据：`build/cfunction_trial/acceptance.json`、`comparison.mat`。实验模型在其父模型目录新增独立 experiments 子目录，不参与正式初始化或构建。
