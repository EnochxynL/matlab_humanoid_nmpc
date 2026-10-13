# 独立 G1 控制核心：Phase 1

本阶段建立软件边界；随后用户明确授权替换为官方 OCS2。核心提取在原 fork 上的全序列验收已通过，官方依赖替换已通过构建与完整输出回归。OCP、模型、配置保持不变，不做 Windows 适配或性能优化。

## 归属与接口

`cpp/include/g1_controller/G1Controller.h` 是公开 API：配置、观测、参考、策略和力矩均使用标准 C++ 类型，PImpl 隐藏 OCS2、Pinocchio 和运行时状态。

- `cpp/controller/`：原控制器以及从 MEX 提取的 ControllerRuntime；拥有状态、策略缓存、tick、线程和安全检查。
- `cpp/whole_body/`：原 OCP、数学适配和 PlannerContext 组装入口。
- `cpp/robot/`、`reference/`、`time/`、`interfaces/`、`diagnostics/`：原应用代码。
- `cpp/whole_body/humanoid_wb_mpc/src/dynamics/`：原 WB 逆动力学辅助实现；数学不变。
- `adapters/matlab/PlannerMex.cpp`：原六个命令、协议解包/编码、MEX 锁和退出回调、异常转换。没有 OCP 初始化、状态反馈、求解调度或机器人安全数学。

公开 API 为 initialize / setReference / query / inverseDynamics / step / shutdown。MEX 仍是 init / reference / query / inverse_dynamics / step / close，仍使用 74/428/95/23/118 维协议。

观测位置在世界系，四元数 wxyz 表示局部到世界旋转，线/角速度在基座局部系；29 关节输入及 23 关节力矩顺序保留。两个 measured contact 槽位继续保留，观测 STANCE 合同没有改变。

ControllerConfig 提供项目数据根目录和日志目录。既有 authority 环境开关保留；standalone replay 从同一 controller_environment.json 设置它们。`SIMULINK_REFERENCE` 等历史名称是参考来源标签，不是 SDK 或 Simulink 运行依赖；本阶段没有重设计这些配置。

## MATLAB-independent 构建

```bash
./scripts/build_core.sh
./scripts/build_mex.sh
```

- G1_BUILD_CORE=ON：目标 g1_controller_core，输出 libg1_controller.so。
- G1_BUILD_MATLAB=OFF：不查找 MATLAB，不构建 MEX。
- G1_BUILD_MATLAB=ON：额外生成 g1_ocp_mex.mexa64，只链接核心库。
- G1_BUILD_TESTS=ON：构建只调用公开 API 的 g1_core_replay。

原 generated/ 的 157 个文件全部保留。项目-owned `math/support/tmwtypes.h` 只定义现有内核实际使用的五种标量类型，不复制 SDK 实现，也不改生成公式。静态断言确认它们与此前 Linux SDK 的具体类型一致：boolean_T=uint8_t、int64_T/int64_t、uint64_T/uint64_t、real_T/double、real32_T/float。

核心动态依赖不包含 MATLAB/MEX 或 ROS middleware。Pinocchio/URDFDOM 等安装包目前仍从 /opt/ros/jazzy 前缀发现，普通 CMake 没有调用 ROS 包管理或编译 ROS 接口。这是数值包的当前安装来源；没有声称本轮已迁移其分发渠道。

.so 依赖配套数值 .so 和原 AD 缓存，不是单文件静态部署。Windows 尚未适配。

## 保留原版依赖

用户后续授权切换官方 OCS2。当前 `third_party/ocs2` 是官方
`leggedrobotics/ocs2` 固定提交 `26386754b8bf31ab78b503971d0cbf4fdbcd7cb4` 的完整、未修改 submodule。
BLASFEO 和 HPIPM 同样使用原版本的完整 submodule，主仓库不再跟踪供应商文件。
`git submodule update --init --recursive` 初始化依赖。

替换前的 core/MEX 验收：2401 个 tick、289 次 MPC 更新，所有 118 输出 double 与已验收行走记录逐位相等；standalone split/step 均通过；无 MATLAB SDK 的核心构建完成。
这些结果只适用于原 fork，不能用于宣称官方版验收成功。

官方替换的处理方式及验收见 `official_ocs2_transition.md`。
历史构建、回归结果和原 vendor 副本保存在忽略目录 `build/core_extraction/`，没有删除。

最终官方版本验收：standalone split/step 与 fresh-process MEX 的完整 2401-tick 输出均逐位相等。SDK 禁用核心构建通过，427 个编译单元无 SDK 路径。详见 `core_extraction_acceptance.json`。
