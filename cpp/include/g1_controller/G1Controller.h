/**
 * @file
 * @brief 不暴露 MATLAB、ROS、Eigen 或 OCS2 类型的控制核心公共接口。
 * 适配器提交具名数据，核心通过 PImpl 隐藏机器人模型、求解器和线程。
 * 这是 C++ 接口，不是保证跨编译器二进制兼容的 C ABI。
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace g1_controller {
inline constexpr std::size_t StateDimension = 58;
inline constexpr std::size_t InputDimension = 35;
inline constexpr std::size_t RobotJointCount = 29;
inline constexpr std::size_t ControlledJointCount = 23;

/**
 * @brief 初始化资源路径：工程根目录用于定位模型和配置，日志目录用于保存运行证据。
 */
struct ControllerConfig {
  std::string projectRoot;
  std::string logDirectory;
};

/**
 * @brief 真实机器人观测。位置在世界系，四元数 wxyz 表示局部到世界旋转。
 * 线/角速度在基座局部系；29 关节按运行时约定的机器人顺序传递。
 * rawBackendTime 保留物理后端时间；核心校验后派生 canonical tick/time。
 * measuredContacts 虽保留在接口中，当前已验收运行路径仍固定观测模式为双足支撑。
 */
struct Observation {
  double rawBackendTime = 0;
  std::array<double, 3> basePosition{};
  std::array<double, 4> baseOrientationWxyz{};
  std::array<double, 3> baseLinearVelocityLocal{};
  std::array<double, 3> baseAngularVelocityLocal{};
  std::array<double, RobotJointCount> jointPositions{};
  std::array<double, RobotJointCount> jointVelocities{};
  std::array<double, 2> measuredContacts{};
};

/**
 * @brief 外部参考与接触时间表的定长容器。
 * 数组为容量，targetCount/eventCount 才是有效数量；模式数为事件数加一。
 * publish 控制本包是否生效；tick 将参考绑定到实际求解的观测时刻。
 */
struct Reference {
  std::uint64_t tick = 0;
  bool publish = false;
  std::size_t targetCount = 0;
  std::array<double, 3> targetTimes{};
  std::array<std::array<double, StateDimension>, 3> targetStates{};
  std::array<std::array<double, InputDimension>, 3> targetInputs{};
  std::size_t eventCount = 0;
  std::array<double, 64> eventTimes{};
  std::array<std::size_t, 65> modes{};
  double terminalTime = 0;
  std::array<double, 8> swingParameters{};
  std::array<double, 4> filteredCommand{};
};

/**
 * @brief 一次 MRT 查询的结果，不是整条预测轨迹，也不携带反馈增益矩阵。
 * 包含58维名义状态、35维策略输入、模式及实际查询时间。
 */
struct Policy {
  std::array<double, StateDimension> state{};
  std::array<double, InputDimension> input{};
  // Keep the original numeric mode representation for exact packet identity.
  double mode = 0;
  double queryTime = 0;
};
/**
 * @brief 23 个受控关节的逆动力学前馈力矩；最终 PD 与饱和处理位于外部执行链。
 */
using Torque = std::array<double, ControlledJointCount>;
/**
 * @brief 组合查询结果：同一策略及由该策略计算的前馈力矩。
 */
struct ControlResult {
  Policy policy;
  Torque torque;
};

// Public ABI contains no Eigen, OCS2, Pinocchio, MATLAB or ROS types.
// One instance owns its runtime, solver worker and policy cache. Configuration
// of the retained mathematical authority still uses the accepted environment.
/**
 * @brief 一个实例拥有一套运行时、求解工作线程及当前策略缓存。
 * 典型调用：initialize → setReference → query → inverseDynamics → shutdown。
 * step 是 query 与 inverseDynamics 的组合；同 tick 的重复查询使用缓存。
 * 接口不承诺可由多个调用线程同时操作；内部工作线程同步由实现负责。
 */
class G1Controller {
public:
  G1Controller();
  ~G1Controller();
  G1Controller(const G1Controller &) = delete;
  G1Controller &operator=(const G1Controller &) = delete;

/**
 * @brief 加载模型、OCP、数学实现和日志配置；已有实例须先 shutdown。
 */
  void initialize(const ControllerConfig &config);
/**
 * @brief 更新外部参考缓存；不会在此调用中直接执行一次 SQP。
 */
  void setReference(const Reference &reference);
/**
 * @brief 接收连续物理 tick 的观测；到 MPC 截止时间时等待求解，再查询 t+5ms 策略。
 */
  Policy query(const Observation &observation);
/**
 * @brief 只接受当前 tick 最近查询的原样策略；拒绝陈旧或被修改的数据。
 * 完成此步骤后才允许推进下一物理 tick，重复调用返回已缓存力矩。
 */
  Torque inverseDynamics(const Policy &policy);
/**
 * @brief 按顺序执行 query 与 inverseDynamics，语义与拆分调用一致。
 */
  ControlResult step(const Observation &observation);
/**
 * @brief 释放控制器并等待其工作线程结束；允许重复调用，析构也会调用。
 */
  void shutdown() noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace g1_controller
