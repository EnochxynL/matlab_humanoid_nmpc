/**
 * @file
 * @brief 控制核心公开数据类型；仅依赖标准 C++，供控制器和适配器共同使用。
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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

} // namespace g1_controller
