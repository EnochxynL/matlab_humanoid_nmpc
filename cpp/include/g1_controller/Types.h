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
inline constexpr std::size_t ControlledJointCount = 23;

/**
 * @brief 初始化资源路径：工程根目录用于定位模型和配置，日志目录用于保存运行证据。
 */
struct ControllerConfig {
  std::string projectRoot;
  std::string logDirectory;
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

/** @brief 显式规划阶段之间传递的身份；由核心创建，调用者不得修改。 */
struct PlannerTick {
  std::uint64_t tick = 0;
  double time = 0;
  double mpcPeriod = 0;
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
} // namespace g1_controller
