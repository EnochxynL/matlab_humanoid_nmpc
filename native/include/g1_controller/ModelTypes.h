#pragma once
#include <array>
#include <cstddef>

namespace g1_controller {
inline constexpr std::size_t StateDimension = 58;
inline constexpr std::size_t InputDimension = 35;
inline constexpr std::size_t ControlledJointCount = 23;
/** 受控关节的逆动力学前馈力矩；PD 与饱和处理位于外部执行链。 */
using Torque = std::array<double, ControlledJointCount>;
}  // namespace g1_controller
