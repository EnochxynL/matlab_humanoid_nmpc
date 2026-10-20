#pragma once
#include <cstdint>
#include <cmath>
#include <stdexcept>

namespace g1_controller {
// 无状态的时间换算与校验；tick 连续性由 G1Controller 管理。
namespace control_time {
  inline constexpr double period = .002;
  inline double fromTick(uint64_t tick) {
    if (tick > (uint64_t{1} << 53)) throw std::runtime_error("Tick exceeds exact double integer range");
    return static_cast<double>(tick) * period;
  }
  inline double validateRawTime(uint64_t tick, double raw) {
    const double canonical = fromTick(tick);
    if (!std::isfinite(raw) || std::abs(raw - canonical) > 1e-12)
      throw std::runtime_error("Raw backend time exceeds canonical physical tick tolerance");
    return canonical;
  }
} // namespace control_time
} // namespace g1_controller
