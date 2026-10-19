#pragma once
#include <cstdint>
#include <cmath>
#include <stdexcept>

namespace g1_controller {
/** @brief 运行所需的物理时间与连续性合同；不创建文件或执行诊断。 */
class CanonicalControlTime {
 public:
  static constexpr double period = .002;
  static double time(uint64_t tick) {
    if (tick > (uint64_t{1} << 53)) throw std::runtime_error("Tick exceeds exact double integer range");
    return static_cast<double>(tick) * period;
  }
  static double validate(uint64_t tick, double raw) {
    const double canonical = time(tick);
    if (!std::isfinite(raw) || std::abs(raw - canonical) > 1e-12)
      throw std::runtime_error("Raw backend time exceeds canonical physical tick tolerance");
    return canonical;
  }
  double accept(uint64_t tick, double raw, uint64_t session) {
    if (tick != next_ || (started_ && session != session_))
      throw std::runtime_error("Physical tick/session discontinuity");
    const double canonical = validate(tick, raw);
    session_ = session; started_ = true; ++next_;
    return canonical;
  }
 private:
  uint64_t next_ = 0, session_ = 0;
  bool started_ = false;
};
} // namespace g1_controller
