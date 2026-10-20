#pragma once
#include <humanoid_common_mpc/common/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <array>
#include <cstdint>
#include <utility>

namespace g1_controller {
/** 外部发布的一次参考：身份、目标轨迹、接触时间表和摆腿配置。 */
struct ReferencePacket {
  ReferencePacket() { schedule.clear(); }

  uint64_t tick = 0, session = 1, epoch = 0, sequence = 0;
  ocs2::TargetTrajectories target;
  ocs2::ModeSchedule schedule;
  ocs2::vector4_t filtered = ocs2::vector4_t::Zero();
  // 位于参考窗口之外的结束标记；不是实际步态切换时刻。
  double terminal = 0;
  std::array<double, 8> swing = {.05, -0.0, .08, -.001, .4, -.15, .3, .005};
};



/** 保存参考包及消费历史，集中维护参考协议；不执行求解或诊断。 */
class PacketManager {
 public:
  PacketManager(std::size_t stateDimension, std::size_t inputDimension, std::array<double,8> swing)
      : stateDimension_(stateDimension), inputDimension_(inputDimension), swing_(swing) {}
  uint64_t currentTick() const { return current_.tick; }
  void publish(ReferencePacket packet) { current_ = std::move(packet); }
  ReferencePacket referenceForSolve(uint64_t observationTick) const;
  void validate(const ReferencePacket& packet, double time, double endTime);
  void markConsumed(const ReferencePacket& packet);

 private:
  std::size_t stateDimension_, inputDimension_;
  std::array<double,8> swing_;
  ReferencePacket current_;
  ReferencePacket previous_;
  bool initialized_ = false;
};
}  // namespace g1_controller
