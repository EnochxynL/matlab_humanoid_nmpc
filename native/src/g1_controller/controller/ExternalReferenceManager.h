#pragma once

#include "extensions/MeasuredMode.h"
#include <humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h>
#include <memory>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <array>
#include <cstdint>

namespace g1_controller {
namespace diagnostics { class Diagnostics; }

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


/** 在 OCS2 求解前接入外部参考，完整流程实现在同名 cpp 中。 */
class ExternalReferenceManager final : public ocs2::ReferenceManagerDecorator,
                                       public ::humanoid::ocp_extensions::MeasuredModeReceiver {
 public:
  // 当前参考包和可选诊断由 PlannerContext 持有，必须比求解器存活更久。
  ExternalReferenceManager(
      std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> originalManager,
      const ReferencePacket& currentPacket, diagnostics::Diagnostics* diagnostics);

  void setMeasuredMode(size_t mode) override { ref_->setMeasuredMode(mode); }
  void preSolverRun(double time, double endTime, const ocs2::vector_t& state) override;

 private:
  ReferencePacket referenceForSolve(double observationTime) const;
  void validate(const ReferencePacket& packet, double time, double endTime);

  std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> ref_;
  const ReferencePacket& currentPacket_;
  diagnostics::Diagnostics* diagnostics_;
  ReferencePacket previous_;
  bool initialized_ = false;
};
}  // namespace g1_controller
