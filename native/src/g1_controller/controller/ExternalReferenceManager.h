#pragma once

#include "controller/PacketManager.h"
#include "extensions/MeasuredMode.h"
#include <humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h>
#include <memory>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>

namespace g1_controller {
namespace diagnostics { class Diagnostics; }

/** 在 OCS2 求解前接入外部参考，完整流程实现在同名 cpp 中。 */
class ExternalReferenceManager final : public ocs2::ReferenceManagerDecorator,
                                       public ::humanoid::ocp_extensions::MeasuredModeReceiver {
 public:
  // 参考包管理器和可选诊断由 G1Controller 持有，必须比求解器存活更久。
  ExternalReferenceManager(
      std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> originalManager,
      PacketManager& packets, diagnostics::Diagnostics* diagnostics);

  void setMeasuredMode(size_t mode) override { ref_->setMeasuredMode(mode); }
  void setObservation(uint64_t tick, double time) { observationTick_ = tick; observationTime_ = time; }
  void preSolverRun(double time, double endTime, const ocs2::vector_t& state) override;

 private:
  std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> ref_;
  PacketManager& packets_;
  diagnostics::Diagnostics* diagnostics_;
  uint64_t observationTick_ = UINT64_MAX;
  double observationTime_ = 0;
};
}  // namespace g1_controller
