#pragma once
#include "OcpDiagnostics.h"
#include "PolicyDiagnostics.h"
#include "ProblemConfiguration.h"
#include "ProcessModules.h"
#include "g1_controller/Diagnostics.h"

namespace g1_controller::diagnostics {
/**
 * 原生求解诊断：在对象所在层采集，保留原生回调的时间与数据语义。
 * 策略、配置和求解统计统一进入批次；绑定层只编码批次，不展开读取接口。
 * 应用协议诊断和文件输出由 MATLAB 管理，此类不拥有文件。
 */
class Diagnostics {
 public:
  template<class Problem>
  Diagnostics(const std::string& taskFile, Problem& interface,
              ocs2::SqpMpc& mpc)
      : policy_(std::make_shared<PolicyDiagnostics>(buffer_)) {
    policy_->configure(interface.getPinocchioInterface(), interface.getMpcRobotModel(),
                       interface.getSwitchedModelReferenceManagerPtr());
    auto ocp = std::make_shared<OcpDiagnostics<Problem>>(interface,mpc,buffer_,observationTick_);
    ocp_ = ocp;
    mpc.getSolverPtr()->addSynchronizedModule(policy_);
    mpc.getSolverPtr()->addSynchronizedModule(ocp);
    buffer_.configurationJson=describeProblemConfiguration(interface,taskFile);
    buffer_.loadedLibraries=g1_controller::describeLoadedModules();
  }
  DiagnosticBatch drain() { return buffer_.drain(); }
  void observation(uint64_t tick) { observationTick_ = tick; }
  void policy(const ocs2::SystemObservation& observation,const ocs2::PrimalSolution& solution,
              const ocs2::CommandData& command) {
    // 显式组织调用，保持原先策略诊断在前、OCP 诊断在后的采样顺序。
    policy_->evaluatePolicy(observation,solution,command);
    ocp_->evaluatePolicy(observation,solution);
  }
 private:
  uint64_t observationTick_ = 0;
  DiagnosticBuffer buffer_;
  std::shared_ptr<PolicyDiagnostics> policy_;
  std::shared_ptr<PolicyDiagnosticEvaluator> ocp_;
};
} // namespace g1_controller::diagnostics
