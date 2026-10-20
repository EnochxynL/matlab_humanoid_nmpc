#include "controller/PlannerContext.h"

#include "controller/CanonicalControlTime.h"
#include "controller/diagnostics/Diagnostics.h"

#include <stdexcept>

namespace g1_controller {

PlannerContext::PlannerContext(const ControllerConfig& config)
    : interface(config.taskFile, config.urdfFile, config.referenceFile),
      mpc(interface.mpcSettings(), interface.sqpSettings(),
          interface.getOptimalControlProblem(), interface.getInitializer()) {
  // 诊断仅观察已有对象；关闭后仍建立同一套 OCP 和外部参考接入。
  if (config.diagnosticsEnabled) {
    diagnostics = std::make_unique<diagnostics::Diagnostics>(config, interface, mpc);
  }
  installReferenceManager();
}

// Diagnostics 在头文件中只有前置声明，因此在这里定义析构函数。
PlannerContext::~PlannerContext() = default;

void PlannerContext::installReferenceManager() {
  referenceManager_ = std::make_shared<ExternalReferenceManager>(
      // 1. 取得原项目已有的参考管理器。
      interface.getSwitchedModelReferenceManagerPtr(),
      // 2. 定义“求解前如何取得参考包”。
      [this](double observationTime) {
        return referenceForSolve(observationTime);
      },
      // 3. 定义“参考被消费后如何记录诊断”。
      [this](const ReferencePacket& packet, double observationTime, double error) {
        if (diagnostics) diagnostics->consumed(packet, observationTime, error);
      });
  // 告诉 OCS2，求解时使用这个适配器。
  mpc.getSolverPtr()->setReferenceManager(referenceManager_);
}

ReferencePacket PlannerContext::referenceForSolve(double observationTime) const {
  // 返回本轮参考的副本；校验依据是实际求解观测，而非调度截止时间。
  // 当前运行协议每次初始化建立一个 session，编号固定为 1。
  auto packet = currentPacket;
  if (packet.session != SessionId) {
    throw std::runtime_error("Reference session does not match the controller session");
  }
  if (CanonicalControlTime::time(packet.tick) != observationTime) {
    throw std::runtime_error("Reference tick does not match the solver observation time");
  }
  if (diagnostics) diagnostics->reference(observationTime, packet.filtered);
  return packet;
}

void PlannerContext::attachBackendDiagnostics(Ocs2Backend& backend) {
  if (!diagnostics) return;

  backend.setTraceCallback(
      [this](const char* event, double time, double deadline, double request) {
        diagnostics->trace(event, time, deadline, request);
      });
  backend.setPolicyCallback(
      [this](const auto& observation, const auto& solution, const auto& command) {
        diagnostics->policy(observation, solution, command);
      });
}

} // namespace g1_controller
