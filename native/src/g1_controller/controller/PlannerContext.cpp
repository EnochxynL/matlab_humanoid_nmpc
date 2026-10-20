#include "controller/PlannerContext.h"

#include "controller/diagnostics/Diagnostics.h"


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
      interface.getSwitchedModelReferenceManagerPtr(), currentPacket, diagnostics.get());
  mpc.getSolverPtr()->setReferenceManager(referenceManager_);
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
