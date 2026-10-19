/**
 * @file
 * @brief 规划上下文的装配实现：构造 OCP/MPC，注册参考接入与诊断回调。
 * 初始化和 configure 不直接求解；求解时刻由 MATLAB/Simulink 决定。
 */
#include "controller/PlannerContext.h"
#include "controller/data/RunDiagnostics.h"
#include "extensions/Environment.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <stdexcept>

namespace g1_controller {

PlannerContext::PlannerContext(const ControllerConfig &i)
    : interface(i.taskFile, i.urdfFile, i.referenceFile),
      mpc(interface.mpcSettings(), interface.sqpSettings(),
          interface.getOptimalControlProblem(), interface.getInitializer()),
      migrated(g1_platform::environmentVariable("TASK5A_AUTHORITY") ==
                   "SIMULINK_REFERENCE") {
  if (i.diagnosticsEnabled)
    diagnostics = std::make_unique<diagnostics::RunDiagnostics>(i, interface, mpc);
  if (migrated) {
    external = std::make_shared<ExternalReferenceManager>(
        interface.getSwitchedModelReferenceManagerPtr(),
        [this](double t) {
          uint64_t tick = std::llround(t / .002);
          auto p = currentPacket;
          if (expectedSession && p.session != expectedSession)
            throw std::runtime_error(
                "G: Reference and PlantState sessions disagree");
          if (p.tick != tick)
            throw std::runtime_error(
                "G: Reference tick does not match actual solver observation");
          if (diagnostics) diagnostics->reference(t, p.filtered);
          return p;
        },
        [this](const ReferencePacket& p, double t, double error) {
          if (diagnostics) diagnostics->consumed(p, t, error);
        });
    mpc.getSolverPtr()->setReferenceManager(external);
  } else
    throw std::runtime_error("MEX requires SIMULINK_REFERENCE authority");
}

PlannerContext::~PlannerContext() = default;

void PlannerContext::configure(Ocs2Backend &c) {
  if (!diagnostics) return;
  c.setTraceCallback([this](const char *e, double t, double d, double r) {
    diagnostics->trace(e, t, d, r);
  });
  c.setPolicyCallback([this](const auto& observation, const auto& solution, const auto& command) {
    diagnostics->policy(observation, solution, command);
  });
}

} // namespace g1_controller
