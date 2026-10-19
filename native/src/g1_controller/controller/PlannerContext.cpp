/**
 * @file
 * @brief 规划上下文的装配实现：构造 OCP/MPC，注册参考接入与诊断回调。
 * 初始化和 configure 不直接求解；求解时刻由 MATLAB/Simulink 决定。
 */
#include "controller/PlannerContext.h"
#include "controller/data/LoadedLibraries.h"
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
      log(std::make_shared<TimeLog>(i.logDirectory, false)),
      migrated(g1_platform::environmentVariable("TASK5A_AUTHORITY") ==
                   "SIMULINK_REFERENCE") {
  writeLoadedLibraries(i.logDirectory + "/loaded_process_maps.txt");
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
          log->reference(t, p.filtered);
          return p;
        },
        i.logDirectory);
    mpc.getSolverPtr()->setReferenceManager(external);
  } else
    throw std::runtime_error("MEX requires SIMULINK_REFERENCE authority");
  mpc.getSolverPtr()->addSynchronizedModule(log);
  log->configure(interface.getPinocchioInterface(),
                 interface.getMpcRobotModel(),
                 interface.getSwitchedModelReferenceManagerPtr());
  diagnostics::exportConfiguration(interface, i.taskFile, i.logDirectory);
  structure = std::make_shared<diagnostics::StructuralObserver>(interface, mpc,
                                                           log, i.logDirectory);
  mpc.getSolverPtr()->addSynchronizedModule(structure);
  std::ofstream(i.logDirectory + "/reference_authority.txt")
      << (migrated ? "SIMULINK_REFERENCE" : "LEGACY_CPP_REFERENCE") << "\n";
}

void PlannerContext::configure(Ocs2Backend &c) {
  c.setTraceCallback([this](const char *e, double t, double d, double r) {
    log->trace(e, t, d, r);
  });
}

} // namespace g1_controller
