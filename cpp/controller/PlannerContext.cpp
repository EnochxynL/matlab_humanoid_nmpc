/**
 * @file
 * @brief 规划上下文的装配实现：构造 OCP/MPC，注册参考接入与诊断回调。
 * 初始化和 configure 不直接求解；工作线程由控制调度器启动。
 */
#include "controller/PlannerContext.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <stdexcept>

namespace g1_controller {

PlannerConfig::PlannerConfig(const std::string &projectRoot, const std::string &logDirectory)
    : backend("simscape"), task(projectRoot + "/config/task.info"),
      ref(projectRoot + "/config/reference.info"),
      urdf(projectRoot + "/assets/g1_29dof.urdf"),
      gait(projectRoot + "/config/gait.info"), mjcf("unused"),
      root(logDirectory), recorded("unused"), horizon(4.8), pacingMs(0),
      startup(0), heavy(false) {}

PlannerContext::PlannerContext(const PlannerConfig &i)
    : interface(i.task, i.urdf, i.ref),
      mpc(interface.mpcSettings(), interface.sqpSettings(),
          interface.getOptimalControlProblem(), interface.getInitializer()),
      log(std::make_shared<TimeLog>(i.root, i.heavy)),
      migrated(std::getenv("TASK5A_AUTHORITY") &&
               std::string(std::getenv("TASK5A_AUTHORITY")) ==
                   "SIMULINK_REFERENCE") {
  {
    std::ifstream maps("/proc/self/maps");
    std::ofstream(i.root + "/loaded_process_maps.txt") << maps.rdbuf();
  }
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
        i.root);
    mpc.getSolverPtr()->setReferenceManager(external);
  } else
    throw std::runtime_error("MEX requires SIMULINK_REFERENCE authority");
  mpc.getSolverPtr()->addSynchronizedModule(log);
  log->configure(interface.getPinocchioInterface(),
                 interface.getMpcRobotModel(),
                 interface.getSwitchedModelReferenceManagerPtr());
  diagnostics::exportConfiguration(interface, i.task, i.root);
  structure = std::make_shared<diagnostics::StructuralObserver>(interface, mpc,
                                                           log, i.root);
  mpc.getSolverPtr()->addSynchronizedModule(structure);
  std::ofstream(i.root + "/reference_authority.txt")
      << (migrated ? "SIMULINK_REFERENCE" : "LEGACY_CPP_REFERENCE") << "\n";
}

void PlannerContext::configure(PlannerExecutor &c) {
  c.enableDeterministicSimulation();
  c.setTraceCallback([this](const char *e, double t, double d, double r) {
    log->trace(e, t, d, r);
  });
}

} // namespace g1_controller
