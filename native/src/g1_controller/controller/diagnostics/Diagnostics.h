#pragma once
#include "StructuralDiagnostics.h"
#include "ProcessModules.h"
#include "g1_controller/Diagnostics.h"

namespace g1_controller::diagnostics {
/** Native diagnostic objects and numeric batches; no file ownership. */
class Diagnostics {
 public:
  template<class Problem>
  Diagnostics(const std::string& taskFile, Problem& interface,
              ocs2::SqpMpc& mpc)
      : timing_(std::make_shared<TimingDiagnostics>(buffer_)) {
    timing_->configure(interface.getPinocchioInterface(), interface.getMpcRobotModel(),
                       interface.getSwitchedModelReferenceManagerPtr());
    auto structure = std::make_shared<StructuralObserver<Problem>>(interface,mpc,timing_,buffer_,observationTick_);
    structure_ = structure;
    mpc.getSolverPtr()->addSynchronizedModule(timing_);
    mpc.getSolverPtr()->addSynchronizedModule(structure);
    buffer_.configurationJson=configurationJson(interface,taskFile);
    buffer_.loadedLibraries=g1_controller::describeLoadedModules();
  }
  DiagnosticBatch drain() { return buffer_.drain(); }
  void observation(uint64_t tick) { observationTick_ = tick; }
  void policy(const ocs2::SystemObservation& observation,const ocs2::PrimalSolution& solution,
              const ocs2::CommandData& command) { structure_->update(observation,solution,command); }
 private:
  uint64_t observationTick_ = 0;
  DiagnosticBuffer buffer_;
  std::shared_ptr<TimingDiagnostics> timing_;
  std::shared_ptr<ocs2::DummyObserver> structure_;
};
} // namespace g1_controller::diagnostics
