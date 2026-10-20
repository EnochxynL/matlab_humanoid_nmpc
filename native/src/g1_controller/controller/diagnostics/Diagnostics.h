#pragma once
#include "StructuralDiagnostics.h"
#include "ProcessModules.h"
#include "controller/PacketManager.h"
#include "g1_controller/G1Controller.h"

namespace g1_controller::diagnostics {
/** Native diagnostic objects and numeric batches; no file ownership. */
class Diagnostics {
 public:
  Diagnostics(const ControllerConfig& config, ocs2::humanoid::WBMpcInterface& interface,
              ocs2::SqpMpc& mpc)
      : timing_(std::make_shared<TimingDiagnostics>(buffer_)) {
    timing_->configure(interface.getPinocchioInterface(), interface.getMpcRobotModel(),
                       interface.getSwitchedModelReferenceManagerPtr());
    structure_ = std::make_shared<StructuralObserver>(interface,mpc,timing_,buffer_);
    mpc.getSolverPtr()->addSynchronizedModule(timing_);
    mpc.getSolverPtr()->addSynchronizedModule(structure_);
    buffer_.configurationJson=configurationJson(interface,config.taskFile);
    buffer_.loadedLibraries=g1_controller::describeLoadedModules();
    buffer_.define("adapter_consumption","session,epoch,sequence,control_tick,canonical_control_time,target_count,event_count,event_mode_max_error,target_max_error,legacy_filter_calls,legacy_gait_decisions");
  }
  DiagnosticBatch drain() { return buffer_.drain(); }
  void reference(double t,const ocs2::vector4_t& command) { timing_->reference(t,command); }
  void consumed(const ReferencePacket& packet,double t,double error) {
    buffer_.row("adapter_consumption",{packet.session,packet.epoch,packet.sequence,packet.tick,t,
        packet.target.timeTrajectory.size(),packet.schedule.eventTimes.size(),0,error,0,0});
  }
  void trace(const char* event,double time) { timing_->trace(event,time); }
  void policy(const ocs2::SystemObservation& observation,const ocs2::PrimalSolution& solution,
              const ocs2::CommandData& command) { structure_->update(observation,solution,command); }
 private:
  DiagnosticBuffer buffer_;
  std::shared_ptr<TimingDiagnostics> timing_;
  std::shared_ptr<StructuralObserver> structure_;
};
} // namespace g1_controller::diagnostics
