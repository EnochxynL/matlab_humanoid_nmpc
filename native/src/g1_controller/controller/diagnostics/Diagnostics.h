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
    std::string columns="control_tick,raw_time,canonical_time";
    for (int j=0;j<118;++j) columns+=",value"+std::to_string(j);
    buffer_.define("planner_samples",columns.c_str());
    buffer_.define("time_identity","raw_backend_time,control_tick,canonical_control_time,raw_minus_canonical,session_id,state_sequence");
    buffer_.define("adapter_consumption","session,epoch,sequence,control_tick,canonical_control_time,target_count,event_count,event_mode_max_error,target_max_error,legacy_filter_calls,legacy_gait_decisions");
  }
  DiagnosticBatch drain() { return buffer_.drain(); }
  void time(uint64_t tick,double raw,double canonical) {
    buffer_.row("time_identity",{raw,tick,canonical,raw-canonical,1,tick});
  }
  void sample(uint64_t tick,double raw,double canonical,const ocs2::vector_t& result) {
    std::vector<DiagnosticValue> row{tick,raw,canonical};
    for(int j=0;j<result.size();++j) row.emplace_back(result[j]);
    buffer_.row("planner_samples",std::move(row));
  }
  void reference(double t,const ocs2::vector4_t& command) { timing_->reference(t,command); }
  void consumed(const ReferencePacket& packet,double t,double error) {
    buffer_.row("adapter_consumption",{packet.session,packet.epoch,packet.sequence,packet.tick,t,
        packet.target.timeTrajectory.size(),packet.schedule.eventTimes.size(),0,error,0,0});
  }
  void trace(const char* event,double t,double deadline,double request) { timing_->trace(event,t,deadline,request); }
  void policy(const ocs2::SystemObservation& observation,const ocs2::PrimalSolution& solution,
              const ocs2::CommandData& command) { structure_->update(observation,solution,command); }
 private:
  DiagnosticBuffer buffer_;
  std::shared_ptr<TimingDiagnostics> timing_;
  std::shared_ptr<StructuralObserver> structure_;
};
} // namespace g1_controller::diagnostics
