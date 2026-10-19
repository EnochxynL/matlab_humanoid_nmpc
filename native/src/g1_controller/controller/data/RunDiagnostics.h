#pragma once
#include "controller/data/diagnostics/StructuralObserver.h"
#include "controller/data/LoadedLibraries.h"
#include "controller/reference/ExternalReferenceManager.h"
#include "g1_controller/G1Controller.h"

namespace g1_controller::diagnostics {
/** @brief 可选诊断会话：独占应用层文件输出和额外 OCP 求值。 */
class RunDiagnostics {
 public:
  RunDiagnostics(const ControllerConfig& config, ocs2::humanoid::WBMpcInterface& interface,
                 ocs2::SqpMpc& mpc)
      : root_(prepareDirectory(config.logDirectory)),
        log_(std::make_shared<TimeLog>(root_, false)),
        samples_(root_ + "/planner_samples.csv"),
        time_(root_ + "/time_identity.csv"),
        consumption_(root_ + "/adapter_consumption.csv") {
    log_->configure(interface.getPinocchioInterface(), interface.getMpcRobotModel(),
                    interface.getSwitchedModelReferenceManagerPtr());
    structure_ = std::make_shared<StructuralObserver>(interface, mpc, log_, root_);
    mpc.getSolverPtr()->addSynchronizedModule(log_);
    mpc.getSolverPtr()->addSynchronizedModule(structure_);
    exportConfiguration(interface, config.taskFile, root_);
    writeLoadedLibraries(root_ + "/loaded_process_maps.txt");
    std::ofstream(root_ + "/reference_authority.txt") << "SIMULINK_REFERENCE\n";
    samples_ << std::setprecision(17) << "control_tick,raw_time,canonical_time";
    for (int j = 0; j < 118; ++j) samples_ << ",value" << j;
    samples_ << '\n';
    time_ << std::setprecision(17)
          << "raw_backend_time,control_tick,canonical_control_time,raw_minus_canonical,session_id,state_sequence\n";
    consumption_ << std::setprecision(17)
          << "session,epoch,sequence,control_tick,canonical_control_time,target_count,event_count,event_mode_max_error,target_max_error,legacy_filter_calls,legacy_gait_decisions\n";
  }
  void time(uint64_t tick, double raw, double canonical) {
    time_ << raw << ',' << tick << ',' << canonical << ',' << raw-canonical << ",1," << tick << '\n' << std::flush;
  }
  void sample(uint64_t tick, double raw, double canonical, const ocs2::vector_t& result) {
    samples_ << tick << ',' << raw << ',' << canonical;
    for (int j = 0; j < result.size(); ++j) samples_ << ',' << result[j];
    samples_ << '\n' << std::flush;
  }
  void reference(double t, const ocs2::vector4_t& command) { log_->reference(t, command); }
  void consumed(const ReferencePacket& packet, double t, double error) {
    consumption_ << packet.session << ',' << packet.epoch << ',' << packet.sequence << ',' << packet.tick
        << ',' << t << ',' << packet.target.timeTrajectory.size() << ','
        << packet.schedule.eventTimes.size() << ",0," << error << ",0,0\n" << std::flush;
  }
  void trace(const char* event, double t, double deadline, double request) { log_->trace(event,t,deadline,request); }
  void policy(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& solution,
              const ocs2::CommandData& command) { structure_->update(observation,solution,command); }

 private:
  static std::string prepareDirectory(const std::string& folder) {
    g1_filesystem::create_directories(folder);
    return folder;
  }
  std::string root_;
  std::shared_ptr<TimeLog> log_;
  std::shared_ptr<StructuralObserver> structure_;
  std::ofstream samples_, time_, consumption_;
};
} // namespace g1_controller::diagnostics
