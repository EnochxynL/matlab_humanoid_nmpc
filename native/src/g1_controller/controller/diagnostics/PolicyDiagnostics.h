#pragma once
#include <humanoid_common_mpc/common/Types.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_mpc/CommandData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <chrono>
#include "FootDiagnostics.h"

namespace g1_controller::diagnostics {
// 求解计时、策略轨迹与查询采样；足端计算使用独立工作区。
inline double wallTime(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class PolicyDiagnostics final:public ocs2::SolverSynchronizedModule {
 public:
 explicit PolicyDiagnostics(g1_controller::diagnostics::DiagnosticBuffer& buffer):feet_(buffer),buffer_(buffer){
  buffer_.define("solver_timing","solver_target_control_time_s,wall_solver_start_s,wall_solver_end_s,solver_module_wall_duration_s,policy_start_time_s,policy_end_time_s");
  buffer_.define("policy_queries","observation_time_s,policy_query_time_s,policy_reference_time_s,policy_start_time_s,policy_end_time_s,policy_age_control_s,policy_observation_age_control_s,wall_query_time_s");
  buffer_.define("policy_vectors","observation_time_s,policy_query_time_s,kind,index,value");
  buffer_.define("policy_knots","solver_target_control_time_s,knot_control_time_s,kind,index,value");
  buffer_.define("gait_schedule","solver_target_control_time_s,kind,index,value");
 }
 void configure(const ocs2::PinocchioInterface&pin,const ocs2::humanoid::MpcRobotModelBase<double>&m,std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager>ref){feet_.configure(pin,m,ref);}
 void preSolverRun(ocs2::scalar_t t,ocs2::scalar_t,const ocs2::vector_t&,const ocs2::ReferenceManagerInterface&)override{target_=t;start_=wallTime();}
 void postSolverRun(const ocs2::PrimalSolution&p)override{
  for(size_t k=0;k<p.timeTrajectory_.size();++k)feet_.write(target_,target_,p.timeTrajectory_[k],"policy_knot",p.stateTrajectory_[k],p.inputTrajectory_[std::min(k,p.inputTrajectory_.size()-1)],p.modeSchedule_);
  for(size_t k=0;k<p.modeSchedule_.eventTimes.size();++k)buffer_.row("gait_schedule",{target_,"event_time",k,p.modeSchedule_.eventTimes[k]});
  for(size_t k=0;k<p.modeSchedule_.modeSequence.size();++k)buffer_.row("gait_schedule",{target_,"mode",k,p.modeSchedule_.modeSequence[k]});
  double end=wallTime();buffer_.row("solver_timing",{target_,start_,end,end-start_,p.timeTrajectory_.front(),p.timeTrajectory_.back()});
  for(size_t k=0;k<p.timeTrajectory_.size();++k){
   for(int j=0;j<p.stateTrajectory_[k].size();++j)buffer_.row("policy_knots",{target_,p.timeTrajectory_[k],"state",j,p.stateTrajectory_[k][j]});
   if(k<p.inputTrajectory_.size())for(int j=0;j<p.inputTrajectory_[k].size();++j)buffer_.row("policy_knots",{target_,p.timeTrajectory_[k],"input",j,p.inputTrajectory_[k][j]});
  }

 }
 void evaluatePolicy(const ocs2::SystemObservation&o,const ocs2::PrimalSolution&p,const ocs2::CommandData&c){
  const double t=o.time+.005,ref=c.mpcInitObservation_.time;
  if(t<p.timeTrajectory_.front()-1e-12||t>p.timeTrajectory_.back()+1e-12)throw std::runtime_error("Query outside policy horizon");
  buffer_.row("policy_queries",{o.time,t,ref,p.timeTrajectory_.front(),p.timeTrajectory_.back(),t-ref,o.time-ref,wallTime()});
  const auto x=ocs2::LinearInterpolation::interpolate(t,p.timeTrajectory_,p.stateTrajectory_);
  const auto u=ocs2::LinearInterpolation::interpolate(t,p.timeTrajectory_,p.inputTrajectory_);
  feet_.write(ref,o.time,o.time,"actual",o.state,u,p.modeSchedule_);feet_.write(ref,o.time,t,"policy_query",x,u,p.modeSchedule_);
  for(const auto&pair:{std::pair<const char*,const ocs2::vector_t*>("observation_state",&o.state),{"state",&x},{"input",&u}})for(int j=0;j<pair.second->size();++j)buffer_.row("policy_vectors",{o.time,t,pair.first,j,(*pair.second)[j]});
 }
 private:FootDiagnostics feet_;g1_controller::diagnostics::DiagnosticBuffer& buffer_;double target_=0,start_=0;

};

} // namespace g1_controller::diagnostics
