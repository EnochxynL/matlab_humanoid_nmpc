#pragma once
#include <humanoid_common_mpc/common/Types.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_sqp/SqpMpc.h>
#include "controller/data/diagnostics/PolicyObserver.h"
#include <chrono>
#include "controller/data/Filesystem.h"
#include <fstream>
#include <iomanip>
#include <mutex>
inline double wallTime(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
#include "controller/data/diagnostics/FootLog.h"
class TimeLog final:public ocs2::SolverSynchronizedModule,public ocs2::DummyObserver {
 public:
 explicit TimeLog(const g1_filesystem::path&root,bool heavy):feet_(root),heavy_(heavy),trace_((root/"timing_trace.csv").string()),solves_((root/"solver_timing.csv").string()),references_((root/"reference.csv").string()),queries_((root/"policy_queries.csv").string()),vectors_((root/"policy_vectors.csv").string()),knots_((root/"policy_knots.csv").string()),load_((root/"logging_load.csv").string()),gait_((root/"gait_schedule.csv").string()),modes_((root/"gait_queries.csv").string()){
  trace_<<"event,control_time_s,scheduled_update_control_time_s,request_id,wall_monotonic_time_s\n";
  solves_<<"solver_target_control_time_s,wall_solver_start_s,wall_solver_end_s,solver_module_wall_duration_s,policy_start_time_s,policy_end_time_s\n";
  references_<<"control_time_s,v_x,v_y,height,yaw_rate\n";
  queries_<<"observation_time_s,policy_query_time_s,policy_reference_time_s,policy_start_time_s,policy_end_time_s,policy_age_control_s,policy_observation_age_control_s,wall_query_time_s\n";
  vectors_<<"observation_time_s,policy_query_time_s,kind,index,value\n";
  knots_<<"solver_target_control_time_s,knot_control_time_s,kind,index,value\n";
  gait_<<"solver_target_control_time_s,kind,index,value\n";modes_<<"observation_time_s,query_control_time_s,observation_mode,policy_mode\n";
  load_<<"observation_time_s,repeat,knot_count,first_state_norm\n";
  for(auto*s:{&trace_,&solves_,&references_,&queries_,&vectors_,&knots_,&load_,&gait_,&modes_}){*s<<std::setprecision(17);if(!*s)throw std::runtime_error("Cannot open Task3D log");}
 }
 void configure(const ocs2::PinocchioInterface&pin,const ocs2::humanoid::MpcRobotModelBase<double>&m,std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager>ref){feet_.configure(pin,m,ref);}
 void trace(const char*event,double t,double deadline,double request){std::lock_guard<std::mutex>lock(mutex_);trace_<<event<<','<<t<<','<<deadline<<','<<request<<','<<wallTime()<<'\n'<<std::flush;}
 void reference(double t,const ocs2::vector4_t&v){references_<<t;for(int i=0;i<4;++i)references_<<','<<v[i];references_<<'\n'<<std::flush;}
 void preSolverRun(ocs2::scalar_t t,ocs2::scalar_t,const ocs2::vector_t&,const ocs2::ReferenceManagerInterface&)override{target_=t;start_=wallTime();}
 void postSolverRun(const ocs2::PrimalSolution&p)override{
  for(size_t k=0;k<p.timeTrajectory_.size();++k)feet_.write(target_,target_,p.timeTrajectory_[k],"policy_knot",p.stateTrajectory_[k],p.inputTrajectory_[std::min(k,p.inputTrajectory_.size()-1)],p.modeSchedule_);
  for(size_t k=0;k<p.modeSchedule_.eventTimes.size();++k)gait_<<target_<<",event_time,"<<k<<','<<p.modeSchedule_.eventTimes[k]<<'\n';
  for(size_t k=0;k<p.modeSchedule_.modeSequence.size();++k)gait_<<target_<<",mode,"<<k<<','<<p.modeSchedule_.modeSequence[k]<<'\n';gait_<<std::flush;
  double end=wallTime();solves_<<target_<<','<<start_<<','<<end<<','<<end-start_<<','<<p.timeTrajectory_.front()<<','<<p.timeTrajectory_.back()<<'\n'<<std::flush;
  for(size_t k=0;k<p.timeTrajectory_.size();++k){for(int j=0;j<p.stateTrajectory_[k].size();++j)knots_<<target_<<','<<p.timeTrajectory_[k]<<",state,"<<j<<','<<p.stateTrajectory_[k][j]<<'\n';if(k<p.inputTrajectory_.size())for(int j=0;j<p.inputTrajectory_[k].size();++j)knots_<<target_<<','<<p.timeTrajectory_[k]<<",input,"<<j<<','<<p.inputTrajectory_[k][j]<<'\n';}knots_<<std::flush;
 }
 void update(const ocs2::SystemObservation&o,const ocs2::PrimalSolution&p,const ocs2::CommandData&c)override{
  const double t=o.time+.005,ref=c.mpcInitObservation_.time;
  if(t<p.timeTrajectory_.front()-1e-12||t>p.timeTrajectory_.back()+1e-12)throw std::runtime_error("Query outside policy horizon");
  modes_<<o.time<<','<<t<<','<<o.mode<<','<<p.modeSchedule_.modeAtTime(t)<<'\n'<<std::flush;
  queries_<<o.time<<','<<t<<','<<ref<<','<<p.timeTrajectory_.front()<<','<<p.timeTrajectory_.back()<<','<<t-ref<<','<<o.time-ref<<','<<wallTime()<<'\n'<<std::flush;
  const auto x=ocs2::LinearInterpolation::interpolate(t,p.timeTrajectory_,p.stateTrajectory_);
  const auto u=ocs2::LinearInterpolation::interpolate(t,p.timeTrajectory_,p.inputTrajectory_);
  feet_.write(ref,o.time,o.time,"actual",o.state,u,p.modeSchedule_);feet_.write(ref,o.time,t,"policy_query",x,u,p.modeSchedule_);
  for(const auto&pair:{std::pair<const char*,const ocs2::vector_t*>("observation_state",&o.state),{"state",&x},{"input",&u}})for(int j=0;j<pair.second->size();++j)vectors_<<o.time<<','<<t<<','<<pair.first<<','<<j<<','<<(*pair.second)[j]<<'\n';vectors_<<std::flush;
  trace("policy_consumed",o.time,ref,0);
  if(heavy_)for(int i=0;i<512;++i)load_<<o.time<<','<<i<<','<<p.timeTrajectory_.size()<<','<<p.stateTrajectory_.front().norm()<<'\n'<<std::flush;
 }
 private:FootLog feet_;bool heavy_;std::ofstream trace_,solves_,references_,queries_,vectors_,knots_,load_,gait_,modes_;std::mutex mutex_;double target_=0,start_=0;
};
