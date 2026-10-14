#include "extensions/MeasuredMode.h"
#pragma once
#include <memory>
#include <array>
#include <algorithm>
#include "data/time/CanonicalControlTime.h"
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h>
#include <functional>
#include <fstream>
#include <iomanip>
namespace task5a {
struct Packet {
 Packet(){schedule.clear();}
 uint64_t tick=0,session=1,epoch=0,sequence=0;
 ocs2::TargetTrajectories target;
 ocs2::ModeSchedule schedule;
 ocs2::vector4_t filtered=ocs2::vector4_t::Zero();
 double terminal=0;
 std::array<double,8>swing={.05,-0.0,.08,-.001,.4,-.15,.3,.005};
};
// Preserve the original reference-manager object already held by OCP costs,
// constraints and precomputation. The public GaitSchedule supports an empty
// template (continue forever): install it once, then feed the sender's complete
// raw schedule plus a removable terminal marker. Existing getModeSchedule pops
// only this marker; no C++ template tiling, gait decision or filter runs.
// The original terrain adaptation, swing splines and OCP consumers remain.
class ExternalReferenceAdapter final:public ocs2::ReferenceManagerDecorator, public g1::MeasuredModeReceiver {
 public:
 void setMeasuredMode(size_t mode) override {ref_->setMeasuredMode(mode);}
 using Provider=std::function<Packet(double)>;
 ExternalReferenceAdapter(std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager>ref,Provider provider,const std::string&folder)
 :ReferenceManagerDecorator(ref),ref_(ref),provider_(std::move(provider)),log_(folder+"/adapter_consumption.csv"){
  ocs2::humanoid::ModeSequenceTemplate empty({0.,1.},{3});empty.modeSequence.clear();empty.switchingTimes.clear();
  ref_->getGaitSchedule()->insertModeSequenceTemplate(empty,0.,0.);
  log_<<std::setprecision(17)<<"session,epoch,sequence,control_tick,canonical_control_time,target_count,event_count,event_mode_max_error,target_max_error,legacy_filter_calls,legacy_gait_decisions\n";
 }
 void preSolverRun(double t,double end,const ocs2::vector_t&x)override{
  Packet p=provider_(t);validate(p,t,end);
  auto padded=p.schedule;padded.eventTimes.push_back(p.terminal);padded.modeSequence.push_back(3);
  ref_->getGaitSchedule()->updateModeSchedule(padded);ref_->setTargetTrajectories(p.target);
  ref_->preSolverRun(t,end,x);
  const auto&s=ref_->getModeSchedule();const auto&target=ref_->getTargetTrajectories();
  if(s.eventTimes!=p.schedule.eventTimes||s.modeSequence!=p.schedule.modeSequence)throw std::runtime_error("H: Adapter altered the authoritative raw schedule");
  double error=0;if(target.timeTrajectory!=p.target.timeTrajectory)throw std::runtime_error("H: Adapter changed target knot times");
  for(size_t k=0;k<target.stateTrajectory.size();++k)error=std::max(error,(target.stateTrajectory[k]-p.target.stateTrajectory[k]).lpNorm<Eigen::Infinity>());
  if(error!=0)throw std::runtime_error("H: Adapter changed target values");
  log_<<p.session<<','<<p.epoch<<','<<p.sequence<<','<<p.tick<<','<<t<<','<<p.target.timeTrajectory.size()<<','<<s.eventTimes.size()<<",0,"<<error<<",0,0\n"<<std::flush;
  previous_=p;initialized_=true;
 }
 private:
 void validate(const Packet&p,double t,double end){
  if(p.session==0||p.sequence==0||CanonicalControlTime::time(p.tick)!=t)throw std::runtime_error("G: Reference identity rejected");
  if(initialized_&&(p.session!=previous_.session||p.epoch!=previous_.epoch||p.sequence!=previous_.sequence+1||p.tick<=previous_.tick))throw std::runtime_error("G: Reference session/order rejected; fresh process per epoch");
  auto n=p.target.timeTrajectory.size();if((n!=1&&n!=3)||p.target.stateTrajectory.size()!=n||p.target.inputTrajectory.size()!=n)throw std::runtime_error("G: Target dimensions rejected");
  for(size_t k=0;k<n;++k){if(!std::isfinite(p.target.timeTrajectory[k])||(k&&p.target.timeTrajectory[k]<=p.target.timeTrajectory[k-1])||p.target.stateTrajectory[k].size()!=58||p.target.inputTrajectory[k].size()!=35||!p.target.stateTrajectory[k].allFinite()||!p.target.inputTrajectory[k].allFinite())throw std::runtime_error("G: Nonfinite or malformed target");}
  const auto&e=p.schedule.eventTimes;const auto&m=p.schedule.modeSequence;
  if(e.empty()||e.size()>64||m.size()!=e.size()+1||!std::is_sorted(e.begin(),e.end()))throw std::runtime_error("G: Schedule shape/order rejected");
  for(double v:e)if(!std::isfinite(v))throw std::runtime_error("G: Nonfinite event");for(auto v:m)if(v>3)throw std::runtime_error("G: Unknown mode");
  if(!std::isfinite(p.terminal)||p.terminal<=e.back()||p.terminal<=end+(end-t))throw std::runtime_error("G: Terminal marker is not outside the reference window");
  const std::array<double,8>expected={.05,-0.0,.08,-.001,.4,-.15,.3,.005};if(p.swing!=expected)throw std::runtime_error("F: Swing parameters changed from the accepted consumer configuration");
 }
 std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager>ref_;Provider provider_;std::ofstream log_;Packet previous_;bool initialized_=false;
};
}
