#pragma once
#include "extensions/TermRegistry.h"
// Read-only clone evaluation: never writes an active solver OCP or reference.

#include "DiagnosticBuffer.h"
#include "PolicyDiagnosticEvaluator.h"
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <limits>
#include <type_traits>
#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>
#include "extensions/StateInputSoftConstraint.h"
#include "extensions/StateSoftConstraint.h"
namespace g1_controller::diagnostics {
using namespace ocs2;using namespace ocs2::humanoid;
// 独立 OCP 克隆求值与全身模型诊断，不触发其他诊断对象。
template<class Problem>
class OcpDiagnostics final:public SolverSynchronizedModule,public PolicyDiagnosticEvaluator {
 Problem&interface_;SqpMpc&mpc_;
 OptimalControlProblem clone_;PinocchioInterface pin_;std::unique_ptr<MpcRobotModelBase<double>>model_;
 DiagnosticBuffer& buffer_;vector_t lastDerivative_;double lastSampleTime_=0;std::array<double,8>lastSolver_{};
 double target_=0;size_t previousIterations_=0;
 const uint64_t& observationTick_;
 void scalarCost(const char*group,const std::string&name,double t,double value){
  if(!std::isfinite(value))throw std::runtime_error("Nonfinite read-only cost");
  buffer_.row("structural_costs",{target_,t,group,name,value});
 }
 public:
 OcpDiagnostics(Problem&i,SqpMpc&m,DiagnosticBuffer&buffer,const uint64_t& observationTick)
  :interface_(i),mpc_(m),clone_(i.getOptimalControlProblem()),pin_(i.getPinocchioInterface()),model_(i.getMpcRobotModel().clone()),buffer_(buffer),lastDerivative_(vector_t::Zero(i.getMpcRobotModel().getStateDim())),observationTick_(observationTick){
  buffer_.define("structural_solver","control_tick,observation_time,iterations_this_solve,total_iterations,objective,dynamics_sse,equality_sse,inequality_sse,lq_wall_ms,qp_wall_ms,linesearch_wall_ms,controller_wall_ms,status");
  buffer_.define("structural_dynamics","solver_time,sample_time,index,derivative,A_rows,A_cols,B_rows,B_cols");
  buffer_.define("structural_costs","solver_time,sample_time,group,term,value");
  buffer_.define("structural_constraints","solver_time,sample_time,term,active,dimension,index,residual");
  if constexpr (std::is_same_v<Problem, WBMpcInterface>) buffer_.define("structural_id","observation_time,query_time,index,q_des,qd_des,qdd_des,tau_ff,left_wrench,right_wrench");
 }
 void preSolverRun(double t,double,const vector_t&,const ReferenceManagerInterface&)override{target_=t;}
 void postSolverRun(const PrimalSolution&p)override{
  auto*solver=mpc_.getSolverPtr();auto perf=solver->getPerformanceIndeces();auto n=solver->getNumIterations();// Official OCS2 does not expose per-solve internal timers. Never invent them.
  const double unknown=std::numeric_limits<double>::quiet_NaN();
  struct {double linearQuadraticApproximationTime,solveQpTime,linesearchTime,computeControllerTime;} times{unknown,unknown,unknown,unknown};
  // Public API exposes performance and iteration count; convergence enum is private.
  buffer_.row("structural_solver",{observationTick_,target_,n-previousIterations_,n,perf.cost,perf.dynamicsViolationSSE,perf.equalityConstraintsSSE,perf.inequalityConstraintsSSE,times.linearQuadraticApproximationTime,times.solveQpTime,times.linesearchTime,times.computeControllerTime,"POLICY_RETURNED_CONVERGENCE_ENUM_AND_INTERNAL_TIMERS_UNAVAILABLE"});previousIterations_=n;lastSolver_={double(n),perf.cost,perf.dynamicsViolationSSE,perf.equalityConstraintsSSE,times.linearQuadraticApproximationTime,times.solveQpTime,times.linesearchTime,times.computeControllerTime};
  clone_.targetTrajectoriesPtr=&interface_.getSwitchedModelReferenceManagerPtr()->getTargetTrajectories();
  // One interior policy knot per solve. Objects and precomputation are cloned,
  // sharing only the unchanged reference manager after solver completion.
  size_t k=std::min(p.timeTrajectory_.size()/2,p.inputTrajectory_.size()-1);double t=p.timeTrajectory_[k];auto&x=p.stateTrajectory_[k];auto&u=p.inputTrajectory_[k];auto&pc=*clone_.preComputationPtr;
  pc.request(Request::Cost+Request::Constraint+Request::SoftConstraint,t,x,u);
  auto lin=clone_.dynamicsPtr->linearApproximation(t,x,u,pc);lastDerivative_=lin.f;lastSampleTime_=t;
  for(int j=0;j<lin.f.size();++j)buffer_.row("structural_dynamics",{target_,t,j,lin.f[j],lin.dfdx.rows(),lin.dfdx.cols(),lin.dfdu.rows(),lin.dfdu.cols()});
  double total=0;
  auto inputCosts=[&](const char*group,auto&collection){for(auto&[name,index]: ::humanoid::ocp_extensions::termNames(*collection)){auto&term=collection->get(name);double v=term.isActive(t)?term.getValue(t,x,u,*clone_.targetTrajectoriesPtr,pc):0;scalarCost(group,name,t,v);total+=v;}};
  inputCosts("running",clone_.costPtr);inputCosts("input_soft",clone_.softConstraintPtr);
  for(auto&[name,index]: ::humanoid::ocp_extensions::termNames(*clone_.stateSoftConstraintPtr)){auto&term=clone_.stateSoftConstraintPtr->get(name);double v=term.isActive(t)?term.getValue(t,x,*clone_.targetTrajectoriesPtr,pc):0;scalarCost("state_soft",name,t,v);total+=v;}
  scalarCost("running_total","sum_running_and_soft",t,total);
  for(auto&[name,index]: ::humanoid::ocp_extensions::termNames(*clone_.equalityConstraintPtr)){auto&term=clone_.equalityConstraintPtr->get(name);bool active=term.isActive(t);auto dim=active?term.getNumConstraints(t):0;if(active){auto r=term.getValue(t,x,u,pc);for(int j=0;j<r.size();++j)buffer_.row("structural_constraints",{target_,t,name,1,dim,j,r[j]});}else buffer_.row("structural_constraints",{target_,t,name,0,0,-1,0});}
  for(auto&[name,index]: ::humanoid::ocp_extensions::termNames(*clone_.softConstraintPtr)){auto&wrapper=clone_.softConstraintPtr->get<::humanoid::ocp_extensions::StateInputSoftConstraint>(name);auto&term=wrapper.get();bool active=term.isActive(t);if(active){auto r=term.getValue(t,x,u,pc);for(int j=0;j<r.size();++j)buffer_.row("structural_constraints",{target_,t,name,1,r.size(),j,r[j]});}else buffer_.row("structural_constraints",{target_,t,name,0,0,-1,0});}
  // Terminal contribution is separately evaluated, never added to instantaneous running total.
  double tf=p.timeTrajectory_.back();auto&xf=p.stateTrajectory_.back();pc.requestFinal(Request::Cost,tf,xf);
  for(auto&[name,index]: ::humanoid::ocp_extensions::termNames(*clone_.finalCostPtr))scalarCost("terminal",name,tf,clone_.finalCostPtr->get(name).getValue(tf,xf,*clone_.targetTrajectoriesPtr,pc));

 }
 void evaluatePolicy(const SystemObservation&o,const PrimalSolution&p)override{
  // 通用 OCP 诊断适用于两种问题；以下力矩和 218 维快照仅适用于全身模型。
  if constexpr (std::is_same_v<Problem, WBMpcInterface>) {
  auto& model = static_cast<WBAccelMpcRobotModel<double>&>(*model_);double t=o.time+.005;auto x=LinearInterpolation::interpolate(t,p.timeTrajectory_,p.stateTrajectory_);auto u=LinearInterpolation::interpolate(t,p.timeTrajectory_,p.inputTrajectory_);
  auto tau=computeJointTorques<double>(x,u,pin_,model);auto q=model.getJointAngles(x);auto qd=model.getJointVelocities(x,u);auto qdd=model.getJointAccelerations(u);
  for(int j=0;j<23;++j)buffer_.row("structural_id",{o.time,t,j,q[j],qd[j],qdd[j],tau[j],j<6?u[j]:0,j<6?u[j+6]:0});
  auto accel=computeGeneralizedAccelerations<double>(x,u,pin_,model);std::vector<double>snapshot{1,o.time,t,p.timeTrajectory_.front(),p.timeTrajectory_.back(),double(p.modeSchedule_.modeAtTime(t))};
  for(auto*v:{&x,&u,&tau,&accel})for(int j=0;j<v->size();++j)snapshot.push_back((*v)[j]);for(auto v:lastSolver_)snapshot.push_back(v);snapshot.push_back(lastSampleTime_);for(int j=0;j<58;++j)snapshot.push_back(lastDerivative_[j]);
  if(snapshot.size()!=218)throw std::runtime_error("Structural snapshot shape");
  buffer_.snapshot=std::move(snapshot);
  }
 }
};
}
