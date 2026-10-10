#pragma once
// Read-only clone evaluation: never writes an active solver OCP or reference.
#include "diagnostics/TimeLog.h"
#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
namespace task5b {
using namespace ocs2;using namespace ocs2::humanoid;
inline void exportConfiguration(WBMpcInterface& i,const std::string&task,const std::string&folder){
 boost::property_tree::ptree raw,out;boost::property_tree::read_info(task,raw);out.add_child("source_config",raw);
 auto&s=i.sqpSettings();auto&m=i.mpcSettings();out.put("state_dim",i.getMpcRobotModel().getStateDim());out.put("input_dim",i.getMpcRobotModel().getInputDim());
 out.put("loaded.horizon",m.timeHorizon_);out.put("loaded.mpc_hz",m.mpcDesiredFrequency_);out.put("loaded.mrt_hz",m.mrtDesiredFrequency_);out.put("loaded.cold_start",m.coldStart_);
 #define FIELD(name) out.put("loaded.sqp." #name,s.name)
 FIELD(sqpIteration);FIELD(deltaTol);FIELD(costTol);FIELD(alpha_decay);FIELD(alpha_min);FIELD(gamma_c);FIELD(g_max);FIELD(g_min);FIELD(armijoFactor);FIELD(dt);FIELD(useFeedbackPolicy);FIELD(createValueFunction);FIELD(inequalityConstraintMu);FIELD(inequalityConstraintDelta);FIELD(projectStateInputEqualityConstraints);FIELD(extractProjectionMultiplier);FIELD(nThreads);FIELD(threadPriority);
 #undef FIELD
 out.put("loaded.sqp.integrator",sensitivity_integrator::toString(s.integratorType));
 boost::property_tree::ptree names;for(auto&n:i.modelSettings().mpcModelJointNames){boost::property_tree::ptree item;item.put("",n);names.push_back({"",item});}out.add_child("joint_names_mpc_order",names);
 auto&ocp=i.getOptimalControlProblem();
 auto inventory=[&](const char*name,auto&collection){boost::property_tree::ptree rows;for(auto&[n,index]:collection->getTermNameMap()){boost::property_tree::ptree row;row.put("name",n);row.put("index",index);rows.push_back({"",row});}out.add_child(name,rows);};
 inventory("running_cost",ocp.costPtr);inventory("terminal_cost",ocp.finalCostPtr);inventory("state_soft",ocp.stateSoftConstraintPtr);inventory("input_soft",ocp.softConstraintPtr);inventory("hard_equality",ocp.equalityConstraintPtr);
 boost::property_tree::write_json(folder+"/loaded_configuration.json",out);
}
class StructuralObserver final:public SolverSynchronizedModule,public DummyObserver {
 WBMpcInterface&interface_;SqpMpc&mpc_;std::shared_ptr<TimeLog>log_;
 OptimalControlProblem clone_;PinocchioInterface pin_;WBAccelMpcRobotModel<double>model_;
 std::string root_;vector_t lastDerivative_=vector_t::Zero(58);double lastSampleTime_=0;std::array<double,8>lastSolver_{};
 std::ofstream solver_,dynamics_,costs_,constraints_,id_;double target_=0;size_t previousIterations_=0;
 void scalarCost(const char*group,const std::string&name,double t,double value){if(!std::isfinite(value))throw std::runtime_error("Nonfinite read-only cost");costs_<<target_<<','<<t<<','<<group<<','<<name<<','<<value<<'\n';}
 public:
 StructuralObserver(WBMpcInterface&i,SqpMpc&m,std::shared_ptr<TimeLog>log,const std::string&root):interface_(i),mpc_(m),log_(log),root_(root),clone_(i.getOptimalControlProblem()),pin_(i.getPinocchioInterface()),model_(i.modelSettings()),solver_(root+"/structural_solver.csv"),dynamics_(root+"/structural_dynamics.csv"),costs_(root+"/structural_costs.csv"),constraints_(root+"/structural_constraints.csv"),id_(root+"/structural_id.csv"){
  solver_<<"control_tick,observation_time,iterations_this_solve,total_iterations,objective,dynamics_sse,equality_sse,inequality_sse,lq_wall_ms,qp_wall_ms,linesearch_wall_ms,controller_wall_ms,status\n";
  dynamics_<<"solver_time,sample_time,index,derivative,A_rows,A_cols,B_rows,B_cols\n";
  costs_<<"solver_time,sample_time,group,term,value\n";
  constraints_<<"solver_time,sample_time,term,active,dimension,index,residual\n";
  id_<<"observation_time,query_time,index,q_des,qd_des,qdd_des,tau_ff,left_wrench,right_wrench\n";
  for(auto*f:{&solver_,&dynamics_,&costs_,&constraints_,&id_})*f<<std::setprecision(17);
 }
 void preSolverRun(double t,double,const vector_t&,const ReferenceManagerInterface&)override{target_=t;}
 void postSolverRun(const PrimalSolution&p)override{
  auto*solver=mpc_.getSolverPtr();auto perf=solver->getPerformanceIndeces();auto n=solver->getNumIterations();auto times=dynamic_cast<SqpSolver*>(solver)->getBenchmarks();
  // Public API exposes performance and iteration count; convergence enum is private.
  solver_<<std::llround(target_/.002)<<','<<target_<<','<<n-previousIterations_<<','<<n<<','<<perf.cost<<','<<perf.dynamicsViolationSSE<<','<<perf.equalityConstraintsSSE<<','<<perf.inequalityConstraintsSSE<<','<<times.linearQuadraticApproximationTime<<','<<times.solveQpTime<<','<<times.linesearchTime<<','<<times.computeControllerTime<<",POLICY_RETURNED_CONVERGENCE_ENUM_UNAVAILABLE\n"<<std::flush;previousIterations_=n;lastSolver_={double(n),perf.cost,perf.dynamicsViolationSSE,perf.equalityConstraintsSSE,times.linearQuadraticApproximationTime,times.solveQpTime,times.linesearchTime,times.computeControllerTime};
  clone_.targetTrajectoriesPtr=&interface_.getSwitchedModelReferenceManagerPtr()->getTargetTrajectories();
  // One interior policy knot per solve. Objects and precomputation are cloned,
  // sharing only the unchanged reference manager after solver completion.
  size_t k=std::min(p.timeTrajectory_.size()/2,p.inputTrajectory_.size()-1);double t=p.timeTrajectory_[k];auto&x=p.stateTrajectory_[k];auto&u=p.inputTrajectory_[k];auto&pc=*clone_.preComputationPtr;
  pc.request(Request::Cost+Request::Constraint+Request::SoftConstraint,t,x,u);
  auto lin=clone_.dynamicsPtr->linearApproximation(t,x,u,pc);lastDerivative_=lin.f;lastSampleTime_=t;
  for(int j=0;j<lin.f.size();++j)dynamics_<<target_<<','<<t<<','<<j<<','<<lin.f[j]<<','<<lin.dfdx.rows()<<','<<lin.dfdx.cols()<<','<<lin.dfdu.rows()<<','<<lin.dfdu.cols()<<'\n';
  double total=0;
  auto inputCosts=[&](const char*group,auto&collection){for(auto&[name,index]:collection->getTermNameMap()){auto&term=collection->get(name);double v=term.isActive(t)?term.getValue(t,x,u,*clone_.targetTrajectoriesPtr,pc):0;scalarCost(group,name,t,v);total+=v;}};
  inputCosts("running",clone_.costPtr);inputCosts("input_soft",clone_.softConstraintPtr);
  for(auto&[name,index]:clone_.stateSoftConstraintPtr->getTermNameMap()){auto&term=clone_.stateSoftConstraintPtr->get(name);double v=term.isActive(t)?term.getValue(t,x,*clone_.targetTrajectoriesPtr,pc):0;scalarCost("state_soft",name,t,v);total+=v;}
  scalarCost("running_total","sum_running_and_soft",t,total);
  for(auto&[name,index]:clone_.equalityConstraintPtr->getTermNameMap()){auto&term=clone_.equalityConstraintPtr->get(name);bool active=term.isActive(t);auto dim=active?term.getNumConstraints(t):0;if(active){auto r=term.getValue(t,x,u,pc);for(int j=0;j<r.size();++j)constraints_<<target_<<','<<t<<','<<name<<",1,"<<dim<<','<<j<<','<<r[j]<<'\n';}else constraints_<<target_<<','<<t<<','<<name<<",0,0,-1,0\n";}
  for(auto&[name,index]:clone_.softConstraintPtr->getTermNameMap()){auto&wrapper=clone_.softConstraintPtr->get<StateInputSoftConstraint>(name);auto&term=wrapper.get();bool active=term.isActive(t);if(active){auto r=term.getValue(t,x,u,pc);for(int j=0;j<r.size();++j)constraints_<<target_<<','<<t<<','<<name<<",1,"<<r.size()<<','<<j<<','<<r[j]<<'\n';}else constraints_<<target_<<','<<t<<','<<name<<",0,0,-1,0\n";}
  // Terminal contribution is separately evaluated, never added to instantaneous running total.
  double tf=p.timeTrajectory_.back();auto&xf=p.stateTrajectory_.back();pc.requestFinal(Request::Cost,tf,xf);
  for(auto&[name,index]:clone_.finalCostPtr->getTermNameMap())scalarCost("terminal",name,tf,clone_.finalCostPtr->get(name).getValue(tf,xf,*clone_.targetTrajectoriesPtr,pc));
  dynamics_<<std::flush;costs_<<std::flush;constraints_<<std::flush;
 }
 void update(const SystemObservation&o,const PrimalSolution&p,const CommandData&c)override{
  log_->update(o,p,c);double t=o.time+.005;auto x=LinearInterpolation::interpolate(t,p.timeTrajectory_,p.stateTrajectory_);auto u=LinearInterpolation::interpolate(t,p.timeTrajectory_,p.inputTrajectory_);
  auto tau=computeJointTorques<double>(x,u,pin_,model_);auto q=model_.getJointAngles(x);auto qd=model_.getJointVelocities(x,u);auto qdd=model_.getJointAccelerations(u);
  for(int j=0;j<23;++j)id_<<o.time<<','<<t<<','<<j<<','<<q[j]<<','<<qd[j]<<','<<qdd[j]<<','<<tau[j]<<','<<(j<6?u[j]:0)<<','<<(j<6?u[j+6]:0)<<'\n';id_<<std::flush;
  auto accel=computeGeneralizedAccelerations<double>(x,u,pin_,model_);std::vector<double>snapshot{1,o.time,t,p.timeTrajectory_.front(),p.timeTrajectory_.back(),double(p.modeSchedule_.modeAtTime(t))};
  for(auto*v:{&x,&u,&tau,&accel})for(int j=0;j<v->size();++j)snapshot.push_back((*v)[j]);for(auto v:lastSolver_)snapshot.push_back(v);snapshot.push_back(lastSampleTime_);for(int j=0;j<58;++j)snapshot.push_back(lastDerivative_[j]);
  if(snapshot.size()!=218)throw std::runtime_error("Structural snapshot shape");
  {std::ofstream out(root_+"/structural_snapshot.tmp",std::ios::binary);out.write(reinterpret_cast<const char*>(snapshot.data()),snapshot.size()*sizeof(double));}
  std::filesystem::rename(root_+"/structural_snapshot.tmp",root_+"/structural_snapshot.bin");
 }
};
}
