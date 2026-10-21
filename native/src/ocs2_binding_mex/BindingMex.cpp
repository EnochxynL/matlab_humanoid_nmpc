// Direct native object binding. MATLAB owns the clock, stages and packet protocol.
#include "mex.h"
#include <humanoid_wb_mpc/WBMpcInterface.h>
#ifdef G1_HAS_CENTROIDAL
#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#endif
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>
#include "controller/diagnostics/Diagnostics.h"
#include "EncodeDiagnostics.h"
#ifdef G1_HAS_CENTROIDAL
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#endif
#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
using ocs2::vector_t;
// Only explicit payload materializations in THIS binding are counted. Eigen
// maps borrow MATLAB memory; OCS2's internal copies are audited separately.
struct CopyCount { double operations=0, elements=0; void add(size_t n) { ++operations; elements+=n; } };
struct Problem {
  std::unique_ptr<ocs2::RobotInterface> interface;
  std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> reference;
  ocs2::mpc::Settings mpcSettings;
  ocs2::sqp::Settings sqpSettings;
  size_t nx=0, nu=0;
  size_t joints=0;
  std::array<double,8> swing;
  std::function<vector_t(const double*)> convert;
  std::function<void(Problem&)> enableDiagnostics;
  std::unique_ptr<g1_controller::diagnostics::Diagnostics> diagnostics;
  ocs2::SystemObservation diagnosticObservation;
  bool observationSet=false, targetSet=false, scheduleSet=false, failed=false;
  CopyCount targets, schedules, observations, queryStates, policyOutputs;
  // Reverse destruction order keeps every borrowed native dependency alive.
  std::unique_ptr<ocs2::SqpMpc> mpc;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt;
  template<class Interface> explicit Problem(std::unique_ptr<Interface> p,const std::string& task)
      : reference(p->getSwitchedModelReferenceManagerPtr()),
        mpcSettings(p->mpcSettings()), sqpSettings(p->sqpSettings()),
        nx(p->getMpcRobotModel().getStateDim()), nu(p->getMpcRobotModel().getInputDim()) {
    joints=p->getMpcRobotModel().getJointDim();
    const auto settings=ocs2::humanoid::loadSwingTrajectorySettings(task,"swing_trajectory_config",false);
    swing={settings.liftOffVelocity,settings.touchDownVelocity,settings.swingHeight,
      settings.touchDownHeightOffset,settings.swingTimeScale,settings.impactProximityFactorLiftOffVelocity,
      settings.impactProximityFactorTouchDownVelocity,settings.impactProximityFactorMidPointValue};
    auto* definition=p.get();
    enableDiagnostics=[definition,task](Problem& owner) {
      g1_controller::ControllerConfig config;config.taskFile=task;
      owner.diagnostics=std::make_unique<g1_controller::diagnostics::Diagnostics>(config,*definition,*owner.mpc);
    };
    convert=[definition](const double* data)->vector_t {
      const size_t nq=6+definition->getMpcRobotModel().getJointDim();
#ifdef G1_HAS_CENTROIDAL
      if constexpr(std::is_same_v<Interface,ocs2::humanoid::CentroidalMpcInterface>) {
        auto& pin=definition->getPinocchioInterface();const auto& info=definition->getCentroidalModelInfo();
        const vector_t q=Eigen::Map<const vector_t>(data,nq),v=Eigen::Map<const vector_t>(data+nq,nq);
        ocs2::updateCentroidalDynamics(pin,info,q);
        vector_t result(6+nq);result.head(6)=ocs2::getCentroidalMomentumMatrix(pin)*v/info.robotMass;
        result.tail(nq)=q;return result;
      } else
#endif
      return Eigen::Map<const vector_t>(data,2*nq);
    };
    interface=std::move(p);
  }
};
std::map<uint64_t,std::unique_ptr<Problem>> problems;
uint64_t nextHandle=static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
void cleanup() { problems.clear(); }
void require(bool condition,const char* message) { if(!condition) throw std::runtime_error(message); }
std::string text(const mxArray* a) {
  require(mxIsChar(a),"Expected character vector");
  char* s=mxArrayToString(a);require(s,"Cannot read string");
  std::string result(s);mxFree(s);return result;
}
const double* numbers(const mxArray* a,size_t count) {
  require(mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a)&&mxGetNumberOfElements(a)==count,
          "Expected real dense double array of matching size");
  const double* p=mxGetPr(a);
  for(size_t k=0;k<count;++k) require(std::isfinite(p[k]),"Nonfinite input");
  return p;
}
double scalar(const mxArray* a) { return numbers(a,1)[0]; }
Problem& problem(const mxArray* a) {
  require(mxIsUint64(a)&&mxGetNumberOfElements(a)==1,"Expected uint64 object handle");
  auto it=problems.find(*static_cast<const uint64_t*>(mxGetData(a)));
  require(it!=problems.end(),"Unknown or destroyed object handle");return *it->second;
}
ocs2::MPC_MRT_Interface& solver(Problem& p) {
  require(bool(p.mrt),"Create solver first");return *p.mrt;
}
ocs2::TargetTrajectories target(Problem& p,const mxArray* times,const mxArray* states,const mxArray* inputs) {
  const size_t n=mxGetNumberOfElements(times);require(n>0,"Empty target trajectory");
  const double* t=numbers(times,n);const double* x=numbers(states,n*p.nx);const double* u=numbers(inputs,n*p.nu);
  require(mxGetM(states)==p.nx&&mxGetN(states)==n&&mxGetM(inputs)==p.nu&&mxGetN(inputs)==n,
          "Target matrices must be nx-by-knots and nu-by-knots");
  for(size_t k=1;k<n;++k)require(t[k]>t[k-1],"Target times must increase");
  ocs2::TargetTrajectories r;
  r.timeTrajectory.assign(t,t+n);p.targets.add(n);
  r.stateTrajectory.reserve(n);r.inputTrajectory.reserve(n);
  for(size_t k=0;k<n;++k) {
    r.stateTrajectory.emplace_back(Eigen::Map<const vector_t>(x+k*p.nx,p.nx));p.targets.add(p.nx);
    r.inputTrajectory.emplace_back(Eigen::Map<const vector_t>(u+k*p.nu,p.nu));p.targets.add(p.nu);
  }
  return r;
}
mxArray* output(const vector_t& v,CopyCount& count) {
  auto* a=mxCreateDoubleMatrix(v.size(),1,mxREAL);
  std::copy_n(v.data(),v.size(),mxGetPr(a));count.add(v.size());return a;
}
}

// Pinocchio headers introduce explicit DLL exports. MinGW then stops its
// implicit export-all behavior, so export the MEX gateway explicitly.
#ifdef _WIN32
__declspec(dllexport)
#endif
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]) {
  try {
    require(nrhs>=1,"Expected command");const auto command=text(prhs[0]);
    if(command=="create_problem") {
      require(nrhs==2&&nlhs==1&&mxIsStruct(prhs[1])&&mxGetNumberOfElements(prhs[1])==1,
              "create_problem(config) requires one output");
      const auto field=[&](const char* key) { auto* a=mxGetField(prhs[1],0,key);require(a,"Missing config field");return text(a); };
      const auto formulation=field("formulation"),task=field("taskFile"),urdf=field("urdfFile"),ref=field("referenceFile");
      std::unique_ptr<Problem> p;
      if(formulation=="whole_body")p=std::make_unique<Problem>(std::make_unique<ocs2::humanoid::WBMpcInterface>(task,urdf,ref),task);
#ifdef G1_HAS_CENTROIDAL
      else if(formulation=="centroidal")p=std::make_unique<Problem>(std::make_unique<ocs2::humanoid::CentroidalMpcInterface>(task,urdf,ref),task);
#endif
      else throw std::runtime_error("Unsupported formulation");
      static bool registered=false;if(!registered){mexAtExit(cleanup);registered=true;}
      plhs[0]=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);
      const auto handle=++nextHandle;problems.emplace(handle,std::move(p));
      *static_cast<uint64_t*>(mxGetData(plhs[0]))=handle;mexLock();return;
    }
    require(nrhs>=2,"Expected object handle");auto& p=problem(prhs[1]);
    if(command=="destroy") {
      require(nrhs==2&&nlhs==0,"destroy(handle)");
      problems.erase(*static_cast<const uint64_t*>(mxGetData(prhs[1])));mexUnlock();
    } else if(command=="dimensions") {
      require(nrhs==2&&nlhs==1,"dimensions(handle)");
      plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL);auto* a=mxGetPr(plhs[0]);
      a[0]=p.nx;a[1]=p.nu;a[2]=1./p.mpcSettings.mpcDesiredFrequency_;
    } else if(command=="create_solver") {
      require(nrhs==2&&nlhs==0&&!p.mpc,"create_solver requires an unsolved problem object");
      auto mpc=std::make_unique<ocs2::SqpMpc>(p.mpcSettings,p.sqpSettings,
          p.interface->getOptimalControlProblem(),p.interface->getInitializer());
      mpc->getSolverPtr()->setReferenceManager(p.reference);
      auto mrt=std::make_unique<ocs2::MPC_MRT_Interface>(*mpc);
      p.mpc=std::move(mpc);p.mrt=std::move(mrt);
    } else if(command=="model_settings") {
      require(nrhs==2&&nlhs==1,"model_settings(handle)");
      const char* fields[]={"joints","horizon","swing"};plhs[0]=mxCreateStructMatrix(1,1,3,fields);
      mxSetField(plhs[0],0,"joints",mxCreateDoubleScalar(p.joints));
      mxSetField(plhs[0],0,"horizon",mxCreateDoubleScalar(p.mpcSettings.timeHorizon_));
      auto* a=mxCreateDoubleMatrix(8,1,mxREAL);std::copy(p.swing.begin(),p.swing.end(),mxGetPr(a));mxSetField(plhs[0],0,"swing",a);
    } else if(command=="convert_observation") {
      require(nrhs==3&&nlhs==1,"convert_observation(handle,qv)");
      const auto result=p.convert(numbers(prhs[2],2*(6+p.joints)));
      require(result.allFinite(),"Nonfinite converted observation");
      CopyCount ignored;plhs[0]=output(result,ignored);
    } else if(command=="enable_diagnostics") {
      require(nrhs==2&&nlhs==0&&!p.diagnostics,"enable_diagnostics(handle)");solver(p);p.enableDiagnostics(p);
    } else if(command=="diagnostics") {
      require(nrhs==2&&nlhs==1,"diagnostics(handle)");
      plhs[0]=encodeDiagnostics(p.diagnostics?p.diagnostics->drain():g1_controller::DiagnosticBatch{});
    } else if(command=="diagnostic_tick") {
      require(nrhs==3&&nlhs==0,"diagnostic_tick(handle,tick)");
      const double tick=scalar(prhs[2]);require(tick>=0&&tick<=9007199254740992.&&tick==std::floor(tick),"Invalid diagnostic tick");
      if(p.diagnostics)p.diagnostics->observation(static_cast<uint64_t>(tick));
    } else if(command=="diagnostic_trace") {
      require(nrhs==4&&nlhs==0,"diagnostic_trace(handle,event,time)");
      const auto event=text(prhs[2]);const double time=scalar(prhs[3]);
      if(p.diagnostics)p.diagnostics->trace(event.c_str(),time);
    } else if(command=="diagnostic_reference") {
      require(nrhs==4&&nlhs==0,"diagnostic_reference(handle,time,command)");
      const double time=scalar(prhs[2]);const auto* values=numbers(prhs[3],4);
      if(p.diagnostics)p.diagnostics->reference(time,Eigen::Map<const ocs2::vector4_t>(values));
    } else if(command=="reference_snapshot") {
      require(nrhs==2&&nlhs==1,"reference_snapshot(handle)");
      const auto& r=p.reference->getTargetTrajectories();const auto& s=p.reference->getModeSchedule();
      const char* fields[]={"times","states","inputs","events","modes"};plhs[0]=mxCreateStructMatrix(1,1,5,fields);
      auto* times=mxCreateDoubleMatrix(r.timeTrajectory.size(),1,mxREAL);
      std::copy(r.timeTrajectory.begin(),r.timeTrajectory.end(),mxGetPr(times));mxSetField(plhs[0],0,"times",times);
      auto* states=mxCreateDoubleMatrix(p.nx,r.stateTrajectory.size(),mxREAL);
      for(size_t k=0;k<r.stateTrajectory.size();++k)std::copy_n(r.stateTrajectory[k].data(),p.nx,mxGetPr(states)+k*p.nx);
      mxSetField(plhs[0],0,"states",states);
      auto* inputs=mxCreateDoubleMatrix(p.nu,r.inputTrajectory.size(),mxREAL);
      for(size_t k=0;k<r.inputTrajectory.size();++k)std::copy_n(r.inputTrajectory[k].data(),p.nu,mxGetPr(inputs)+k*p.nu);
      mxSetField(plhs[0],0,"inputs",inputs);
      auto* events=mxCreateDoubleMatrix(s.eventTimes.size(),1,mxREAL);
      std::copy(s.eventTimes.begin(),s.eventTimes.end(),mxGetPr(events));mxSetField(plhs[0],0,"events",events);
      auto* modes=mxCreateDoubleMatrix(s.modeSequence.size(),1,mxREAL);
      std::copy(s.modeSequence.begin(),s.modeSequence.end(),mxGetPr(modes));mxSetField(plhs[0],0,"modes",modes);
    } else if(command=="disable_gait_extension") {
      require(nrhs==2&&nlhs==0,"disable_gait_extension(handle)");
      ocs2::humanoid::ModeSequenceTemplate empty({0.,1.},{3});
      empty.modeSequence.clear();empty.switchingTimes.clear();
      p.reference->getGaitSchedule()->insertModeSequenceTemplate(empty,0.,0.);
    } else if(command=="reset"||command=="set_target") {
      require(nrhs==5&&nlhs==0,"Expected handle,times,stateMatrix,inputMatrix");
      auto r=target(p,prhs[2],prhs[3],prhs[4]);
      if(command=="reset")solver(p).resetMpcNode(r);
      else { p.reference->setTargetTrajectories(std::move(r));p.targetSet=true; }
    } else if(command=="set_mode_schedule") {
      require(nrhs==4&&nlhs==0,"set_mode_schedule(handle,eventTimes,modes)");
      const size_t n=mxGetNumberOfElements(prhs[2]);
      require(n>=2,"Humanoid schedule requires real events and a terminal marker");
      const auto* e=numbers(prhs[2],n);const auto* m=numbers(prhs[3],n+1);
      require(std::is_sorted(e,e+n),"Events must be ordered");
      ocs2::ModeSchedule schedule;schedule.eventTimes.assign(e,e+n);p.schedules.add(n);
      schedule.modeSequence.resize(n+1);
      for(size_t k=0;k<=n;++k){require(m[k]>=0&&m[k]<=3&&m[k]==std::floor(m[k]),"Unknown humanoid contact mode");schedule.modeSequence[k]=size_t(m[k]);}
      p.schedules.add(n+1);
      p.reference->getGaitSchedule()->updateModeSchedule(schedule);
      p.scheduleSet=true;
    } else if(command=="set_observation") {
      require(nrhs==6&&nlhs==0,"set_observation(handle,time,state,input,mode)");
      ocs2::SystemObservation observation;observation.time=scalar(prhs[2]);
      observation.state=Eigen::Map<const vector_t>(numbers(prhs[3],p.nx),p.nx);p.observations.add(p.nx);
      observation.input=Eigen::Map<const vector_t>(numbers(prhs[4],p.nu),p.nu);p.observations.add(p.nu);
      const double mode=scalar(prhs[5]);require(mode>=0&&mode<=3&&mode==std::floor(mode),"Unknown contact mode");
      observation.mode=size_t(mode);p.reference->setMeasuredMode(observation.mode);
      solver(p).setCurrentObservation(observation);
      if(p.diagnostics)p.diagnosticObservation=observation;
      p.observationSet=true;
    } else if(command=="advance_mpc") {
      require(nrhs==2&&nlhs==0,"advance_mpc(handle)");
      require(!p.failed,"Previous solve failed; destroy and recreate this object");
      require(p.observationSet&&p.targetSet&&p.scheduleSet,"Set observation, target and fresh padded schedule before solving");
      p.scheduleSet=false;
      try { solver(p).advanceMpc(); } catch(...) { p.failed=true;throw; }
    } else if(command=="update_policy") {
      require(nrhs==2&&nlhs==1,"update_policy(handle) requires one output");
      plhs[0]=mxCreateLogicalScalar(solver(p).updatePolicy());
    } else if(command=="evaluate_policy") {
      require(nrhs==4&&nlhs==3,"[state,input,mode]=evaluate_policy(handle,time,state)");
      auto& mrt=solver(p);require(mrt.initialPolicyReceived(),"No policy installed");
      const double t=scalar(prhs[2]);
      const vector_t state=Eigen::Map<const vector_t>(numbers(prhs[3],p.nx),p.nx);p.queryStates.add(p.nx);
      vector_t x,u;size_t mode;mrt.evaluatePolicy(t,state,x,u,mode);
      require(x.size()==p.nx&&u.size()==p.nu&&x.allFinite()&&u.allFinite(),"Invalid policy output");
      plhs[0]=output(x,p.policyOutputs);plhs[1]=output(u,p.policyOutputs);plhs[2]=mxCreateDoubleScalar(double(mode));
      if(p.diagnostics)p.diagnostics->policy(p.diagnosticObservation,mrt.getPolicy(),mrt.getCommand());
    } else if(command=="copy_stats") {
      require(nrhs==2&&nlhs==1,"copy_stats(handle)");
      const char* fields[]={"targets","schedules","observations","queryStates","policyOutputs"};
      const CopyCount* counts[]={&p.targets,&p.schedules,&p.observations,&p.queryStates,&p.policyOutputs};
      plhs[0]=mxCreateStructMatrix(1,1,5,fields);
      for(size_t k=0;k<5;++k){auto* a=mxCreateDoubleMatrix(1,2,mxREAL);mxGetPr(a)[0]=counts[k]->operations;mxGetPr(a)[1]=counts[k]->elements;mxSetField(plhs[0],0,fields[k],a);}
    } else throw std::runtime_error("Unknown binding command");
  } catch(const std::exception& e) { mexErrMsgIdAndTxt("OCS2:Binding", "%s",e.what()); }
}
