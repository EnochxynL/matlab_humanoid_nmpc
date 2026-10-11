#include "mex.h"
#include "mex/PlannerContext.h"
#include <robot_model/RobotState.h>
#include <filesystem>
#include <memory>
#include <cstdint>

namespace {
std::unique_ptr<PlannerContext> context;
std::unique_ptr<robot::model::RobotDescription> description;
std::unique_ptr<robot::model::RobotState> state;
std::unique_ptr<WaveBControlTimeWBMpcController> controller;
std::vector<size_t> ids;
uint64_t lastTick=UINT64_MAX, sequence=0;
vector_t lastResult=vector_t::Zero(118);
vector_t lastPolicy=vector_t::Zero(95);
bool actionReady=false;
double lastRawTime=0;
std::ofstream sampleLog;
std::unique_ptr<CanonicalControlTime> canonicalLog;
std::string stringArg(const mxArray* a){if(!mxIsChar(a))throw std::runtime_error("Expected character vector");char*p=mxArrayToString(a);std::string s(p);mxFree(p);return s;}
const double* doubles(const mxArray*a,size_t n){if(!mxIsDouble(a)||mxIsComplex(a)||mxGetNumberOfElements(a)!=n)throw std::runtime_error("Invalid numeric argument dimensions");auto*p=mxGetPr(a);for(size_t j=0;j<n;++j)if(!std::isfinite(p[j]))throw std::runtime_error("Nonfinite argument");return p;}
void close(){sampleLog.close();canonicalLog.reset();controller.reset();state.reset();description.reset();context.reset();ids.clear();lastTick=UINT64_MAX;sequence=0;actionReady=false;}
void atExit(){close();}
void init(const std::string& root,const std::string& folder){
 if(context)throw std::runtime_error("Close existing MEX controller before initialization");
 std::filesystem::create_directories(folder);
 std::vector<std::string>a={"mex","simscape",root+"/config/task.info",root+"/config/reference.info",root+"/assets/g1_29dof.urdf",root+"/config/gait.info","unused","4.8",folder,"0","0","0","unused"};
 std::vector<char*>args;for(auto&s:a)args.push_back(s.data());Inputs inputs(args.data());
 context=std::make_unique<PlannerContext>(inputs);context->expectedSession=1;
 canonicalLog=std::make_unique<CanonicalControlTime>(folder);
 sampleLog.open(folder+"/planner_samples.csv");sampleLog<<std::setprecision(17)<<"control_tick,raw_time,canonical_time";for(int j=0;j<118;++j)sampleLog<<",value"<<j;sampleLog<<"\n";
 description=std::make_unique<robot::model::RobotDescription>(inputs.urdf);
 state=std::make_unique<robot::model::RobotState>(*description);
 auto names=description->getJointNames();std::sort(names.begin(),names.end());ids=description->getJointIndices(names);
 controller=std::make_unique<WaveBControlTimeWBMpcController>(*description,context->interface.modelSettings(),context->mpc,context->interface.getPinocchioInterface(),context->interface.mpcSettings().mpcDesiredFrequency_,context->structure);
 context->configure(*controller);mexLock();
}
void reference(const double*a){
 if(!context)throw std::runtime_error("Initialize MEX first");if(!a[1])return;
 if(sequence && context->currentPacket.tick==static_cast<uint64_t>(a[0]))return;
 task5a::Packet p;p.tick=static_cast<uint64_t>(a[0]);p.session=1;p.sequence=++sequence;
 size_t n=static_cast<size_t>(a[2]),m=static_cast<size_t>(a[285]);if((n!=1&&n!=3)||m<1||m>64)throw std::runtime_error("Reference shape rejected");
 for(size_t k=0;k<n;++k){p.target.timeTrajectory.push_back(a[3+k]);p.target.stateTrajectory.emplace_back(Eigen::Map<const vector_t>(a+6+58*k,58));p.target.inputTrajectory.emplace_back(Eigen::Map<const vector_t>(a+180+35*k,35));}
 for(size_t k=0;k<m;++k)p.schedule.eventTimes.push_back(a[286+k]);
 for(size_t k=0;k<=m;++k)p.schedule.modeSequence.push_back(static_cast<size_t>(a[350+k]));
 p.terminal=a[415];for(int k=0;k<8;++k)p.swing[k]=a[416+k];for(int k=0;k<4;++k)p.filtered[k]=a[424+k];context->currentPacket=std::move(p);
}
void query(const double*a){
 if(!controller)throw std::runtime_error("Initialize MEX first");uint64_t tick=static_cast<uint64_t>(std::llround(a[0]/.002));
 if(a[3]<.35 || a[3]>1.3)throw std::runtime_error("Base height outside physical safety envelope");
 double pitch=std::asin(std::clamp(2*(a[4]*a[6]-a[7]*a[5]),-1.,1.));
 double roll=std::atan2(2*(a[4]*a[5]+a[6]*a[7]),1-2*(a[5]*a[5]+a[6]*a[6]));
 if(std::abs(pitch)>.7853981633974483 || std::abs(roll)>.7853981633974483)throw std::runtime_error("Tilt outside physical safety envelope");
 if(lastTick==tick)return;if(lastTick!=UINT64_MAX&&!actionReady)throw std::runtime_error("Complete inverse dynamics before advancing policy tick");if(lastTick!=UINT64_MAX&&tick!=lastTick+1)throw std::runtime_error("Nonsequential physical tick");
 double t=canonicalLog->accept(tick,a[0],1,tick);
 state->setTime(t);state->setRootPositionInWorldFrame(robot::vector3_t(a+1));
 robot::quaternion_t q(a[4],a[5],a[6],a[7]);if(std::abs(q.norm()-1)>1e-6)throw std::runtime_error("Quaternion rejected");state->setRootRotationLocalToWorldFrame(q);
 state->setRootLinearVelocityInLocalFrame(robot::vector3_t(a+8));state->setRootAngularVelocityInLocalFrame(robot::vector3_t(a+11));
 for(size_t j=0;j<29;++j){state->setJointPosition(ids[j],a[14+j]);state->setJointVelocity(ids[j],a[43+j]);}
 // Original successful controller uses STANCE for measured observation mode.
 state->setContactFlag(0,true);state->setContactFlag(1,true);
 if(lastTick==UINT64_MAX)controller->startMpcThread(*state);
 vector_t x,u;size_t mode;controller->evaluatePolicy(*state,x,u,mode);
 lastPolicy<<x,u,double(mode),t+.005;lastTick=tick;lastRawTime=a[0];actionReady=false;
}
void inverseDynamics(const double* p){
 if(!controller||lastTick==UINT64_MAX)throw std::runtime_error("Query a policy before inverse dynamics");
 // Bind the action to exactly the policy queried in this tick; reject stale/mutated packets.
 if(!(Eigen::Map<const vector_t>(p,95).array()==lastPolicy.array()).all())
  throw std::runtime_error("Inverse dynamics policy identity mismatch");
 if(actionReady)return;
 vector_t x=lastPolicy.head(58),u=lastPolicy.segment(58,35);
 vector_t tau=controller->inverseDynamics(x,u);
 lastResult<<x,u,tau,lastPolicy[93],lastPolicy[94];actionReady=true;
 sampleLog<<lastTick<<','<<lastRawTime<<','<<double(lastTick)*.002;
 for(int j=0;j<118;++j)sampleLog<<','<<lastResult[j];sampleLog<<'\n'<<std::flush;
}
}
void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
 try{
  if(nrhs<1)throw std::runtime_error("Expected init/reference/step/close command");auto command=stringArg(prhs[0]);
  if(command=="init"){if(nrhs!=3||nlhs)throw std::runtime_error("init(root,folder)");static bool registered=false;if(!registered){mexAtExit(atExit);registered=true;}init(stringArg(prhs[1]),stringArg(prhs[2]));}
  else if(command=="close"){close();if(mexIsLocked())mexUnlock();}
  else if(command=="reference"){if(nrhs!=2)throw std::runtime_error("reference requires 428 values");reference(doubles(prhs[1],428));}
  else if(command=="step"){if(nrhs!=2||nlhs!=1)throw std::runtime_error("step requires 74 values and one output");query(doubles(prhs[1],74));inverseDynamics(lastPolicy.data());plhs[0]=mxCreateDoubleMatrix(118,1,mxREAL);std::copy(lastResult.data(),lastResult.data()+118,mxGetPr(plhs[0]));}
  else if(command=="query"){if(nrhs!=2||nlhs!=1)throw std::runtime_error("query requires 74 values and one output");query(doubles(prhs[1],74));plhs[0]=mxCreateDoubleMatrix(95,1,mxREAL);std::copy(lastPolicy.data(),lastPolicy.data()+95,mxGetPr(plhs[0]));}
  else if(command=="inverse_dynamics"){if(nrhs!=2||nlhs!=1)throw std::runtime_error("inverse_dynamics requires queried 95-value policy and one output");inverseDynamics(doubles(prhs[1],95));plhs[0]=mxCreateDoubleMatrix(23,1,mxREAL);std::copy(lastResult.data()+93,lastResult.data()+116,mxGetPr(plhs[0]));}
  else throw std::runtime_error("Unknown MEX command");
 }catch(const std::exception&e){mexErrMsgIdAndTxt("G1:PlannerMex","%s",e.what());}
}
