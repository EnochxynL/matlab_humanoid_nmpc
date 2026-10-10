#include "controller/WBMpcController.h"
#include "reference/ControlTimeMotionManager.h"
#include "diagnostics/TimeLog.h"
#include "reference/ExternalReferenceAdapter.h"
#include "diagnostics/StructuralObserver.h"
#include "math/GeneratedFootCost.h"
#include "math/GeneratedFootContracts.h"
#include "math/GeometryInstallBridge.h"
#include "math/GeneratedQuadraticCosts.h"
#include "time/CanonicalControlTime.h"

#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <humanoid_wb_mpc/command/WBMpcTargetTrajectoriesCalculator.h>
#include <ros2_robot_hw_interface/ros2_robot_hw_interface.hpp>
#include <algorithm>
#include <sstream>
#include <unordered_map>
using namespace ocs2;using namespace ocs2::humanoid;
struct Inputs {
 std::string backend,task,ref,urdf,gait,mjcf,root,recorded;
 double horizon,pacingMs,startup;bool heavy;
 Inputs(char**a):backend(a[1]),task(a[2]),ref(a[3]),urdf(a[4]),gait(a[5]),mjcf(a[6]),root(a[8]),recorded(a[12]),horizon(std::stod(a[7])),pacingMs(std::stod(a[9])),startup(std::stod(a[10])),heavy(std::stoi(a[11])){}
};
struct Context {
 WBMpcInterface interface;std::shared_ptr<task5c::Evidence> mathematicalEvidence,frictionEvidence,polynomialEvidence,relaxedEvidence,jointEvidence,terminalEvidence,runningEvidence,stateEvidence,nominalStateEvidence,nominalInputEvidence,preEvidence,stanceEvidence,swingEvidence,momentEvidence,collisionEvidence,footCostEvidence;OptimalControlProblem migratedProblem;task5c::GeneratedInitializer migratedInitializer;SqpMpc mpc;
 std::unique_ptr<WBMpcTargetTrajectoriesCalculator>calculator;
 std::shared_ptr<TimeLog>log;std::shared_ptr<ControlTimeMotionManager>motion;
 std::shared_ptr<task5a::ExternalReferenceAdapter>external;std::shared_ptr<task5b::StructuralObserver>structure;
 std::map<uint64_t,task5a::Packet>packets;bool migrated;uint64_t expectedSession=0;
 Context(const Inputs&i):interface(i.task,i.urdf,i.ref),mathematicalEvidence(std::make_shared<task5c::Evidence>(i.root)),frictionEvidence(std::make_shared<task5c::Evidence>(i.root,"friction_")),polynomialEvidence(std::make_shared<task5c::Evidence>(i.root,"polynomial_")),relaxedEvidence(std::make_shared<task5c::Evidence>(i.root,"relaxed_")),jointEvidence(std::make_shared<task5c::Evidence>(i.root,"joint_")),terminalEvidence(std::make_shared<task5c::Evidence>(i.root,"terminal_")),runningEvidence(std::make_shared<task5c::Evidence>(i.root,"running_")),stateEvidence(std::make_shared<task5c::Evidence>(i.root,"state_")),nominalStateEvidence(std::make_shared<task5c::Evidence>(i.root,"nominal_state_")),nominalInputEvidence(std::make_shared<task5c::Evidence>(i.root,"nominal_input_")),preEvidence(std::make_shared<task5c::Evidence>(i.root,"precomputation_")),stanceEvidence(std::make_shared<task5c::Evidence>(i.root,"stance_")),swingEvidence(std::make_shared<task5c::Evidence>(i.root,"swing_")),momentEvidence(std::make_shared<task5c::Evidence>(i.root,"moment_")),collisionEvidence(std::make_shared<task5c::Evidence>(i.root,"collision_")),footCostEvidence(std::make_shared<task5c::Evidence>(i.root,"foot_cost_")),migratedProblem(task5c::footcost::installFootCosts(task5c::footcost::installCollisionBridge(task5c::footcost::installMomentBridge(task5c::wavec::install(task5c::installQuadraticCosts(task5c::installJointLimits(task5c::installPenalties(task5c::installFriction(task5c::install(interface,mathematicalEvidence),interface,i.task,frictionEvidence),polynomialEvidence,relaxedEvidence),interface,jointEvidence),interface,terminalEvidence,runningEvidence,nominalStateEvidence,nominalInputEvidence),interface,preEvidence,stanceEvidence,swingEvidence),interface,i.task,momentEvidence),interface,i.task,collisionEvidence),interface,footCostEvidence)),migratedInitializer(interface.getInitializer(),interface,nominalInputEvidence),mpc(interface.mpcSettings(),interface.sqpSettings(),migratedProblem,migratedInitializer),log(std::make_shared<TimeLog>(i.root,i.heavy)),migrated(std::getenv("TASK5A_AUTHORITY")&&std::string(std::getenv("TASK5A_AUTHORITY"))=="SIMULINK_REFERENCE"){
  {std::ifstream maps("/proc/self/maps");std::ofstream(i.root+"/loaded_process_maps.txt")<<maps.rdbuf();}
  if(migrated){
   if(!std::getenv("TASK5A_PACKET_FILE"))throw std::runtime_error("No authoritative Simulink packet source");
   if(i.backend!="simscape")packets=task5a::loadPackets(std::getenv("TASK5A_PACKET_FILE"));
   external=std::make_shared<task5a::ExternalReferenceAdapter>(interface.getSwitchedModelReferenceManagerPtr(),[this](double t){uint64_t tick=std::llround(t/.002);auto p=packets.empty()?task5a::readBinaryPacket(std::getenv("TASK5A_PACKET_FILE")):packets.at(tick);
    if(expectedSession&&p.session!=expectedSession)throw std::runtime_error("G: Reference and PlantState sessions disagree");
    if(p.tick!=tick)throw std::runtime_error("G: Reference tick does not match actual solver observation");
    log->reference(t,p.filtered);return p;},i.root);
   mpc.getSolverPtr()->setReferenceManager(external);
  }else{
   calculator=std::make_unique<WBMpcTargetTrajectoriesCalculator>(i.ref,interface.getMpcRobotModel(),interface.mpcSettings().timeHorizon_);
   motion=std::make_shared<ControlTimeMotionManager>(i.gait,i.ref,interface.getSwitchedModelReferenceManagerPtr(),interface.getMpcRobotModel(),[this](const vector4_t&v,scalar_t t,scalar_t,const vector_t&x){log->reference(t,v);return calculator->commandedVelocityToTargetTrajectories(v,t,x);});
   mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());mpc.getSolverPtr()->addSynchronizedModule(motion);
  }
  mpc.getSolverPtr()->addSynchronizedModule(log);log->configure(interface.getPinocchioInterface(),interface.getMpcRobotModel(),interface.getSwitchedModelReferenceManagerPtr());
  task5b::exportConfiguration(interface,i.task,i.root); structure=std::make_shared<task5b::StructuralObserver>(interface,mpc,log,i.root);mpc.getSolverPtr()->addSynchronizedModule(structure);
  std::ofstream(i.root+"/reference_authority.txt")<<(migrated?"SIMULINK_REFERENCE":"LEGACY_CPP_REFERENCE")<<"\n";
 }
 void command(double t,bool step=false){if(migrated)return;WalkingVelocityCommand v;v.linear_velocity_x=std::getenv("TASK4A_STANDING")?0:(t>=.5?.1:0);motion->setAndScaleVelocityCommand(v);}
 void configure(WaveBControlTimeWBMpcController&c){c.setMathematicsEvidence(stateEvidence,nominalInputEvidence);c.enableDeterministicSimulation();c.setTraceCallback([this](const char*e,double t,double d,double r){log->trace(e,t,d,r);});}
};
class ActionLog {
 std::ofstream out_;const robot::model::RobotDescription&description_;std::vector<std::string>names_;
 public:ActionLog(const std::string&root,const robot::model::RobotDescription&d):out_(std::filesystem::path(root)/"actions.csv"),description_(d),names_(d.getJointNames()){
  std::sort(names_.begin(),names_.end());out_<<std::setprecision(17)<<"control_time_s";for(auto&n:names_)for(auto*f:{"q_des","qd_des","kp","kd","tau_ff"})out_<<','<<n<<'_'<<f;out_<<'\n';
 }
 void write(const robot::model::RobotState&s,const robot::model::RobotJointAction&a){out_<<s.getTime();for(auto&n:names_){auto&v=a.at(description_.getJointIndex(n)).value();for(double x:{v.q_des,v.qd_des,v.kp,v.kd,v.feed_forward_effort}){if(!std::isfinite(x))throw std::runtime_error("Nonfinite joint action");out_<<','<<x;}}out_<<'\n'<<std::flush;}
};
void pacing(double milliseconds){if(milliseconds>0)std::this_thread::sleep_for(std::chrono::duration<double,std::milli>(milliseconds));}
void summary(const Inputs&i,double start,double end){std::ofstream(std::filesystem::path(i.root)/"run_summary.json")<<std::setprecision(17)<<"{\"backend\":\""<<i.backend<<"\",\"mode\":\"SIMULATION_TIME_DETERMINISTIC\",\"requested_horizon_control_s\":"<<i.horizon<<",\"last_control_time_s\":"<<end<<",\"execution_wall_duration_s\":"<<wallTime()-start<<",\"startup_wall_delay_s\":"<<i.startup<<",\"pacing_wall_ms_per_control_tick\":"<<i.pacingMs<<",\"heavy_logging\":"<<(i.heavy?"true":"false")<<"}\n";}
void simscape(const Inputs&i,Context&c,int argc,char**argv){
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("g3d_time_plant");g1_bridge::Ros2RobotHWInterface hw(i.urdf);ActionLog actions(i.root,hw.getRobotDescription());
 std::ofstream rawActions(std::filesystem::path(i.root)/"controller_action.csv");
 rawActions<<std::setprecision(17)<<"session,state_sequence,action_sequence,joint_name,q_des,qd_des,kp,kd,tau_ff,mapping_error\n";
 WaveBControlTimeWBMpcController controller(hw.getRobotDescription(),c.interface.modelSettings(),c.mpc,c.interface.getPinocchioInterface(),c.interface.mpcSettings().mpcDesiredFrequency_,c.structure);c.configure(controller);
 CanonicalControlTime clock(i.root);uint64_t originSequence=0;uint64_t session=0,seq=0;bool started=false,failed=false;double start=wallTime(),last=0;
 const uint64_t endpoint=static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());auto qos=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
 auto pub=node->create_publisher<g1_bridge::Ros2RobotHWInterface::ActionMsg>("/g1_bridge/action",qos);
 auto obs=node->create_publisher<g1_bridge::Ros2RobotHWInterface::ObservationMsg>("/g1_bridge/observation",qos);
 auto sub=node->create_subscription<g1_bridge::Ros2RobotHWInterface::StateMsg>("/g1_bridge/state",qos,[&](g1_bridge::Ros2RobotHWInterface::StateMsg::ConstSharedPtr msg){
  try{
   if(failed)return;if(started&&msg->session_id!=session)throw std::runtime_error("Fresh process required for new epoch/session");std::string reason;if(!hw.consume(*msg,reason))throw std::runtime_error("Rejected physical state: "+reason);
   double receiveEpoch=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();c.log->trace("transport_observation_received",msg->sim_time,msg->sim_time,msg->sequence);if(!started)originSequence=msg->sequence;
   c.expectedSession=msg->session_id;auto state=hw.getRobotState();double canonical=clock.accept(msg->sequence-originSequence,msg->sim_time,msg->session_id,msg->sequence);state.setTime(canonical);c.command(canonical);
   if(!started){session=msg->session_id;start=wallTime();controller.startMpcThread(state);started=true;}
   controller.computeJointControlAction(canonical,state,hw.getRobotJointAction());actions.write(state,hw.getRobotJointAction());
   auto wire=hw.serializeAction(endpoint,++seq);
   for(size_t j=0;j<29;++j){
    const auto&a=hw.getRobotJointAction().at(hw.jointIds()[j]).value();
    const double mappingError=std::max({std::abs(wire.q_des[j]-a.q_des),std::abs(wire.qd_des[j]-a.qd_des),std::abs(wire.kp[j]-a.kp),std::abs(wire.kd[j]-a.kd),std::abs(wire.tau_ff[j]-a.feed_forward_effort)});
    if(mappingError!=0)throw std::runtime_error("Serialization changed controller action");
    rawActions<<session<<','<<msg->sequence<<','<<seq<<','<<g1_bridge::canonical_names[j]<<','<<a.q_des<<','<<a.qd_des<<','<<a.kp<<','<<a.kd<<','<<a.feed_forward_effort<<','<<mappingError<<'\n';
   }
   rawActions<<std::flush;pub->publish(wire);obs->publish(hw.observation(receiveEpoch,std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()));last=msg->sim_time;
  }catch(const std::exception&e){failed=true;std::ofstream(std::filesystem::path(i.root)/"failure.txt")<<e.what();RCLCPP_ERROR(node->get_logger(),"%s",e.what());}
 });
 std::cout<<"WAITING_FOR_VALID_PLANT_STATE"<<std::endl;rclcpp::spin(node);summary(i,start,last);rclcpp::shutdown();if(failed)throw std::runtime_error("Simscape runtime failed; see failure.txt");
}
int main(int argc,char**argv){
 if(argc!=13)throw std::runtime_error("runner simscape task ref urdf gait unused horizon output pacing_ms startup_s heavy unused");
 Inputs i(argv);if(i.backend!="simscape")throw std::runtime_error("This source extraction supports Simscape only");
 std::filesystem::create_directories(i.root);Context c(i);simscape(i,c,argc,argv);
}
