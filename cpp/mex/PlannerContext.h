#pragma once
#include "controller/WBMpcController.h"
#include "diagnostics/TimeLog.h"
#include "reference/ExternalReferenceAdapter.h"
#include "diagnostics/StructuralObserver.h"
#include "math/GeneratedFootCost.h"
#include "math/GeneratedFootContracts.h"
#include "math/GeometryInstallBridge.h"
#include "math/GeneratedQuadraticCosts.h"
#include "time/CanonicalControlTime.h"

#include <humanoid_wb_mpc/WBMpcInterface.h>

#include <algorithm>
#include <sstream>
#include <unordered_map>
using namespace ocs2;using namespace ocs2::humanoid;
struct Inputs {
 std::string backend,task,ref,urdf,gait,mjcf,root,recorded;
 double horizon,pacingMs,startup;bool heavy;
 Inputs(char**a):backend(a[1]),task(a[2]),ref(a[3]),urdf(a[4]),gait(a[5]),mjcf(a[6]),root(a[8]),recorded(a[12]),horizon(std::stod(a[7])),pacingMs(std::stod(a[9])),startup(std::stod(a[10])),heavy(std::stoi(a[11])){}
};
struct PlannerContext {
 WBMpcInterface interface;std::shared_ptr<task5c::Evidence> mathematicalEvidence,frictionEvidence,polynomialEvidence,relaxedEvidence,jointEvidence,terminalEvidence,runningEvidence,stateEvidence,nominalStateEvidence,nominalInputEvidence,preEvidence,stanceEvidence,swingEvidence,momentEvidence,collisionEvidence,footCostEvidence;OptimalControlProblem migratedProblem;task5c::GeneratedInitializer migratedInitializer;SqpMpc mpc;
 std::shared_ptr<TimeLog>log;
 std::shared_ptr<task5a::ExternalReferenceAdapter>external;std::shared_ptr<task5b::StructuralObserver>structure;
 task5a::Packet currentPacket; std::map<uint64_t,task5a::Packet>packets;bool migrated;uint64_t expectedSession=0;
 PlannerContext(const Inputs&i):interface(i.task,i.urdf,i.ref),mathematicalEvidence(std::make_shared<task5c::Evidence>(i.root)),frictionEvidence(std::make_shared<task5c::Evidence>(i.root,"friction_")),polynomialEvidence(std::make_shared<task5c::Evidence>(i.root,"polynomial_")),relaxedEvidence(std::make_shared<task5c::Evidence>(i.root,"relaxed_")),jointEvidence(std::make_shared<task5c::Evidence>(i.root,"joint_")),terminalEvidence(std::make_shared<task5c::Evidence>(i.root,"terminal_")),runningEvidence(std::make_shared<task5c::Evidence>(i.root,"running_")),stateEvidence(std::make_shared<task5c::Evidence>(i.root,"state_")),nominalStateEvidence(std::make_shared<task5c::Evidence>(i.root,"nominal_state_")),nominalInputEvidence(std::make_shared<task5c::Evidence>(i.root,"nominal_input_")),preEvidence(std::make_shared<task5c::Evidence>(i.root,"precomputation_")),stanceEvidence(std::make_shared<task5c::Evidence>(i.root,"stance_")),swingEvidence(std::make_shared<task5c::Evidence>(i.root,"swing_")),momentEvidence(std::make_shared<task5c::Evidence>(i.root,"moment_")),collisionEvidence(std::make_shared<task5c::Evidence>(i.root,"collision_")),footCostEvidence(std::make_shared<task5c::Evidence>(i.root,"foot_cost_")),migratedProblem(task5c::footcost::installFootCosts(task5c::footcost::installCollisionBridge(task5c::footcost::installMomentBridge(task5c::wavec::install(task5c::installQuadraticCosts(task5c::installJointLimits(task5c::installPenalties(task5c::installFriction(task5c::install(interface,mathematicalEvidence),interface,i.task,frictionEvidence),polynomialEvidence,relaxedEvidence),interface,jointEvidence),interface,terminalEvidence,runningEvidence,nominalStateEvidence,nominalInputEvidence),interface,preEvidence,stanceEvidence,swingEvidence),interface,i.task,momentEvidence),interface,i.task,collisionEvidence),interface,footCostEvidence)),migratedInitializer(interface.getInitializer(),interface,nominalInputEvidence),mpc(interface.mpcSettings(),interface.sqpSettings(),migratedProblem,migratedInitializer),log(std::make_shared<TimeLog>(i.root,i.heavy)),migrated(std::getenv("TASK5A_AUTHORITY")&&std::string(std::getenv("TASK5A_AUTHORITY"))=="SIMULINK_REFERENCE"){
  {std::ifstream maps("/proc/self/maps");std::ofstream(i.root+"/loaded_process_maps.txt")<<maps.rdbuf();}
  if(migrated){
   external=std::make_shared<task5a::ExternalReferenceAdapter>(interface.getSwitchedModelReferenceManagerPtr(),[this](double t){uint64_t tick=std::llround(t/.002);auto p=currentPacket;
    if(expectedSession&&p.session!=expectedSession)throw std::runtime_error("G: Reference and PlantState sessions disagree");
    if(p.tick!=tick)throw std::runtime_error("G: Reference tick does not match actual solver observation");
    log->reference(t,p.filtered);return p;},i.root);
   mpc.getSolverPtr()->setReferenceManager(external);
  }else throw std::runtime_error("MEX requires SIMULINK_REFERENCE authority");
  mpc.getSolverPtr()->addSynchronizedModule(log);log->configure(interface.getPinocchioInterface(),interface.getMpcRobotModel(),interface.getSwitchedModelReferenceManagerPtr());
  task5b::exportConfiguration(interface,i.task,i.root); structure=std::make_shared<task5b::StructuralObserver>(interface,mpc,log,i.root);mpc.getSolverPtr()->addSynchronizedModule(structure);
  std::ofstream(i.root+"/reference_authority.txt")<<(migrated?"SIMULINK_REFERENCE":"LEGACY_CPP_REFERENCE")<<"\n";
 }
 void configure(WaveBControlTimeWBMpcController&c){c.setMathematicsEvidence(stateEvidence,nominalInputEvidence);c.enableDeterministicSimulation();c.setTraceCallback([this](const char*e,double t,double d,double r){log->trace(e,t,d,r);});}
};
