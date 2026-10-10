#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "math/GeneratedConstraint.h"
#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <humanoid_wb_mpc/WBMpcPreComputation.h>
#include <humanoid_wb_mpc/end_effector/PinocchioEndEffectorDynamicsCppAd.h>
#include <humanoid_wb_mpc/constraint/EndEffectorDynamicsAccelerationsConstraint.h>
#include <humanoid_wb_mpc/constraint/EndEffectorDynamicsLinearAccConstraint.h>
#include <ocs2_core/misc/Numerics.h>
#include "precomputationPlan.h"
#include "swingConfiguration.h"
#include "footLinearResidual.h"
namespace task5c::wavec {
using namespace ocs2;using namespace ocs2::humanoid;
inline void same(const matrix_t&a,const matrix_t&b,const char* what,double tol=0.) {
 if(a.rows()!=b.rows()||a.cols()!=b.cols())throw std::runtime_error(std::string(what)+" dimensions");
 for(Eigen::Index n=0;n<a.size();++n)if(!std::isfinite(a.data()[n])||!std::isfinite(b.data()[n])||std::abs(a.data()[n]-b.data()[n])>tol+(tol>0?2e-14:0.)*std::abs(b.data()[n]))throw std::runtime_error(std::string(what)+" mismatch");
}
using SwingConfig=EndEffectorDynamicsLinearAccConstraint::Config;
class GeneratedPreComputation final:public HumanoidPreComputation {
 std::unique_ptr<WBMpcPreComputation> original_;std::vector<SwingConfig> configs_;
 Authority authority_;std::shared_ptr<Evidence> evidence_;
 public:
 GeneratedPreComputation(WBMpcInterface&i,Authority a,std::shared_ptr<Evidence>e):HumanoidPreComputation(i.getPinocchioInterface(),*i.getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner(),i.getMpcRobotModel()),original_(new WBMpcPreComputation(i.getPinocchioInterface(),*i.getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner(),i.getMpcRobotModel())),configs_(2),authority_(a),evidence_(std::move(e)){}
 GeneratedPreComputation(const GeneratedPreComputation&r):HumanoidPreComputation(r),original_(r.original_->clone()),configs_(r.configs_),authority_(r.authority_),evidence_(r.evidence_){++evidence_->clones;}
 GeneratedPreComputation* clone()const override{return new GeneratedPreComputation(*this);}
 const std::vector<SwingConfig>& accelerationConfigs()const{return configs_;}
 void request(RequestSet req,double t,const vector_t&x,const vector_t&u)override {
  ++evidence_->value;evidence_->stack();
  if(authority_!=Authority::Active){++evidence_->legacyEvaluations;original_->request(req,t,x,u);}
  if(authority_==Authority::Legacy){pinocchioInterface_=original_->getPinocchioInterface();configs_=original_->getEeNormalAccelerationConstraintConfigs();for(int k=0;k<2;++k)R_world_to_contacts_[k]=original_->getRWorldToContacts(k);return;}
  boolean_T requests[5]={req.contains(Request::Dynamics),req.contains(Request::Cost),req.contains(Request::Constraint),req.contains(Request::SoftConstraint),req.contains(Request::Approximation)},plan[2];
  precomputationPlan(requests,plan);++evidence_->generated;
  if(plan[0])updatePinocchioModelKinematics(mpcRobotModelPtr_->getGeneralizedCoordinates(x)); // strict original U12 dependency
  if(plan[1])for(int k=0;k<2;++k){
   auto& g=mpcRobotModelPtr_->modelSettings.footConstraintConfig;
   boolean_T enabled=!numerics::almost_eq(g.positionErrorGain_z,0.);
   double z[3]={enabled?swingTrajectoryPlannerPtr_->getZpositionConstraint(k,t):0.,swingTrajectoryPlannerPtr_->getZvelocityConstraint(k,t),swingTrajectoryPlannerPtr_->getZaccelerationConstraint(k,t)};
   double gains[3]={g.positionErrorGain_z,g.linearVelocityErrorGain_z,g.linearAccelerationErrorGain_z};
   double b,Ax[3],Av[3],Aa[3];swingConfiguration(z,gains,enabled,&b,Ax,Av,Aa);++evidence_->generated;
   SwingConfig cfg;cfg.b=vector_t::Constant(1,b);cfg.Av=Eigen::Map<Eigen::Matrix<double,1,3>>(Av);cfg.Aa=Eigen::Map<Eigen::Matrix<double,1,3>>(Aa);if(enabled)cfg.Ax=Eigen::Map<Eigen::Matrix<double,1,3>>(Ax);configs_[k]=cfg;
   auto frame=pinocchioInterface_.getModel().getFrameId(mpcRobotModelPtr_->modelSettings.contactNames6DoF[k]);R_world_to_contacts_[k]=pinocchioInterface_.getData().oMf[frame].rotation().inverse(); // strict original U12 operation
  }
  if(authority_==Authority::Shadow){++evidence_->shadowComparisons;
   for(int k=0;k<2;++k){auto&a=configs_[k];auto&b=original_->getEeNormalAccelerationConstraintConfigs()[k];same(a.b,b.b,"pre b");same(a.Ax,b.Ax,"pre Ax");same(a.Av,b.Av,"pre Av");same(a.Aa,b.Aa,"pre Aa");same(R_world_to_contacts_[k],original_->getRWorldToContacts(k),"pre R");}
   for(size_t k=0;k<pinocchioInterface_.getData().oMf.size();++k){same(pinocchioInterface_.getData().oMf[k].translation(),original_->getPinocchioInterface().getData().oMf[k].translation(),"frame position");same(pinocchioInterface_.getData().oMf[k].rotation(),original_->getPinocchioInterface().getData().oMf[k].rotation(),"frame rotation");}
  }
 }
};
// Build the frozen factory config verbatim; no new gain choice or runtime setters.
inline EndEffectorDynamicsAccelerationsConstraint::Config stanceConfig(const ModelSettings::FootConstraintConfig&g){
 EndEffectorDynamicsAccelerationsConstraint::Config c;c.b.setZero(6);c.Ax.setZero(6,6);c.Av.setIdentity(6,6);c.Aa.setIdentity(6,6);
 if(!numerics::almost_eq(g.positionErrorGain_z,0.))c.Ax(2,2)=g.positionErrorGain_z;
 if(!numerics::almost_eq(g.orientationErrorGain,0.))c.Ax.block(3,3,3,3)=matrix_t::Identity(3,3)*g.orientationErrorGain;
 c.Av.block(0,0,2,2)=matrix_t::Identity(2,2)*g.linearVelocityErrorGain_xy;c.Av(2,2)=g.linearVelocityErrorGain_z;c.Av.block(3,3,3,3)=matrix_t::Identity(3,3)*g.angularVelocityErrorGain;
 c.Aa.block(0,0,2,2)=matrix_t::Identity(2,2)*g.linearAccelerationErrorGain_xy;c.Aa(2,2)=g.linearAccelerationErrorGain_z;c.Aa.block(3,3,3,3)=matrix_t::Identity(3,3)*g.angularAccelerationErrorGain;return c;
}
class GeneratedFootConstraint final:public StateInputConstraint {
 std::shared_ptr<WBAccelMpcRobotModel<ad_scalar_t>> modelOwner_;
 std::unique_ptr<StateInputConstraint> lifecycle_;
 std::unique_ptr<EndEffectorDynamics<scalar_t>> dependency_;
 mutable std::unique_ptr<EndEffectorDynamicsLinearAccConstraint> oldSwing_;
 bool stance_;size_t leg_;EndEffectorDynamicsAccelerationsConstraint::Config stanceConfig_;
 Authority authority_;std::shared_ptr<Evidence> evidence_;
 using Data=Eigen::Matrix<double,6,94>;
 static Data pack(const VectorFunctionLinearApproximation&a){Data d=Data::Zero();d.block(0,0,a.f.size(),1)=a.f;d.block(0,1,a.f.size(),58)=a.dfdx;if(a.dfdu.size())d.block(0,59,a.f.size(),35)=a.dfdu;return d;}
 SwingConfig swing(const PreComputation&pre)const {
  if(auto*p=dynamic_cast<const GeneratedPreComputation*>(&pre))return p->accelerationConfigs()[leg_];
  return cast<WBMpcPreComputation>(pre).getEeNormalAccelerationConstraintConfigs()[leg_];
 }
 VectorFunctionLinearApproximation candidate(const vector_t&x,const vector_t&u,const PreComputation&pre,bool derivatives)const {
  Data p=Data::Zero(),v=p,a=p;matrix_t Ax=matrix_t::Zero(6,6),Av=Ax,Aa=Ax;vector_t b=vector_t::Zero(6);boolean_T enabled[3];
  if(stance_){Ax=stanceConfig_.Ax;Av=stanceConfig_.Av;Aa=stanceConfig_.Aa;b=stanceConfig_.b;for(int k=0;k<3;++k)enabled[k]=true;}
  else{auto c=swing(pre);b[0]=c.b[0];enabled[0]=c.Ax.size()>0;enabled[1]=c.Av.size()>0;enabled[2]=c.Aa.size()>0;if(enabled[0])Ax.block(0,0,1,3)=c.Ax;if(enabled[1])Av.block(0,0,1,3)=c.Av;if(enabled[2])Aa.block(0,0,1,3)=c.Aa;}
  if(derivatives){
   if(enabled[0]){auto pos=dependency_->getPositionLinearApproximation(x).front();p=pack(pos);if(stance_){auto rot=dependency_->getOrientationErrorWrtPlaneLinearApproximation(x,{vector3_t(0,0,1)}).front();p.block(3,0,3,1)=rot.f;p.block(3,1,3,58)=rot.dfdx;}}
   if(enabled[1])v=pack(stance_?dependency_->getTwistLinearApproximation(x,u).front():dependency_->getVelocityLinearApproximation(x,u).front());
   if(enabled[2])a=pack(stance_?dependency_->getAccelerationsLinearApproximation(x,u).front():dependency_->getLinearAccelerationLinearApproximation(x,u).front());
  }else{
   if(enabled[0]){p.block(0,0,3,1)=dependency_->getPosition(x).front();if(stance_)p.block(3,0,3,1)=dependency_->getOrientationErrorWrtPlane(x,{vector3_t(0,0,1)}).front();}
   if(enabled[1]){if(stance_)v.col(0)=dependency_->getTwist(x,u).front();else v.block(0,0,3,1)=dependency_->getVelocity(x,u).front();}
   if(enabled[2]){if(stance_)a.col(0)=dependency_->getAccelerations(x,u).front();else a.block(0,0,3,1)=dependency_->getLinearAcceleration(x,u).front();}
  }
  Data r;footLinearResidual(b.data(),Ax.data(),Av.data(),Aa.data(),enabled,p.data(),v.data(),a.data(),r.data());++evidence_->generated;
  VectorFunctionLinearApproximation out;int n=stance_?6:1;out.f=r.block(0,0,n,1);out.dfdx=r.block(0,1,n,58);out.dfdu=r.block(0,59,n,35);return out;
 }
 VectorFunctionLinearApproximation legacy(double t,const vector_t&x,const vector_t&u,const PreComputation&pre,bool deriv)const {
  ++evidence_->legacyEvaluations;
  if(stance_){if(deriv)return lifecycle_->getLinearApproximation(t,x,u,pre);auto r=VectorFunctionLinearApproximation::Zero(6,58,35);r.f=lifecycle_->getValue(t,x,u,pre);return r;}
  oldSwing_->configure(swing(pre));if(deriv)return oldSwing_->getLinearApproximation(t,x,u,pre);auto r=VectorFunctionLinearApproximation::Zero(1,58,35);r.f=oldSwing_->getValue(t,x,u,pre);return r;
 }
 public:
 GeneratedFootConstraint(std::unique_ptr<StateInputConstraint>orig,const EndEffectorDynamics<scalar_t>&dep,bool stance,size_t leg,const ModelSettings&settings,Authority auth,std::shared_ptr<Evidence>e,std::shared_ptr<WBAccelMpcRobotModel<ad_scalar_t>> owner):StateInputConstraint(ConstraintOrder::Linear),modelOwner_(std::move(owner)),lifecycle_(std::move(orig)),dependency_(dep.clone()),oldSwing_(new EndEffectorDynamicsLinearAccConstraint(dep,1)),stance_(stance),leg_(leg),stanceConfig_(stanceConfig(settings.footConstraintConfig)),authority_(auth),evidence_(std::move(e)){}
 GeneratedFootConstraint(const GeneratedFootConstraint&r):StateInputConstraint(r),modelOwner_(r.modelOwner_),lifecycle_(r.lifecycle_->clone()),dependency_(r.dependency_->clone()),oldSwing_(r.oldSwing_->clone()),stance_(r.stance_),leg_(r.leg_),stanceConfig_(r.stanceConfig_),authority_(r.authority_),evidence_(r.evidence_){++evidence_->clones;}
 GeneratedFootConstraint*clone()const override{return new GeneratedFootConstraint(*this);}
 bool isActive(double t)const override{++evidence_->activation;return lifecycle_->isActive(t);}
 size_t getNumConstraints(double t)const override{return lifecycle_->getNumConstraints(t);}
 vector_t getValue(double t,const vector_t&x,const vector_t&u,const PreComputation&pre)const override{
  ++evidence_->value;evidence_->stack();if(authority_==Authority::Legacy)return legacy(t,x,u,pre,false).f;
  auto r=candidate(x,u,pre,false);if(authority_==Authority::Shadow){auto old=legacy(t,x,u,pre,false);++evidence_->shadowComparisons;same(r.f,old.f,"foot value",1e-12);return old.f;}return r.f;
 }
 VectorFunctionLinearApproximation getLinearApproximation(double t,const vector_t&x,const vector_t&u,const PreComputation&pre)const override{
  ++evidence_->linear;evidence_->stack();if(authority_==Authority::Legacy)return legacy(t,x,u,pre,true);
  auto r=candidate(x,u,pre,true);if(authority_==Authority::Shadow){auto old=legacy(t,x,u,pre,true);++evidence_->shadowComparisons;same(r.f,old.f,"foot f",1e-12);same(r.dfdx,old.dfdx,"foot dx",1e-12);same(r.dfdu,old.dfdu,"foot du",1e-12);return old;}return r;
 }
};
inline OptimalControlProblem install(OptimalControlProblem p,WBMpcInterface&i,std::shared_ptr<Evidence>pre,std::shared_ptr<Evidence>stance,std::shared_ptr<Evidence>swing){
 if(!std::getenv("TASK5C_WAVE_C"))return p;
 auto adModel=std::make_shared<WBAccelMpcRobotModel<ad_scalar_t>>(i.modelSettings());auto ref=i.getSwitchedModelReferenceManagerPtr();
 std::map<std::string,std::unique_ptr<EndEffectorDynamics<scalar_t>>> deps;
 for(const auto&name:i.modelSettings().contactNames6DoF)deps[name]=std::make_unique<PinocchioEndEffectorDynamicsCppAd>(i.getPinocchioInterface(),*adModel,std::vector<std::string>{name},name,i.modelSettings().modelFolderCppAd,false,false);
 std::vector<std::pair<size_t,std::string>>ordered;for(auto[n,k]:p.equalityConstraintPtr->getTermNameMap())ordered.emplace_back(k,n);std::sort(ordered.begin(),ordered.end());
 auto collection=std::make_unique<StateInputConstraintCollection>();
 for(auto[k,n]:ordered){auto term=p.equalityConstraintPtr->extract(n);bool found=false;for(size_t leg=0;leg<2;++leg){auto&name=i.modelSettings().contactNames6DoF[leg];bool isStance=n==name+"_zeroVelocity",isSwing=n==name+"_normalVelocity";if(isStance||isSwing){term=std::make_unique<GeneratedFootConstraint>(std::move(term),*deps.at(name),isStance,leg,i.modelSettings(),authorityFor(isStance?"TASK5C_STANCE_AUTHORITY":"TASK5C_SWING_AUTHORITY"),isStance?stance:swing,adModel);found=true;break;}}collection->add(n,std::move(term));}
 p.equalityConstraintPtr=std::move(collection);
 p.preComputationPtr=std::make_unique<GeneratedPreComputation>(i,authorityFor("TASK5C_PRECOMPUTATION_AUTHORITY"),pre);return p;
}
}
