#include "extensions/G1RotationTransforms.h"
#pragma once
#include "math/LocalWrenchGeometry.h"
#include <humanoid_wb_mpc/cost/EndEffectorDynamicsFootCost.h>
#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>
#include <humanoid_wb_mpc/end_effector/PinocchioEndEffectorDynamicsCppAd.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "footCostResidualScalar.h"
#include "footCostResidual.h"
namespace task5c::footcost {
using namespace ocs2;using namespace ocs2::humanoid;
class FootCostGraph final:public StateInputCostGaussNewtonAd {
 std::unique_ptr<EndEffectorDynamicsFootCost>parameters_;
 mutable PinocchioInterfaceCppAd pin_;
 std::shared_ptr<WBAccelMpcRobotModel<ad_scalar_t>>model_;
 pinocchio::FrameIndex frame_;
 public:
 FootCostGraph(WBMpcInterface&i,int leg,std::unique_ptr<EndEffectorDynamicsFootCost>old,const std::string&folder):parameters_(std::move(old)),pin_(i.getPinocchioInterface().toCppAd()),model_(std::make_shared<WBAccelMpcRobotModel<ad_scalar_t>>(i.modelSettings())),frame_(pin_.getModel().getFrameId(i.modelSettings().contactNames[leg])){
  initialize(58,35,37,"task5c_matlab_foot_cost_leg"+std::to_string(leg),folder,false,false);
 }
 FootCostGraph(const FootCostGraph&r):StateInputCostGaussNewtonAd(r),parameters_(r.parameters_->clone()),pin_(r.pin_),model_(r.model_),frame_(r.frame_){}
 FootCostGraph*clone()const override{return new FootCostGraph(*this);}
 bool isActive(double t)const override{return parameters_->isActive(t);}
 vector_t getParameters(double t,const TargetTrajectories&target,const PreComputation&pre)const override{return parameters_->getParameters(t,target,pre);} // Includes original target queries and impact factor, no dead-query removal.
 protected:
 ad_vector_t costVectorFunction(ad_scalar_t,const ad_vector_t&state,const ad_vector_t&input,const ad_vector_t&parameters) const override{
  const auto rf=pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;const auto&model=pin_.getModel();auto&data=pin_.getData();ad_vector_t q=model_->getGeneralizedCoordinates(state),v=model_->getGeneralizedVelocities(state,input);ad_vector_t a=computeGeneralizedAccelerations<ad_scalar_t>(state,input,pin_,*model_);pinocchio::forwardKinematics(model,data,q,v,a);auto frameData=pinocchio::updateFramePlacement(model,data,frame_);
  ad_vector3_t linearVelocity=pinocchio::getFrameVelocity(model,data,frame_,rf).linear();Eigen::Quaternion<ad_scalar_t> orientation=g1::geometry::matrixToQuaternion(frameData.rotation());ad_vector3_t angularVelocity=pinocchio::getFrameVelocity(model,data,frame_,rf).angular();auto accel=pinocchio::getFrameClassicalAcceleration(model,data,frame_,rf);ad_vector3_t linearAccel=accel.linear(),angularAccel=accel.angular();
  const PlanarEndEffectorDynamicsReference<ad_scalar_t>reference(parameters.head(18));ad_vector3_t plane=g1::geometry::quaternionDistanceToPlane<ad_scalar_t>(orientation,reference.getPlaneNormal());ad_scalar_t geometry[18],p[37],result[18];for(auto&n:geometry)n=0.;for(int k=0;k<3;++k){geometry[k+3]=plane[k];geometry[k+6]=linearVelocity[k];geometry[k+9]=angularVelocity[k];geometry[k+12]=linearAccel[k];geometry[k+15]=angularAccel[k];}for(int k=0;k<37;++k)p[k]=parameters[k];
  footCostResidualScalar(geometry,p,result);ad_vector_t out(18);for(int k=0;k<18;++k)out[k]=result[k];return out;
 }
};
}
