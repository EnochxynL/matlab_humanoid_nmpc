#pragma once
#include "DiagnosticBuffer.h"
#include <limits>
#include <algorithm>
#include <array>
#include <memory>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <humanoid_common_mpc/common/MpcRobotModelBase.h>
#include <humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
namespace g1_controller::diagnostics {

class FootDiagnostics {
 std::unique_ptr<ocs2::PinocchioInterface> pin_;
 std::unique_ptr<ocs2::humanoid::MpcRobotModelBase<double>> model_;
 std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> ref_;
 g1_controller::diagnostics::DiagnosticBuffer& buffer_;std::array<pinocchio::FrameIndex,2> ids_;
 public:
 explicit FootDiagnostics(g1_controller::diagnostics::DiagnosticBuffer& buffer):buffer_(buffer){
  buffer_.define("foot_kinematics","solver_target_time,observation_time,sample_time,kind,leg,mode,planned_support,z_reference,x,y,z,vx,vy,vz,min_sole_z");
 }
 void configure(const ocs2::PinocchioInterface&pin,const ocs2::humanoid::MpcRobotModelBase<double>&m,std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager>ref){
  pin_=std::make_unique<ocs2::PinocchioInterface>(pin);model_.reset(m.clone());ref_=ref;
  for(int i=0;i<2;++i)ids_[i]=pin_->getModel().getFrameId(m.modelSettings.contactNames6DoF.at(i));
 }
 void write(double target,double observation,double t,const char*kind,const ocs2::vector_t&x,const ocs2::vector_t&u,const ocs2::ModeSchedule&schedule){
  const auto&q=model_->getGeneralizedCoordinates(x);const auto&v=model_->getGeneralizedVelocities(x,u);
  auto&data=pin_->getData();const auto&model=pin_->getModel();pinocchio::forwardKinematics(model,data,q,v);pinocchio::updateFramePlacements(model,data);
  auto mode=schedule.modeAtTime(t);auto flags=ocs2::humanoid::modeNumber2StanceLeg(mode);
  for(int leg=0;leg<2;++leg){auto pose=data.oMf[ids_[leg]];auto velocity=pinocchio::getFrameVelocity(model,data,ids_[leg],pinocchio::LOCAL_WORLD_ALIGNED);double clearance=std::numeric_limits<double>::infinity();
   // Accepted contact-frame point-cloud geometry, both feet; ground z=0.
   for(auto point:{Eigen::Vector3d(-.085,.025,0),Eigen::Vector3d(-.085,-.025,0),Eigen::Vector3d(.085,.03,0),Eigen::Vector3d(.085,-.03,0)})clearance=std::min(clearance,(pose.translation()+pose.rotation()*point)[2]);
   buffer_.row("foot_kinematics",{target,observation,t,kind,leg,mode,flags[leg],
       ref_->getSwingTrajectoryPlanner()->getZpositionConstraint(leg,t),
       pose.translation()[0],pose.translation()[1],pose.translation()[2],
       velocity.linear()[0],velocity.linear()[1],velocity.linear()[2],clearance});
  }

 }
};

} // namespace g1_controller::diagnostics
