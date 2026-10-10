#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <humanoid_common_mpc/pinocchio_model/PinocchioFrameConversions.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
namespace task5c::geometry {
using namespace ocs2;using namespace ocs2::humanoid;
class LocalWrenchGeometry {
 std::unique_ptr<CppAdInterface> ad_;
 public:
 LocalWrenchGeometry(WBMpcInterface&i,int leg,const std::string&folder){
  auto pin=i.getPinocchioInterface().toCppAd();auto model=std::make_shared<WBAccelMpcRobotModel<ad_scalar_t>>(i.modelSettings());
  CppAdInterface::ad_function_t fn=[pin,model,leg](const ad_vector_t&xu,ad_vector_t&out)mutable{
   auto data=pin.getData();ad_vector_t x=xu.head(58),u=xu.tail(35);
   updateFramePlacements(model->getGeneralizedCoordinates(x),pin.getModel(),data);
   auto frame=getContactFrameIndex(pin,*model,leg);
   out.resize(6);out.head(3)=rotateVectorWorldToLocal(model->getContactForce(u,leg),data,frame);
   out.tail(3)=rotateVectorWorldToLocal(model->getContactMoment(u,leg),data,frame);
  };
  ad_=std::make_unique<CppAdInterface>(fn,93,"task5c_local_wrench_leg"+std::to_string(leg),folder);
  ad_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First,false);
 }
 LocalWrenchGeometry(const LocalWrenchGeometry&r):ad_(new CppAdInterface(*r.ad_)){}
 matrix_t pack(const vector_t&x,const vector_t&u,bool jac)const{
  vector_t xu(93);xu<<x,u;matrix_t p=matrix_t::Zero(6,94);p.col(0)=ad_->getFunctionValue(xu);
  if(jac)p.rightCols(93)=ad_->getJacobian(xu);return p;
 }
};
}
