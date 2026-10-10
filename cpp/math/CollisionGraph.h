#pragma once
#include "math/LocalWrenchGeometry.h"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <humanoid_common_mpc/constraint/FootCollisionConstraint.h>
#include "collisionMarginsScalar.h"
#include "collisionMargins.h"
namespace task5c::collision {
using namespace ocs2;using namespace ocs2::humanoid;
class CollisionGraph {
 std::unique_ptr<CppAdInterface>ad_;
 public:
 CollisionGraph(WBMpcInterface&i,const std::string&task,const std::string&folder){
  auto pin=i.getPinocchioInterface().toCppAd();auto model=std::make_shared<WBAccelMpcRobotModel<ad_scalar_t>>(i.modelSettings());auto cfg=FootCollisionConstraint::loadFootCollisionConstraintConfig(task,false);
  std::vector<std::string>names={cfg.leftAnkleFrame,cfg.rightAnkleFrame,cfg.leftFootCenterFrame,cfg.rightFootCenterFrame,cfg.leftFootFrame1,cfg.rightFootFrame1,cfg.leftFootFrame2,cfg.rightFootFrame2,cfg.leftKneeFrame,cfg.rightKneeFrame};
  CppAdInterface::ad_parameterized_function_t fn=[pin,model,names](const ad_vector_t&tx,const ad_vector_t&parameters,ad_vector_t&out)mutable{
   ad_vector_t x=tx.tail(58);auto data=pin.getData();ad_vector_t q=model->getGeneralizedCoordinates(x);pinocchio::forwardKinematics(pin.getModel(),data,q);
   ad_scalar_t positions[30],r[2],result[16];r[0]=parameters[0];r[1]=parameters[1];for(int n=0;n<10;++n){ad_vector3_t pos=pinocchio::updateFramePlacement(pin.getModel(),data,pin.getModel().getFrameId(names[n])).translation();for(int k=0;k<3;++k)positions[3*n+k]=pos[k];}
   collisionMarginsScalar(positions,r,result);out.resize(16);for(int k=0;k<16;++k)out[k]=result[k];
  };
  ad_=std::make_unique<CppAdInterface>(fn,59,2,"task5c_matlab_collision",folder);ad_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First,false);
 }
 CollisionGraph(const CollisionGraph&r):ad_(new CppAdInterface(*r.ad_)){}
 VectorFunctionLinearApproximation evaluate(double t,const vector_t&x,const vector_t&parameters,bool jac)const {
  vector_t tx(59);tx<<t,x;VectorFunctionLinearApproximation r;r.f=ad_->getFunctionValue(tx,parameters);if(jac)r.dfdx=ad_->getJacobian(tx,parameters).rightCols(58);else r.dfdx.setZero(16,58);r.dfdu.resize(16,0);return r;
 }
};
}
