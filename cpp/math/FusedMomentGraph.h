#pragma once
#include "math/LocalWrenchGeometry.h"
#include "contactMomentMarginsScalar.h"
#include <humanoid_common_mpc/contact/ContactRectangle.h>
#include "contactMomentMargins.h"
namespace task5c::momentfused {
using namespace ocs2;using namespace ocs2::humanoid;
class FusedMomentGraph {
 std::unique_ptr<CppAdInterface>ad_;
 public:
 FusedMomentGraph(WBMpcInterface&i,int leg,const std::string&task,const std::string&folder){
  auto pin=i.getPinocchioInterface().toCppAd();auto model=std::make_shared<WBAccelMpcRobotModel<ad_scalar_t>>(i.modelSettings());auto bounds=ContactRectangle::loadContactRectangle(task,i.modelSettings(),leg,false).getBounds();
  CppAdInterface::ad_parameterized_function_t fn=[pin,model,leg,bounds](const ad_vector_t&txu,const ad_vector_t&,ad_vector_t&out)mutable{
   ad_vector_t x=txu.segment(1,58),u=txu.tail(35);auto data=pin.getData();updateFramePlacements(model->getGeneralizedCoordinates(x),pin.getModel(),data);auto frame=getContactFrameIndex(pin,*model,leg);
   ad_vector3_t force=rotateVectorWorldToLocal(model->getContactForce(u,leg),data,frame),moment=rotateVectorWorldToLocal(model->getContactMoment(u,leg),data,frame);
   ad_scalar_t b[4],packed[564],result[376];b[0]=bounds.x_min;b[1]=bounds.x_max;b[2]=bounds.y_min;b[3]=bounds.y_max;for(auto&v:packed)v=0.;
   for(int k=0;k<3;++k){packed[k]=force[k];packed[k+3]=moment[k];}
   contactMomentMarginsScalar(b,packed,result);out.resize(4);for(int k=0;k<4;++k)out[k]=result[k];
  };
  ad_=std::make_unique<CppAdInterface>(fn,94,0,"task5c_matlab_moment_fused_leg"+std::to_string(leg),folder);ad_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First,false);
 }
 FusedMomentGraph(const FusedMomentGraph&r):ad_(new CppAdInterface(*r.ad_)){}
 VectorFunctionLinearApproximation evaluate(double t,const vector_t&x,const vector_t&u,bool jac)const {
  vector_t txu(94);txu<<t,x,u;VectorFunctionLinearApproximation r;r.f=ad_->getFunctionValue(txu);if(jac){auto j=ad_->getJacobian(txu);r.dfdx=j.middleCols(1,58);r.dfdu=j.rightCols(35);}else{r.dfdx.setZero(4,58);r.dfdu.setZero(4,35);}return r;
 }
};
}
