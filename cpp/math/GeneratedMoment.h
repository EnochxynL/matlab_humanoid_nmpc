#include "extensions/ActiveConstraint.h"
#pragma once
#include "math/FusedMomentGraph.h"
#include "math/GeneratedFootContracts.h"
#include <humanoid_common_mpc/contact/ContactRectangle.h>
#include "extensions/G1StateInputSoftConstraint.h"
#include "contactMomentMargins.h"
namespace task5c::momentfused {
class GeneratedMoment final:public StateInputConstraint, public g1::ActiveConstraint {
 std::unique_ptr<StateInputConstraint> original_;FusedMomentGraph dependency_;Authority auth_;std::shared_ptr<Evidence> evidence_;
 VectorFunctionLinearApproximation candidate(double t,const vector_t&x,const vector_t&u,bool jac)const {++evidence_->generated;return dependency_.evaluate(t,x,u,jac);}
 public:
 GeneratedMoment(WBMpcInterface&i,int leg,const std::string&task,const std::string&cache,std::unique_ptr<StateInputConstraint>old,Authority a,std::shared_ptr<Evidence>e):StateInputConstraint(ConstraintOrder::Linear),original_(std::move(old)),dependency_(i,leg,task,cache),auth_(a),evidence_(std::move(e)){}
 GeneratedMoment(const GeneratedMoment&r):StateInputConstraint(r),original_(r.original_->clone()),dependency_(r.dependency_),auth_(r.auth_),evidence_(r.evidence_){++evidence_->clones;}
 GeneratedMoment*clone()const override{return new GeneratedMoment(*this);}
 bool isActive(double t)const override{++evidence_->activation;return original_->isActive(t);}
 void setActive(bool v)override{g1::setActive(*original_,v);}bool getActive()const override{return g1::active(*original_);}
 size_t getNumConstraints(double t)const override{return original_->getNumConstraints(t);}
 vector_t getValue(double t,const vector_t&x,const vector_t&u,const PreComputation&p)const override{
  ++evidence_->value;evidence_->stack();if(auth_==Authority::Legacy){++evidence_->legacyEvaluations;return original_->getValue(t,x,u,p);}
  auto r=candidate(t,x,u,false);if(auth_==Authority::Shadow){++evidence_->legacyEvaluations;++evidence_->shadowComparisons;auto old=original_->getValue(t,x,u,p);task5c::wavec::same(r.f,old,"moment value",1e-12);return old;}return r.f;
 }
 VectorFunctionLinearApproximation getLinearApproximation(double t,const vector_t&x,const vector_t&u,const PreComputation&p)const override{
  ++evidence_->linear;evidence_->stack();if(auth_==Authority::Legacy){++evidence_->legacyEvaluations;return original_->getLinearApproximation(t,x,u,p);}
  auto r=candidate(t,x,u,true);if(auth_==Authority::Shadow){++evidence_->legacyEvaluations;++evidence_->shadowComparisons;auto old=original_->getLinearApproximation(t,x,u,p);task5c::wavec::same(r.f,old.f,"moment f",1e-12);task5c::wavec::same(r.dfdx,old.dfdx,"moment dx",1e-12);task5c::wavec::same(r.dfdu,old.dfdu,"moment du",1e-12);return old;}return r;
 }
};
inline OptimalControlProblem installMoment(OptimalControlProblem p,WBMpcInterface&i,const std::string&task,std::shared_ptr<Evidence>e){
 if(!std::getenv("TASK5C_MOMENT"))return p;
 auto cache=std::getenv("TASK5C_GEOMETRY_CACHE");if(!cache)throw std::runtime_error("Require isolated geometry cache path");
 for(int leg=0;leg<2;++leg){auto name=i.modelSettings().contactNames6DoF[leg]+"_contactMomentXY";auto&cost=p.softConstraintPtr->get<G1StateInputSoftConstraint>(name);auto&ptr=cost.getConstraintPtr();ptr=std::make_unique<GeneratedMoment>(i,leg,task,cache,std::move(ptr),authorityFor("TASK5C_MOMENT_AUTHORITY"),e);}
 return p;
}
}
