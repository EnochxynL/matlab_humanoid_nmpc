#pragma once
#include "math/CollisionGraph.h"
#include "math/GeneratedMoment.h"
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
namespace task5c::collision {
class GeneratedCollisionCost final:public StateCost {
 std::unique_ptr<StateSoftConstraint>original_;std::unique_ptr<FootCollisionConstraint>lifecycle_;MultidimensionalPenalty penalty_;CollisionGraph graph_;Authority auth_;std::shared_ptr<Evidence>evidence_;
 public:
 GeneratedCollisionCost(StateSoftConstraint&old,WBMpcInterface&i,const std::string&task,const std::string&cache,Authority a,std::shared_ptr<Evidence>e):original_(old.clone()),lifecycle_(old.get<FootCollisionConstraint>().clone()),penalty_(old.getPenalty()),graph_(i,task,cache),auth_(a),evidence_(std::move(e)){}
 GeneratedCollisionCost(const GeneratedCollisionCost&r):StateCost(r),original_(r.original_->clone()),lifecycle_(r.lifecycle_->clone()),penalty_(r.penalty_),graph_(r.graph_),auth_(r.auth_),evidence_(r.evidence_){++evidence_->clones;}
 GeneratedCollisionCost*clone()const override{return new GeneratedCollisionCost(*this);}
 bool isActive(double t)const override{++evidence_->activation;return lifecycle_->isActive(t);}
 void setActive(bool b){lifecycle_->setActive(b);original_->get<FootCollisionConstraint>().setActive(b);}
 void setSphereRadii(double foot,double knee){lifecycle_->setSphereRadii(foot,knee);original_->get<FootCollisionConstraint>().setSphereRadii(foot,knee);}
 scalar_t getValue(double t,const vector_t&x,const TargetTrajectories&target,const PreComputation&pre)const override{
  ++evidence_->value;evidence_->stack();if(auth_==Authority::Legacy){++evidence_->legacyEvaluations;return original_->getValue(t,x,target,pre);}
  auto c=graph_.evaluate(t,x,lifecycle_->getParameters(t,pre),false);++evidence_->generated;auto f=penalty_.getValue(t,c.f);
  if(auth_==Authority::Shadow){++evidence_->legacyEvaluations;++evidence_->shadowComparisons;auto old=original_->getValue(t,x,target,pre);if(std::abs(f-old)>1e-12+2e-14*std::abs(old))throw std::runtime_error("collision cost f");return old;}return f;
 }
 ScalarFunctionQuadraticApproximation getQuadraticApproximation(double t,const vector_t&x,const TargetTrajectories&target,const PreComputation&pre)const override{
  ++evidence_->linear;evidence_->stack();if(auth_==Authority::Legacy){++evidence_->legacyEvaluations;return original_->getQuadraticApproximation(t,x,target,pre);}
  auto c=graph_.evaluate(t,x,lifecycle_->getParameters(t,pre),true);++evidence_->generated;auto a=penalty_.getQuadraticApproximation(t,c);
  if(auth_==Authority::Shadow){++evidence_->legacyEvaluations;++evidence_->shadowComparisons;auto b=original_->getQuadraticApproximation(t,x,target,pre);if(std::abs(a.f-b.f)>1e-12+2e-14*std::abs(b.f))throw std::runtime_error("collision cost f");task5c::wavec::same(a.dfdx,b.dfdx,"collision g",1e-12);task5c::wavec::same(a.dfdxx,b.dfdxx,"collision H",1e-12);return b;}return a;
 }
};
inline OptimalControlProblem installCollision(OptimalControlProblem p,WBMpcInterface&i,const std::string&task,std::shared_ptr<Evidence>e){
 if(!std::getenv("TASK5C_COLLISION"))return p;const char*cache=std::getenv("TASK5C_COLLISION_CACHE");if(!cache)throw std::runtime_error("Require isolated collision cache");
 std::vector<std::pair<size_t,std::string>>names;for(auto [name,index]:p.stateSoftConstraintPtr->getTermNameMap())names.emplace_back(index,name);std::sort(names.begin(),names.end());auto replacement=std::make_unique<StateCostCollection>();
 for(auto [index,name]:names){if(name=="FootCollisionSoftConstraint")replacement->add(name,std::make_unique<GeneratedCollisionCost>(p.stateSoftConstraintPtr->get<StateSoftConstraint>(name),i,task,cache,authorityFor("TASK5C_COLLISION_AUTHORITY"),e));else replacement->add(name,std::unique_ptr<StateCost>(p.stateSoftConstraintPtr->get(name).clone()));}p.stateSoftConstraintPtr=std::move(replacement);return p;
}
}
