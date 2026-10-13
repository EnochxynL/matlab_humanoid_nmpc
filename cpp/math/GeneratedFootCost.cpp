#include "extensions/TermRegistry.h"
#include "math/GeneratedFootCost.h"
#include "math/FootCostGraph.h"

namespace task5c::footcost {
GeneratedFootCost::GeneratedFootCost(EndEffectorDynamicsFootCost&old,WBMpcInterface&i,int leg,const std::string&cache,Authority a,std::shared_ptr<Evidence>e):original_(old.clone()),graph_(std::make_unique<FootCostGraph>(i,leg,std::unique_ptr<EndEffectorDynamicsFootCost>(old.clone()),cache)),auth_(a),evidence_(std::move(e)){}
GeneratedFootCost::GeneratedFootCost(const GeneratedFootCost&r):StateInputCost(r),original_(r.original_->clone()),graph_(r.graph_->clone()),auth_(r.auth_),evidence_(r.evidence_){++evidence_->clones;}
GeneratedFootCost*GeneratedFootCost::clone()const{return new GeneratedFootCost(*this);}
bool GeneratedFootCost::isActive(double t)const{++evidence_->activation;return graph_->isActive(t);}
scalar_t GeneratedFootCost::getValue(double t,const vector_t&x,const vector_t&u,const TargetTrajectories&target,const PreComputation&pre)const{
  ++evidence_->value;evidence_->stack();if(auth_==Authority::Legacy){++evidence_->legacyEvaluations;return original_->getValue(t,x,u,target,pre);}++evidence_->generated;auto f=graph_->getValue(t,x,u,target,pre);
  if(auth_==Authority::Shadow){++evidence_->legacyEvaluations;++evidence_->shadowComparisons;auto old=original_->getValue(t,x,u,target,pre);if(!std::isfinite(f)||!std::isfinite(old)||f!=old)throw std::runtime_error("foot cost value exact gate");return old;}return f;
 }
ScalarFunctionQuadraticApproximation GeneratedFootCost::getQuadraticApproximation(double t,const vector_t&x,const vector_t&u,const TargetTrajectories&target,const PreComputation&pre)const{
  ++evidence_->linear;evidence_->stack();if(auth_==Authority::Legacy){++evidence_->legacyEvaluations;return original_->getQuadraticApproximation(t,x,u,target,pre);}++evidence_->generated;auto a=graph_->getQuadraticApproximation(t,x,u,target,pre);
  if(auth_==Authority::Shadow){++evidence_->legacyEvaluations;++evidence_->shadowComparisons;auto b=original_->getQuadraticApproximation(t,x,u,target,pre);if(!std::isfinite(a.f)||!std::isfinite(b.f)||a.f!=b.f)throw std::runtime_error("foot cost f exact gate");for(auto pair:{std::pair<matrix_t,matrix_t>(a.dfdx,b.dfdx),{a.dfdu,b.dfdu},{a.dfdxx,b.dfdxx},{a.dfdux,b.dfdux},{a.dfduu,b.dfduu}})if(!pair.first.allFinite()||!pair.second.allFinite()||(pair.first.array()!=pair.second.array()).any())throw std::runtime_error("foot cost quadratic exact gate");return b;}return a;
 }
OptimalControlProblem installFootCosts(OptimalControlProblem p,WBMpcInterface&i,std::shared_ptr<Evidence>e){
 if(!std::getenv("TASK5C_FOOT_COST"))return p;const char*cache=std::getenv("TASK5C_FOOT_COST_CACHE");if(!cache)throw std::runtime_error("Require isolated foot cost cache");
 std::vector<std::pair<size_t,std::string>>names;for(auto [name,index]:g1::termNames(*p.costPtr))names.emplace_back(index,name);std::sort(names.begin(),names.end());auto replacement=std::make_unique<StateInputCostCollection>();int replaced=0;
 for(auto [index,name]:names){int leg=-1;for(int k=0;k<2;++k)if(name==i.modelSettings().contactNames6DoF[k]+"_TaskSpaceTrackingCost")leg=k;if(leg>=0){auto&old=dynamic_cast<EndEffectorDynamicsFootCost&>(p.costPtr->get(name));g1::addTerm(*replacement,name,std::make_unique<GeneratedFootCost>(old,i,leg,cache,authorityFor("TASK5C_FOOT_COST_AUTHORITY"),e));++replaced;}else g1::addTerm(*replacement,name,std::unique_ptr<StateInputCost>(p.costPtr->get(name).clone()));}if(replaced!=2)throw std::runtime_error("Foot owning cost boundary not found");p.costPtr=std::move(replacement);return p;
}
}
