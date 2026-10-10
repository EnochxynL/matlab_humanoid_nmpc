#pragma once
#include "math/GeneratedPenalties.h"
#include "jointLimitCost.h"
#include <humanoid_common_mpc/constraint/JointLimitsSoftConstraint.h>
#include <humanoid_common_mpc/pinocchio_model/pinocchioUtils.h>
namespace task5c {
class GeneratedJointLimits final : public StateCost {
  std::unique_ptr<StateCost> original_;
  std::pair<vector_t,vector_t> bounds_;
  Authority authority_;
  std::shared_ptr<Evidence> evidence_;
  ScalarFunctionQuadraticApproximation candidate(const vector_t& x) const {
    scalar_t mu,delta;
    dynamic_cast<JointLimitsSoftConstraint&>(*original_).getGains(mu,delta);
    double config[2]={mu,delta};
    ScalarFunctionQuadraticApproximation a;
    a.dfdx.resize(58);a.dfdxx.resize(58,58);
    jointLimitCost(x.data()+6,bounds_.first.data(),bounds_.second.data(),config,
                   &a.f,a.dfdx.data(),a.dfdxx.data());
    ++evidence_->generated;return a;
  }
 public:
  GeneratedJointLimits(std::unique_ptr<StateCost> p,std::pair<vector_t,vector_t> bounds,
      Authority authority,std::shared_ptr<Evidence> e)
      :original_(std::move(p)),bounds_(std::move(bounds)),authority_(authority),evidence_(std::move(e)){}
  GeneratedJointLimits(const GeneratedJointLimits& p)
      :original_(p.original_->clone()),bounds_(p.bounds_),authority_(p.authority_),evidence_(p.evidence_){++evidence_->clones;}
  GeneratedJointLimits* clone() const override {return new GeneratedJointLimits(*this);}
  bool isActive(scalar_t t) const override {++evidence_->activation;return original_->isActive(t);}
  scalar_t getValue(scalar_t t,const vector_t& x,const TargetTrajectories& target,const PreComputation& pre) const override {
    ++evidence_->value;
    if(authority_==Authority::Legacy){++evidence_->legacyEvaluations;return original_->getValue(t,x,target,pre);}
    auto a=candidate(x);
    if(authority_==Authority::Shadow){auto old=original_->getValue(t,x,target,pre);++evidence_->legacyEvaluations;++evidence_->shadowComparisons;
      if(old!=a.f)throw std::runtime_error("Joint cost exact shadow mismatch");return old;}
    return a.f;
  }
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t,const vector_t& x,
      const TargetTrajectories& target,const PreComputation& pre) const override {
    ++evidence_->linear;evidence_->stack();
    if(authority_==Authority::Legacy){++evidence_->legacyEvaluations;return original_->getQuadraticApproximation(t,x,target,pre);}
    auto a=candidate(x);
    if(authority_==Authority::Shadow){auto old=original_->getQuadraticApproximation(t,x,target,pre);++evidence_->legacyEvaluations;++evidence_->shadowComparisons;
      if(old.f!=a.f || !(old.dfdx.array()==a.dfdx.array()).all() || !(old.dfdxx.array()==a.dfdxx.array()).all())
        throw std::runtime_error("Joint full quadratic exact shadow mismatch");return old;}
    return a;
  }
};
inline OptimalControlProblem installJointLimits(OptimalControlProblem p,WBMpcInterface& i,std::shared_ptr<Evidence> e) {
  if(!std::getenv("TASK5C_JOINT"))return p;
  auto& model=i.getMpcRobotModel();
  if(model.getStateDim()!=58 || model.getJointDim()!=23 || model.getJointStartindex()!=6)
    throw std::runtime_error("Audited JointLimit WB packing changed");
  auto bounds=readPinocchioJointLimits(i.getPinocchioInterface(),i.modelSettings());
  auto replacement=std::make_unique<StateCostCollection>();
  std::vector<std::pair<size_t,std::string>> ordered;
  for(auto& [name,index]:p.stateSoftConstraintPtr->getTermNameMap())ordered.emplace_back(index,name);
  std::sort(ordered.begin(),ordered.end());
  for(auto& [index,name]:ordered){auto old=p.stateSoftConstraintPtr->extract(name);
    if(name=="jointLimits")old=std::make_unique<GeneratedJointLimits>(std::move(old),bounds,authorityFor("TASK5C_JOINT_AUTHORITY"),e);
    replacement->add(name,std::move(old));}
  p.stateSoftConstraintPtr=std::move(replacement);return p;
}
}
