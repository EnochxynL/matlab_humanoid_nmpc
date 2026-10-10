#pragma once
#include "math/GeneratedJointLimits.h"
#include "math/GeneratedNominals.h"
#include "quadraticTerminal.h"
#include "quadraticRunning.h"
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
namespace task5c {
inline void costShadow(const ScalarFunctionQuadraticApproximation& a,
    const ScalarFunctionQuadraticApproximation& b) {
  // Frozen oracle tolerance for reductions; no tolerance on complete policies.
  auto compare=[](const matrix_t& x,const matrix_t& y){
    return x.rows()==y.rows() && x.cols()==y.cols() &&
       ((x-y).array().abs() <= 1e-10+2e-14*y.array().abs()).all();
  };
  if(std::abs(a.f-b.f)>1e-10+2e-14*std::abs(b.f) ||
      !compare(a.dfdx,b.dfdx)||!compare(a.dfdu,b.dfdu)||!compare(a.dfdxx,b.dfdxx)||
      !compare(a.dfduu,b.dfduu)||!compare(a.dfdux,b.dfdux))
    throw std::runtime_error("Frozen quadratic shadow mismatch");
}
class GeneratedTerminal final : public StateCost {
  std::unique_ptr<StateCost> old_; Authority authority_; std::shared_ptr<Evidence> e_;
  ScalarFunctionQuadraticApproximation candidate(scalar_t t,const vector_t& x,const TargetTrajectories& target) const {
    matrix_t q;dynamic_cast<QuadraticStateCost&>(*old_).getGains(q);
    const auto desired=target.getDesiredState(t);ScalarFunctionQuadraticApproximation a;
    a.dfdx.resize(58);a.dfdxx.resize(58,58);
    quadraticTerminal(x.data(),desired.data(),q.data(),&a.f,a.dfdx.data(),a.dfdxx.data());++e_->generated;return a;
  }
 public:
  GeneratedTerminal(std::unique_ptr<StateCost> old,Authority a,std::shared_ptr<Evidence> e):old_(std::move(old)),authority_(a),e_(std::move(e)){}
  GeneratedTerminal(const GeneratedTerminal& p):old_(p.old_->clone()),authority_(p.authority_),e_(p.e_){++e_->clones;}
  GeneratedTerminal* clone() const override{return new GeneratedTerminal(*this);}
  bool isActive(scalar_t t) const override{++e_->activation;return old_->isActive(t);}
  scalar_t getValue(scalar_t t,const vector_t& x,const TargetTrajectories& target,const PreComputation& pre) const override {
    ++e_->value;if(authority_==Authority::Legacy){++e_->legacyEvaluations;return old_->getValue(t,x,target,pre);}
    auto a=candidate(t,x,target);if(authority_==Authority::Shadow){auto old=old_->getValue(t,x,target,pre);++e_->legacyEvaluations;++e_->shadowComparisons;if(std::abs(a.f-old)>1e-10+2e-14*std::abs(old))throw std::runtime_error("Terminal value shadow mismatch");return old;}return a.f;
  }
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t,const vector_t& x,const TargetTrajectories& target,const PreComputation& pre) const override {
    ++e_->linear;e_->stack();if(authority_==Authority::Legacy){++e_->legacyEvaluations;return old_->getQuadraticApproximation(t,x,target,pre);}
    auto a=candidate(t,x,target);if(authority_==Authority::Shadow){auto old=old_->getQuadraticApproximation(t,x,target,pre);++e_->legacyEvaluations;++e_->shadowComparisons;costShadow(a,old);return old;}return a;
  }
};
class GeneratedRunning final : public StateInputCost {
  std::unique_ptr<StateInputCost> old_;Authority authority_;std::shared_ptr<Evidence> e_;
  std::shared_ptr<Evidence> nominalStateEvidence_,nominalInputEvidence_;
  const SwitchedModelReferenceManager* ref_;const PinocchioInterface* pin_;const MpcRobotModelBase<scalar_t>* model_;
  ScalarFunctionQuadraticApproximation candidate(scalar_t t,const vector_t& x,const vector_t& u,const TargetTrajectories& target) const {
    matrix_t q,r,p;dynamic_cast<QuadraticStateInputCost&>(*old_).getGains(q,r,p);
    if(p.size())throw std::runtime_error("Frozen WB quadratic P must remain empty");
    const auto nominalX=generatedNominalState(*ref_,*model_,target,x,t,nominalStateEvidence_);
    const auto nominalU=generatedNominalInput(*pin_,ref_->getContactFlags(t),*model_,nominalInputEvidence_);
    Task5CQuadraticRunning generated;
    quadraticRunning(x.data(),u.data(),nominalX.data(),nominalU.data(),q.data(),r.data(),&generated);
    ++e_->generated;ScalarFunctionQuadraticApproximation a;
    a.f=generated.f;a.dfdx=Eigen::Map<vector_t>(generated.dfdx,58);a.dfdu=Eigen::Map<vector_t>(generated.dfdu,35);
    a.dfdxx=Eigen::Map<matrix_t>(generated.dfdxx,58,58);a.dfduu=Eigen::Map<matrix_t>(generated.dfduu,35,35);a.dfdux=Eigen::Map<matrix_t>(generated.dfdux,35,58);return a;
  }
 public:
  GeneratedRunning(std::unique_ptr<StateInputCost> old,Authority a,std::shared_ptr<Evidence> e,WBMpcInterface& i,std::shared_ptr<Evidence> ns,std::shared_ptr<Evidence> ni)
    :old_(std::move(old)),authority_(a),e_(std::move(e)),nominalStateEvidence_(std::move(ns)),nominalInputEvidence_(std::move(ni)),ref_(i.getSwitchedModelReferenceManagerPtr().get()),pin_(&i.getPinocchioInterface()),model_(&i.getMpcRobotModel()){}
  GeneratedRunning(const GeneratedRunning& p):old_(p.old_->clone()),authority_(p.authority_),e_(p.e_),nominalStateEvidence_(p.nominalStateEvidence_),nominalInputEvidence_(p.nominalInputEvidence_),ref_(p.ref_),pin_(p.pin_),model_(p.model_){++e_->clones;}
  GeneratedRunning* clone() const override{return new GeneratedRunning(*this);}
  bool isActive(scalar_t t) const override{++e_->activation;return old_->isActive(t);}
  scalar_t getValue(scalar_t t,const vector_t& x,const vector_t& u,const TargetTrajectories& target,const PreComputation& pre) const override{
    ++e_->value;if(authority_==Authority::Legacy){++e_->legacyEvaluations;return old_->getValue(t,x,u,target,pre);}
    auto a=candidate(t,x,u,target);if(authority_==Authority::Shadow){auto old=old_->getValue(t,x,u,target,pre);++e_->legacyEvaluations;++e_->shadowComparisons;if(std::abs(a.f-old)>1e-10+2e-14*std::abs(old))throw std::runtime_error("Running value shadow mismatch");return old;}return a.f;
  }
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t,const vector_t& x,const vector_t& u,const TargetTrajectories& target,const PreComputation& pre) const override{
    ++e_->linear;e_->stack();if(authority_==Authority::Legacy){++e_->legacyEvaluations;return old_->getQuadraticApproximation(t,x,u,target,pre);}
    auto a=candidate(t,x,u,target);if(authority_==Authority::Shadow){auto old=old_->getQuadraticApproximation(t,x,u,target,pre);++e_->legacyEvaluations;++e_->shadowComparisons;costShadow(a,old);return old;}return a;
  }
};
inline OptimalControlProblem installQuadraticCosts(OptimalControlProblem p,WBMpcInterface& i,
    std::shared_ptr<Evidence> terminal,std::shared_ptr<Evidence> running,std::shared_ptr<Evidence> ns,std::shared_ptr<Evidence> ni) {
  if(!std::getenv("TASK5C_QUADRATIC"))return p;
  auto terminalOld=p.finalCostPtr->extract("terminalCost");
  p.finalCostPtr->add("terminalCost",std::make_unique<GeneratedTerminal>(std::move(terminalOld),authorityFor("TASK5C_TERMINAL_AUTHORITY"),terminal));
  // Both collections each contain exactly this one term in the frozen WB OCP.
  auto runningOld=p.costPtr->extract("stateInputQuadraticCost");
  p.costPtr->add("stateInputQuadraticCost",std::make_unique<GeneratedRunning>(std::move(runningOld),authorityFor("TASK5C_RUNNING_AUTHORITY"),running,i,ns,ni));
  return p;
}
}
