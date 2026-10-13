#include "extensions/TermRegistry.h"
#pragma once
#include "math/GeneratedFriction.h"
#include "../../generated/barriers/polynomialBarrier/lib/polynomialBarrier.h"
#include "../../generated/barriers/relaxedBarrier/lib/relaxedBarrier.h"
#include <ocs2_core/cost/StateCostCollection.h>
#include "extensions/G1StateSoftConstraint.h"

namespace task5c {
// Keep the original owning soft cost, constraint, mode gate, reduction and chain
// rule. Replace only the public scalar penalty object. JointLimits is separate.
class GeneratedPenalty final : public augmented::G1AugmentedPenaltyBase {
  std::unique_ptr<augmented::G1AugmentedPenaltyBase> original_;
  bool polynomial_;
  Authority authority_;
  std::shared_ptr<Evidence> evidence_;

  scalar_t evaluate(unsigned component, scalar_t t, scalar_t l, scalar_t h) const {
    auto old = [&]() {
      ++evidence_->legacyEvaluations;
      if (component == 0) return original_->getValue(t,l,h);
      if (component == 1) return original_->getDerivative(t,l,h);
      return original_->getSecondDerivative(t,l,h);
    };
    if (authority_ == Authority::Legacy) return old();
    vector_t config; original_->getParameters(config);
    if (config.size()!=2) throw std::runtime_error("Scalar penalty config2");
    double value, first, second;
    if (polynomial_) polynomialBarrier(h,config.data(),&value,&first,&second);
    else relaxedBarrier(h,config.data(),&value,&first,&second);
    ++evidence_->generated;
    double generated = component==0 ? value : component==1 ? first : second;
    if (authority_ == Authority::Shadow) {
      auto legacy=old(); ++evidence_->shadowComparisons;
      if (legacy!=generated) throw std::runtime_error("Scalar penalty exact shadow mismatch");
      return legacy;
    }
    return generated;
  }
 public:
  GeneratedPenalty(std::unique_ptr<augmented::G1AugmentedPenaltyBase> p, bool poly,
                   Authority a, std::shared_ptr<Evidence> e)
      : original_(std::move(p)),polynomial_(poly),authority_(a),evidence_(std::move(e)) {}
  GeneratedPenalty(const GeneratedPenalty& p)
      : original_(p.original_->clone()),polynomial_(p.polynomial_),authority_(p.authority_),evidence_(p.evidence_) {++evidence_->clones;}
  GeneratedPenalty* clone() const override {return new GeneratedPenalty(*this);}
  std::string name() const override {return original_->name();}
  scalar_t getValue(scalar_t t,scalar_t l,scalar_t h) const override {++evidence_->value;return evaluate(0,t,l,h);}
  scalar_t getDerivative(scalar_t t,scalar_t l,scalar_t h) const override {++evidence_->linear;evidence_->stack();return evaluate(1,t,l,h);}
  scalar_t getSecondDerivative(scalar_t t,scalar_t l,scalar_t h) const override {++evidence_->linear;return evaluate(2,t,l,h);}
  void setParameters(const vector_t& p) override {original_->setParameters(p);}
  void getParameters(vector_t& p) const override {original_->getParameters(p);}
  scalar_t updateMultiplier(scalar_t t,scalar_t l,scalar_t h) const override {return original_->updateMultiplier(t,l,h);}
  scalar_t initializeMultiplier() const override {return original_->initializeMultiplier();}
};

inline OptimalControlProblem installPenalties(OptimalControlProblem p,
    std::shared_ptr<Evidence> poly,std::shared_ptr<Evidence> relaxed) {
  if (!std::getenv("TASK5C_BARRIERS")) return p;
  auto replace=[&](G1MultidimensionalPenalty& penalties) {
    for (auto& original:penalties.getPenaltyPtrArray()) {
      bool polynomial=original->name()=="PieceWisePolynomialBarrierPenalty";
      if (!polynomial && original->name()!="RelaxedBarrierPenalty")
        throw std::runtime_error("Unrecognized original penalty: "+original->name());
      original=std::make_unique<GeneratedPenalty>(std::move(original),polynomial,
          authorityFor(polynomial?"TASK5C_POLYNOMIAL_AUTHORITY":"TASK5C_RELAXED_AUTHORITY"),
          polynomial?poly:relaxed);
    }
  };
  for (auto& [name,index]:g1::termNames(*p.softConstraintPtr))
    replace(p.softConstraintPtr->get<G1StateInputSoftConstraint>(name).getPenalty());
  // Only collision is a G1StateSoftConstraint; JointLimits has private concrete ownership.
  replace(p.stateSoftConstraintPtr->get<G1StateSoftConstraint>("FootCollisionSoftConstraint").getPenalty());
  return p;
}
}
