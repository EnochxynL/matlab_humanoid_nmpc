#pragma once
#include "math/GeneratedConstraint.h"
#include "nominalState.h"
#include "nominalInput.h"
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <ocs2_core/initialization/Initializer.h>
namespace task5c {
inline vector_t generatedNominalInput(const PinocchioInterface& pin,
    const contact_flag_t& flags,const MpcRobotModelBase<scalar_t>& model,
    const std::shared_ptr<Evidence>& evidence) {
  if(!std::getenv("TASK5C_NOMINAL"))return weightCompensatingInput(pin,flags,model);
  auto authority=authorityFor("TASK5C_NOMINAL_INPUT_AUTHORITY");
  if(authority==Authority::Legacy){++evidence->legacyEvaluations;return weightCompensatingInput(pin,flags,model);}
  vector_t u(35);boolean_T contacts[2]={flags[0],flags[1]};
  nominalInput(computeTotalMass(pin.getModel()),contacts,u.data());++evidence->generated;++evidence->value;evidence->stack();
  if(authority==Authority::Shadow){auto old=weightCompensatingInput(pin,flags,model);++evidence->legacyEvaluations;++evidence->shadowComparisons;
    if(!(u.array()==old.array()).all())throw std::runtime_error("Nominal input exact shadow mismatch");return old;}
  return u;
}
inline vector_t generatedNominalState(const SwitchedModelReferenceManager& ref,
    const MpcRobotModelBase<scalar_t>& model,const TargetTrajectories& target,
    const vector_t& x,scalar_t time,const std::shared_ptr<Evidence>& evidence) {
  if(!std::getenv("TASK5C_NOMINAL"))return ref.getDesiredState(target,x,time);
  auto authority=authorityFor("TASK5C_NOMINAL_STATE_AUTHORITY");
  if(authority==Authority::Legacy){++evidence->legacyEvaluations;return ref.getDesiredState(target,x,time);}
  auto raw=target.getDesiredState(time);auto& s=model.modelSettings;
  unsigned int indices[4]={static_cast<unsigned int>(s.j_l_shoulder_y_index),static_cast<unsigned int>(s.j_r_shoulder_y_index),static_cast<unsigned int>(s.j_l_elbow_y_index),static_cast<unsigned int>(s.j_r_elbow_y_index)};
  // Frozen WBMpcInterface sets armSwingReferenceActive true at construction;
  // this runner exposes no subsequent setter. Oracle covers both flag values.
  vector_t out(58);nominalState(raw.data(),x[3],ref.getPhaseVariable(time),indices,true,out.data());++evidence->generated;++evidence->value;evidence->stack();
  if(authority==Authority::Shadow){auto old=ref.getDesiredState(target,x,time);++evidence->legacyEvaluations;++evidence->shadowComparisons;
    if(!(old.array()==out.array()).all())throw std::runtime_error("Nominal state exact shadow mismatch");return old;}
  return out;
}
class GeneratedInitializer final : public Initializer {
  std::unique_ptr<Initializer> old_;
  const SwitchedModelReferenceManager* ref_;const PinocchioInterface* pin_;const MpcRobotModelBase<scalar_t>* model_;
  std::shared_ptr<Evidence> e_;
 public:
  GeneratedInitializer(const Initializer& old,WBMpcInterface& i,std::shared_ptr<Evidence> e)
      :old_(old.clone()),ref_(i.getSwitchedModelReferenceManagerPtr().get()),pin_(&i.getPinocchioInterface()),model_(&i.getMpcRobotModel()),e_(std::move(e)){}
  GeneratedInitializer(const GeneratedInitializer& p):old_(p.old_->clone()),ref_(p.ref_),pin_(p.pin_),model_(p.model_),e_(p.e_){++e_->clones;}
  GeneratedInitializer* clone() const override{return new GeneratedInitializer(*this);}
  void compute(scalar_t t,const vector_t& x,scalar_t next,vector_t& u,vector_t& nextX) override {
    ++e_->value;e_->stack();
    if(!std::getenv("TASK5C_NOMINAL") || authorityFor("TASK5C_NOMINAL_INPUT_AUTHORITY")==Authority::Legacy){++e_->legacyEvaluations;old_->compute(t,x,next,u,nextX);return;}
    u=generatedNominalInput(*pin_,ref_->getContactFlags(t),*model_,e_);nextX=x;
    if(authorityFor("TASK5C_NOMINAL_INPUT_AUTHORITY")==Authority::Shadow){vector_t oldU,oldX;old_->compute(t,x,next,oldU,oldX);
      if(!(oldU.array()==u.array()).all() || !(oldX.array()==nextX.array()).all())throw std::runtime_error("Initializer ownership shadow mismatch");u=oldU;nextX=oldX;}
  }
};
}
