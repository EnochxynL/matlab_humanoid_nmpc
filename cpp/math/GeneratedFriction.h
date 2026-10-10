#pragma once
#include "math/GeneratedConstraint.h"
#include "frictionCone.h"
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/cost/StateInputCostCollection.h>
namespace task5c {
class GeneratedFriction final:public StateInputConstraint {
 const SwitchedModelReferenceManager*ref_;size_t contact_;Authority authority_;std::shared_ptr<Evidence>evidence_;std::unique_ptr<StateInputConstraint>legacy_;std::array<double,4>config_;
 VectorFunctionQuadraticApproximation candidate(scalar_t t,const vector_t&u,bool&active)const{
  if(u.size()!=35)throw std::runtime_error("Friction input35");auto flags=ref_->getContactFlags(t);boolean_T f[2]={flags[0],flags[1]},enabled=legacy_->getActive(),generatedActive;double value;struct0_T a;
  frictionCone(u.data(),static_cast<unsigned char>(contact_),f,enabled,config_.data(),&generatedActive,&value,&a);active=generatedActive;++evidence_->generated;
  VectorFunctionQuadraticApproximation out;out.f=vector_t::Constant(1,value);out.dfdx=Eigen::Map<matrix_t>(a.dfdx,1,58);out.dfdu=Eigen::Map<matrix_t>(a.dfdu,1,35);out.dfdxx.emplace_back(Eigen::Map<matrix_t>(a.dfdxx,58,58));out.dfduu.emplace_back(Eigen::Map<matrix_t>(a.dfduu,35,35));out.dfdux.emplace_back(Eigen::Map<matrix_t>(a.dfdux,35,58));return out;
 }
 static bool exact(const matrix_t&a,const matrix_t&b){return a.rows()==b.rows()&&a.cols()==b.cols()&&(a.array()==b.array()).all();}
 public:
 GeneratedFriction(const SwitchedModelReferenceManager&r,size_t c,Authority a,std::shared_ptr<Evidence>e,std::unique_ptr<StateInputConstraint>old,std::array<double,4>cfg):StateInputConstraint(ConstraintOrder::Quadratic),ref_(&r),contact_(c),authority_(a),evidence_(std::move(e)),legacy_(std::move(old)),config_(cfg){}
 GeneratedFriction(const GeneratedFriction&o):StateInputConstraint(o),ref_(o.ref_),contact_(o.contact_),authority_(o.authority_),evidence_(o.evidence_),legacy_(o.legacy_->clone()),config_(o.config_){++evidence_->clones;}
 GeneratedFriction*clone()const override{return new GeneratedFriction(*this);}
 void setActive(bool v)override{legacy_->setActive(v);}bool getActive()const override{return legacy_->getActive();}size_t getNumConstraints(scalar_t)const override{return 1;}
 bool isActive(scalar_t t)const override{++evidence_->activation;if(authority_==Authority::Legacy)return legacy_->isActive(t);vector_t u=vector_t::Zero(35);bool active;candidate(t,u,active);if(authority_==Authority::Shadow){if(active!=legacy_->isActive(t))throw std::runtime_error("Friction activation mismatch");++evidence_->shadowComparisons;return legacy_->isActive(t);}return active;}
 vector_t getValue(scalar_t t,const vector_t&x,const vector_t&u,const PreComputation&p)const override{++evidence_->value;if(authority_==Authority::Legacy){++evidence_->legacyEvaluations;return legacy_->getValue(t,x,u,p);}bool active;auto a=candidate(t,u,active);if(authority_==Authority::Shadow){auto old=legacy_->getValue(t,x,u,p);++evidence_->legacyEvaluations;++evidence_->shadowComparisons;if(!exact(a.f,old))throw std::runtime_error("Friction value mismatch");return old;}return a.f;}
 VectorFunctionLinearApproximation getLinearApproximation(scalar_t t,const vector_t&x,const vector_t&u,const PreComputation&p)const override{++evidence_->linear;if(authority_==Authority::Legacy){++evidence_->legacyEvaluations;return legacy_->getLinearApproximation(t,x,u,p);}bool active;auto a=candidate(t,u,active);VectorFunctionLinearApproximation out;out.f=a.f;out.dfdx=a.dfdx;out.dfdu=a.dfdu;if(authority_==Authority::Shadow){auto old=legacy_->getLinearApproximation(t,x,u,p);++evidence_->legacyEvaluations;++evidence_->shadowComparisons;if(!exact(out.f,old.f)||!exact(out.dfdx,old.dfdx)||!exact(out.dfdu,old.dfdu))throw std::runtime_error("Friction linear mismatch");return old;}return out;}
 VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t,const vector_t&x,const vector_t&u,const PreComputation&p)const override{++evidence_->linear;evidence_->stack();if(authority_==Authority::Legacy){++evidence_->legacyEvaluations;return legacy_->getQuadraticApproximation(t,x,u,p);}bool active;auto a=candidate(t,u,active);if(authority_==Authority::Shadow){auto old=legacy_->getQuadraticApproximation(t,x,u,p);++evidence_->legacyEvaluations;++evidence_->shadowComparisons;if(!exact(a.f,old.f)||!exact(a.dfdx,old.dfdx)||!exact(a.dfdu,old.dfdu)||!exact(a.dfdxx[0],old.dfdxx[0])||!exact(a.dfduu[0],old.dfduu[0])||!exact(a.dfdux[0],old.dfdux[0]))throw std::runtime_error("Friction quadratic mismatch");return old;}return a;}
};
inline OptimalControlProblem installFriction(OptimalControlProblem p,WBMpcInterface&i,const std::string&task,std::shared_ptr<Evidence>e){
 if(!std::getenv("TASK5C_FRICTION"))return p;boost::property_tree::ptree tree;boost::property_tree::read_info(task,tree);double mu=tree.get<double>("contacts.frictionForceConeSoftConstraint.frictionCoefficient");
 for(size_t c=0;c<2;++c){auto&soft=p.softConstraintPtr->get<StateInputSoftConstraint>(c?"foot_r_contact_frictionForceCone":"foot_l_contact_frictionForceCone");auto original=std::move(soft.getConstraintPtr());soft.getConstraintPtr()=std::make_unique<GeneratedFriction>(*i.getSwitchedModelReferenceManagerPtr(),c,authorityFor("TASK5C_FRICTION_AUTHORITY"),e,std::move(original),std::array<double,4>{mu,25,0,1e-6});}return p;
}
}
