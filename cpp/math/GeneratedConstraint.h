#pragma once
#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <ocs2_core/constraint/StateInputConstraintCollection.h>
#include "zeroWrench.h"
#include <atomic>
#include <fstream>
#include <execinfo.h>
namespace task5c {
using namespace ocs2;using namespace ocs2::humanoid;
enum class Authority { Legacy,Shadow,Active };
inline Authority authority(){const char*v=std::getenv("TASK5C_AUTHORITY");std::string s=v?v:"LEGACY_CPP";if(s=="LEGACY_CPP")return Authority::Legacy;if(s=="MATLAB_SHADOW")return Authority::Shadow;if(s=="MATLAB_ACTIVE")return Authority::Active;throw std::runtime_error("Invalid mathematical authority");}
inline Authority authorityFor(const char* key){const char*v=std::getenv(key);if(!v)return authority();std::string s=v;if(s=="LEGACY_CPP")return Authority::Legacy;if(s=="MATLAB_SHADOW")return Authority::Shadow;if(s=="MATLAB_ACTIVE")return Authority::Active;throw std::runtime_error("Invalid unit authority");}
struct Evidence {
 std::string folder,prefix;std::atomic<uint64_t>activation{0},value{0},linear{0},generated{0},legacyEvaluations{0},shadowComparisons{0},clones{0};std::atomic<bool>stackWritten{false};
 explicit Evidence(std::string f,std::string p=""):folder(std::move(f)),prefix(std::move(p)){}
 void stack(){if(stackWritten.exchange(true))return;void*frames[40];int n=backtrace(frames,40);char**strings=backtrace_symbols(frames,n);std::ofstream o(folder+"/"+prefix+"active_call_stack.txt");for(int i=0;i<n;++i)o<<strings[i]<<'\n';free(strings);}
 ~Evidence(){std::ofstream(folder+"/"+prefix+"authority_calls.json")<<"{\"activation\":"<<activation<<",\"value\":"<<value<<",\"linear\":"<<linear<<",\"generated\":"<<generated<<",\"legacy_evaluations\":"<<legacyEvaluations<<",\"shadow_comparisons\":"<<shadowComparisons<<",\"clones\":"<<clones<<"}\n";}
};
class GeneratedZeroWrench final:public StateInputConstraint {
 const SwitchedModelReferenceManager*ref_;size_t contact_;Authority authority_;std::shared_ptr<Evidence>evidence_;std::unique_ptr<StateInputConstraint>legacy_;
 VectorFunctionLinearApproximation candidate(scalar_t t,const vector_t&u,bool&active)const{
  if(u.size()!=35)throw std::runtime_error("ZeroWrench input35 contract");auto flags=ref_->getContactFlags(t);boolean_T f[2]={flags[0],flags[1]};VectorFunctionLinearApproximation a;
  a.f.resize(6);a.dfdx.resize(6,58);a.dfdu.resize(6,35);bool enabled=legacy_->getActive();
  boolean_T generatedActive;zeroWrench(u.data(),static_cast<unsigned char>(contact_),f,enabled,&generatedActive,a.f.data(),a.dfdx.data(),a.dfdu.data());active=generatedActive;++evidence_->generated;return a;
 }
 public:
 GeneratedZeroWrench(const SwitchedModelReferenceManager&r,size_t c,Authority a,std::shared_ptr<Evidence>e,std::unique_ptr<StateInputConstraint>old):StateInputConstraint(ConstraintOrder::Linear),ref_(&r),contact_(c),authority_(a),evidence_(std::move(e)),legacy_(std::move(old)){}
 GeneratedZeroWrench(const GeneratedZeroWrench&o):StateInputConstraint(o),ref_(o.ref_),contact_(o.contact_),authority_(o.authority_),evidence_(o.evidence_),legacy_(o.legacy_->clone()){++evidence_->clones;}
 GeneratedZeroWrench*clone()const override{return new GeneratedZeroWrench(*this);}
 void setActive(bool v)override{legacy_->setActive(v);}bool getActive()const override{return legacy_->getActive();}
 size_t getNumConstraints(scalar_t)const override{return 6;}
 bool isActive(scalar_t t)const override{
  ++evidence_->activation;if(authority_==Authority::Legacy)return legacy_->isActive(t);
  // The generated implementation owns Boolean activation; original schedule owns time lookup.
  vector_t u=vector_t::Zero(35);bool active;candidate(t,u,active);
  if(authority_==Authority::Shadow){if(active!=legacy_->isActive(t))throw std::runtime_error("Activation shadow mismatch");++evidence_->shadowComparisons;return legacy_->isActive(t);}return active;
 }
 vector_t getValue(scalar_t t,const vector_t&x,const vector_t&u,const PreComputation&p)const override{
  ++evidence_->value;if(authority_==Authority::Legacy){++evidence_->legacyEvaluations;return legacy_->getValue(t,x,u,p);}bool active;auto a=candidate(t,u,active);
  if(authority_==Authority::Shadow){auto old=legacy_->getValue(t,x,u,p);++evidence_->legacyEvaluations;++evidence_->shadowComparisons;if(!(a.f.array()==old.array()).all())throw std::runtime_error("Value shadow mismatch");return old;}return a.f;
 }
 VectorFunctionLinearApproximation getLinearApproximation(scalar_t t,const vector_t&x,const vector_t&u,const PreComputation&p)const override{
  ++evidence_->linear;evidence_->stack();if(authority_==Authority::Legacy){++evidence_->legacyEvaluations;return legacy_->getLinearApproximation(t,x,u,p);}bool active;auto a=candidate(t,u,active);
  if(authority_==Authority::Shadow){auto old=legacy_->getLinearApproximation(t,x,u,p);++evidence_->legacyEvaluations;++evidence_->shadowComparisons;if(!(a.f.array()==old.f.array()).all()||!(a.dfdx.array()==old.dfdx.array()).all()||!(a.dfdu.array()==old.dfdu.array()).all())throw std::runtime_error("Linear shadow mismatch");return old;}return a;
 }
};
inline OptimalControlProblem install(WBMpcInterface&i,std::shared_ptr<Evidence>e){
 OptimalControlProblem p=i.getOptimalControlProblem();auto names=p.equalityConstraintPtr->getTermNameMap();std::vector<std::pair<size_t,std::string>>ordered;for(auto&[name,index]:names)ordered.emplace_back(index,name);std::sort(ordered.begin(),ordered.end());
 auto replacement=std::make_unique<StateInputConstraintCollection>();for(auto&[index,name]:ordered){auto term=p.equalityConstraintPtr->extract(name);if(name=="foot_l_contact_zeroWrench"||name=="foot_r_contact_zeroWrench")term=std::make_unique<GeneratedZeroWrench>(*i.getSwitchedModelReferenceManagerPtr(),name=="foot_l_contact_zeroWrench"?0:1,authorityFor("TASK5C_ZERO_AUTHORITY"),e,std::move(term));replacement->add(name,std::move(term));}
 p.equalityConstraintPtr=std::move(replacement);return p;
}
}
