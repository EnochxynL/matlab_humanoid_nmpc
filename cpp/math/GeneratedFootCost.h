#pragma once
#include "math/GeneratedConstraint.h"
#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <humanoid_wb_mpc/cost/EndEffectorDynamicsFootCost.h>
namespace task5c::footcost {
using namespace ocs2;using namespace ocs2::humanoid;
class GeneratedFootCost final:public StateInputCost {
 std::unique_ptr<EndEffectorDynamicsFootCost> original_;std::unique_ptr<StateInputCost> graph_;Authority auth_;std::shared_ptr<Evidence> evidence_;
 public:
 GeneratedFootCost(EndEffectorDynamicsFootCost&,WBMpcInterface&,int,const std::string&,Authority,std::shared_ptr<Evidence>);
 GeneratedFootCost(const GeneratedFootCost&);
 GeneratedFootCost*clone()const override;
 bool isActive(double)const override;
 scalar_t getValue(double,const vector_t&,const vector_t&,const TargetTrajectories&,const PreComputation&)const override;
 ScalarFunctionQuadraticApproximation getQuadraticApproximation(double,const vector_t&,const vector_t&,const TargetTrajectories&,const PreComputation&)const override;
};
OptimalControlProblem installFootCosts(OptimalControlProblem,WBMpcInterface&,std::shared_ptr<Evidence>);
}
