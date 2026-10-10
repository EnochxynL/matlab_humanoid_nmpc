#pragma once
#include "math/GeneratedConstraint.h"
namespace task5c::footcost {
OptimalControlProblem installMomentBridge(OptimalControlProblem,WBMpcInterface&,const std::string&,std::shared_ptr<Evidence>);
OptimalControlProblem installCollisionBridge(OptimalControlProblem,WBMpcInterface&,const std::string&,std::shared_ptr<Evidence>);
}
