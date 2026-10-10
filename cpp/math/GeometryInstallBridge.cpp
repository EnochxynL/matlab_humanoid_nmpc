#include "math/GeometryInstallBridge.h"
#include "math/GeneratedCollisionCost.h"
namespace task5c::footcost {
OptimalControlProblem installMomentBridge(OptimalControlProblem p,WBMpcInterface&i,const std::string&t,std::shared_ptr<Evidence>e){return task5c::momentfused::installMoment(std::move(p),i,t,e);}
OptimalControlProblem installCollisionBridge(OptimalControlProblem p,WBMpcInterface&i,const std::string&t,std::shared_ptr<Evidence>e){return task5c::collision::installCollision(std::move(p),i,t,e);}
}
