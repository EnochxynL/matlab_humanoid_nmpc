#pragma once
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include "reference/ControlTimeReferenceFilter.h"
namespace ocs2::humanoid {
class ControlTimeMotionManager final : public ProceduralMpcMotionManager {
 public:
  using ProceduralMpcMotionManager::ProceduralMpcMotionManager;
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                    const ReferenceManagerInterface& referenceManager) override;
 private:
  ControlTimeReferenceFilter controlFilter_{5, vector4_t::Zero()};
  bool filterInitialized_ = false;
};
}
