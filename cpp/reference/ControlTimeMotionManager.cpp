/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "reference/ControlTimeMotionManager.h"
#include <humanoid_common_mpc/gait/GaitScheduleUpdater.h>
namespace ocs2::humanoid {
void ControlTimeMotionManager::preSolverRun(scalar_t initTime,
                                              scalar_t finalTime,
                                              const vector_t& initState,
                                              const ReferenceManagerInterface& referenceManager) {
  WalkingVelocityCommand incommingVelCommand = getScaledWalkingVelocityCommand();
  if (!filterInitialized_) { controlFilter_.reset(incommingVelCommand.toVector(), initTime); filterInitialized_ = true; }
  vector4_t filteredVelCommand = controlFilter_.getFilteredVector(incommingVelCommand.toVector(), initTime);

  // Update TargetTrajectories
  TargetTrajectories targetTrajectories = velocityTargetToTargetTrajectoriesFun_(filteredVelCommand, initTime, finalTime, initState);
  switchedModelReferenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  static GaitModeStateConfig currentCfg = gaitModeStates_[currentGaitMode_];
  vector6_t baseVelocity = mpcRobotModelPtr_->getBaseComVelocity(initState);

  // Do not change the gait pattern for at least 0.5s
  if (initTime > lastGaitChangeTime_ + 0.2) {
    if (transitionToFasterGait(filteredVelCommand, baseVelocity, currentCfg)) {
      std::cout << "filteredVelCommand: " << filteredVelCommand.transpose() << std::endl;
      std::cout << "Linear limits: " << currentCfg.minLinVelCmd << ", " << currentCfg.maxLinVelCmd << std::endl;
      currentGaitMode_++;
      currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      std::cout << "ProceduralMpcMotionManager: Increasing to gait:" << currentCfg.gaitCommand << std::endl;
      lastGaitChangeTime_ = initTime;
    } else if (transitionToSlowerGait(filteredVelCommand, baseVelocity, currentCfg)) {
      std::cout << "filteredVelCommand: " << filteredVelCommand.transpose() << std::endl;
      std::cout << "Linear limits: " << currentCfg.minLinVelCmd << ", " << currentCfg.maxLinVelCmd << std::endl;
      currentGaitMode_--;
      currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      std::cout << "ProceduralMpcMotionManager: Decreasing to gait:" << currentCfg.gaitCommand << std::endl;
      lastGaitChangeTime_ = initTime;
    }
  }

  if (currentGaitCommand_ != lastGaitCommand_) {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);

    GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
    lastGaitCommand_ = currentGaitCommand_;
  }
}

}
