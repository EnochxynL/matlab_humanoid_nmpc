/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

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

#include "data/observation/ObservationConversion.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

namespace g1_controller::observation {
using namespace ocs2;
using namespace ocs2::humanoid;
std::array<double, 74> encodeObservation(const Observation &o) {
  std::array<double, 74> a{};
  a[0] = o.rawBackendTime;
  std::copy(o.basePosition.begin(), o.basePosition.end(), a.begin() + 1);
  std::copy(o.baseOrientationWxyz.begin(), o.baseOrientationWxyz.end(),
            a.begin() + 4);
  std::copy(o.baseLinearVelocityLocal.begin(), o.baseLinearVelocityLocal.end(),
            a.begin() + 8);
  std::copy(o.baseAngularVelocityLocal.begin(),
            o.baseAngularVelocityLocal.end(), a.begin() + 11);
  std::copy(o.jointPositions.begin(), o.jointPositions.end(), a.begin() + 14);
  std::copy(o.jointVelocities.begin(), o.jointVelocities.end(), a.begin() + 43);
  std::copy(o.measuredContacts.begin(), o.measuredContacts.end(),
            a.begin() + 72);
  return a;
}
void fillRobotState(robot::model::RobotState& state, const double* a,
                    const std::vector<size_t>& jointIndices, double t) {
    state.setTime(t);
    state.setRootPositionInWorldFrame(robot::vector3_t(a + 1));
    robot::quaternion_t q(a[4], a[5], a[6], a[7]);
    if (std::abs(q.norm() - 1) > 1e-6)
      throw std::runtime_error("Quaternion rejected");
    state.setRootRotationLocalToWorldFrame(q);
    state.setRootLinearVelocityInLocalFrame(robot::vector3_t(a + 8));
    state.setRootAngularVelocityInLocalFrame(robot::vector3_t(a + 11));
    for (size_t j = 0; j < 29; ++j) {
      state.setJointPosition(jointIndices[j], a[14 + j]);
      state.setJointVelocity(jointIndices[j], a[43 + j]);
    }
    // Original successful controller uses STANCE for measured observation mode.
    state.setContactFlag(0, true);
    state.setContactFlag(1, true);
}

/** @brief 保留原状态转换：姿态 ZYX、世界系线速度、Euler 导数和 MPC 关节顺序。 */
void fillMpcState(vector_t& mpcState, const ::robot::model::RobotState& robotState,
                  const WBAccelMpcRobotModel<scalar_t>& mpcRobotModel_,
                  const std::vector<size_t>& mpcJointIndices_) {
  mpcRobotModel_.setBasePosition(mpcState, robotState.getRootPositionInWorldFrame());
  mpcRobotModel_.setBaseOrientationEulerZYX(mpcState, quaternionToEulerZYX(robotState.getRootRotationLocalToWorldFrame()));

  mpcRobotModel_.setJointAngles(mpcState, robotState.getJointPositions(mpcJointIndices_));

  // currently we send local angular and linear velocity
  mpcRobotModel_.setBaseLinearVelocity(mpcState,
                                       robotState.getRootRotationLocalToWorldFrame() * robotState.getRootLinearVelocityInLocalFrame());
  mpcRobotModel_.setBaseOrientationEulerZYXDerivatives(
      mpcState, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(mpcRobotModel_.getBaseOrientationEulerZYX(mpcState),
                                                                               robotState.getRootAngularVelocityInLocalFrame()));

  vector_t dummyInput = vector_t::Zero(mpcRobotModel_.getInputDim());
  mpcRobotModel_.setJointVelocities(mpcState, dummyInput, robotState.getJointVelocities(mpcJointIndices_));

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 组装 OCS2 观测：转换状态、写入物理时间、清零输入并编码接触模式。
 * 这里的零输入是观测初始化字段，不是向机器人发布零力矩。
 */
void fillMpcObservation(ocs2::SystemObservation& mpcObservation, const ::robot::model::RobotState& robotState,
                        const WBAccelMpcRobotModel<scalar_t>& mpcRobotModel_,
                        const std::vector<size_t>& mpcJointIndices_) {
  fillMpcState(mpcObservation.state, robotState, mpcRobotModel_, mpcJointIndices_);
  mpcObservation.time = robotState.getTime();
  mpcObservation.input = vector_t::Zero(mpcRobotModel_.getInputDim());  // Add contact forces later.
  std::vector<bool> configContacts = robotState.getContactFlags();
  assert(configContacts.size() == 2);
  contact_flag_t contactFlags;
  std::copy(configContacts.begin(), configContacts.end(), contactFlags.begin());
  mpcObservation.mode = stanceLeg2ModeNumber(contactFlags);
}

} // namespace g1_controller::observation
