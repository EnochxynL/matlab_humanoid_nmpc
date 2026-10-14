/** @file 观测编码、机器人状态装填及 OCP 坐标转换；不管理求解器或线程。 */
#pragma once
#include "g1_controller/Types.h"
#include "data/observation/RobotState.h"
#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>
#include <ocs2_mpc/SystemObservation.h>

namespace g1_controller::observation {
std::array<double, 74> encodeObservation(const Observation& observation);
// raw 为已验证的74元素兼容观测；保留双足测量模式及既有四元数门禁。
void fillRobotState(robot::model::RobotState& state, const double* raw,
                    const std::vector<size_t>& jointIndices, double canonicalTime);
void fillMpcState(ocs2::vector_t& result, const robot::model::RobotState& state,
                  const ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t>& model,
                  const std::vector<size_t>& jointIndices);
void fillMpcObservation(ocs2::SystemObservation& result, const robot::model::RobotState& state,
                        const ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t>& model,
                        const std::vector<size_t>& jointIndices);
} // namespace g1_controller::observation
