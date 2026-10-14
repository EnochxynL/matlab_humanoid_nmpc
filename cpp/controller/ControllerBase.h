#pragma once

#include <data/action/RobotJointAction.h>
#include <data/observation/RobotState.h>

namespace g1_controller {

class ControlBase {
 public:
  virtual ~ControlBase() = default;

  virtual bool ready() const = 0;

  // Computes [commands] given [joints] states and configs.
  virtual void computeJointControlAction(::robot::scalar_t time,
                                         const ::robot::model::RobotState& robotState,
                                         ::robot::model::RobotJointAction& robotJointAction) = 0;
};

}  // namespace g1_controller