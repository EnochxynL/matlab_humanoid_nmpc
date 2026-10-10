#pragma once
#include <ranges>
#include <robot_model/RobotState.h>
#include <robot_model/RobotJointAction.h>
#include <g1_plant_bridge_msgs/msg/plant_state.hpp>
#include <g1_plant_bridge_msgs/msg/robot_joint_action.hpp>
#include <g1_plant_bridge_msgs/msg/bridge_observation.hpp>
#include <rclcpp/rclcpp.hpp>
#include "canonical.hpp"
namespace g1_bridge {
// Composition preserves the real upstream structures. No PD runs here.
class Ros2RobotHWInterface {
 public:
  using StateMsg=g1_plant_bridge_msgs::msg::PlantState;
  using ActionMsg=g1_plant_bridge_msgs::msg::RobotJointAction;
  using ObservationMsg=g1_plant_bridge_msgs::msg::BridgeObservation;
  explicit Ros2RobotHWInterface(const std::string& urdf);
  bool consume(const StateMsg& msg,std::string& rejection);
  const robot::model::RobotDescription& getRobotDescription() const {return description_;}
  const robot::model::RobotState& getRobotState() const {return state_;}
  robot::model::RobotJointAction& getRobotJointAction(){return action_;}
  ActionMsg serializeAction(uint64_t endpoint,uint64_t sequence) const;
  ObservationMsg observation(double received,double sent) const;
  const std::array<size_t,29>& jointIds() const{return ids_;}
 private:
  robot::model::RobotDescription description_;
  robot::model::RobotState state_;
  robot::model::RobotJointAction action_;
  std::array<size_t,29> ids_{},wire_indices_{};
  std::vector<std::string> wire_names_;
  StateMsg latest_;
  bool initialized_=false;
  std::vector<uint64_t> retired_sessions_;
};
}
