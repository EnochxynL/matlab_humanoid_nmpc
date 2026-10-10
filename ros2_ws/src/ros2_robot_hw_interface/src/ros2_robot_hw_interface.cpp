#include "ros2_robot_hw_interface/ros2_robot_hw_interface.hpp"
#include <unordered_map>
#include <unordered_set>
#include <cmath>
namespace g1_bridge {
Ros2RobotHWInterface::Ros2RobotHWInterface(const std::string& urdf)
 :description_(urdf),state_(description_),action_(description_){
 if(description_.getNumJoints()!=29)throw std::runtime_error("Expected exactly 29 joints");
 for(size_t i=0;i<29;++i) ids_[i]=description_.getJointIndex(canonical_names[i]);
 state_.setTime(0.0); // Upstream constructor does not initialize time.
}
bool Ros2RobotHWInterface::consume(const StateMsg& msg,std::string& reject){
 auto fail=[&](const char* why){reject=why;return false;};
 if(msg.session_id==0)return fail("zero_session");
 if(std::find(retired_sessions_.begin(),retired_sessions_.end(),msg.session_id)!=retired_sessions_.end())return fail("retired_session");
 bool new_session=!initialized_ || msg.session_id!=latest_.session_id;
 if(!new_session && (msg.sequence<=latest_.sequence || msg.sim_time<=latest_.sim_time))return fail("state_rollback");
 if(!std::isfinite(msg.sim_time)||msg.sim_time<0)return fail("invalid_time");
 auto finite=[](const auto& a){return std::all_of(a.begin(),a.end(),[](double x){return std::isfinite(x);});};
 if(!finite(msg.base_position_world)||!finite(msg.base_orientation_wxyz)||!finite(msg.base_linear_velocity_local)||!finite(msg.base_angular_velocity_local)||!finite(msg.joint_position)||!finite(msg.joint_velocity))return fail("nonfinite_state");
 const auto& q=msg.base_orientation_wxyz;
 robot::quaternion_t quat(q[0],q[1],q[2],q[3]);
 if(std::abs(quat.norm()-1.0)>1e-6)return fail("invalid_quaternion");
 std::array<size_t,29> mapping=wire_indices_;
 if(new_session){
  if(msg.joint_names.size()!=29)return fail("joint_count");
  std::unordered_map<std::string,size_t> by_name;
  for(size_t i=0;i<29;++i){if(!by_name.emplace(msg.joint_names[i],i).second)return fail("duplicate_joint");}
  for(size_t i=0;i<29;++i){auto it=by_name.find(canonical_names[i]);if(it==by_name.end())return fail("missing_unknown_joint");mapping[i]=it->second;}
 }else if(msg.joint_names!=wire_names_)return fail("mapping_changed");
 if(new_session && initialized_)retired_sessions_.push_back(latest_.session_id);
 wire_indices_=mapping;wire_names_=msg.joint_names;latest_=msg;initialized_=true;
 state_.setTime(msg.sim_time);
 state_.setRootPositionInWorldFrame(robot::vector3_t(msg.base_position_world.data()));
 state_.setRootRotationLocalToWorldFrame(quat);
 state_.setRootLinearVelocityInLocalFrame(robot::vector3_t(msg.base_linear_velocity_local.data()));
 state_.setRootAngularVelocityInLocalFrame(robot::vector3_t(msg.base_angular_velocity_local.data()));
 for(size_t i=0;i<29;++i){state_.setJointPosition(ids_[i],msg.joint_position[mapping[i]]);state_.setJointVelocity(ids_[i],msg.joint_velocity[mapping[i]]);}
 state_.setContactFlag(0,true);state_.setContactFlag(1,true);
 return true;
}
Ros2RobotHWInterface::ActionMsg Ros2RobotHWInterface::serializeAction(uint64_t endpoint,uint64_t sequence) const{
 ActionMsg msg;msg.session_id=latest_.session_id;msg.endpoint_id=endpoint;msg.sequence=sequence;msg.state_sequence=latest_.sequence;
 msg.joint_names.assign(canonical_names.begin(),canonical_names.end());
 for(size_t i=0;i<29;++i){const auto& a=action_.at(ids_[i]).value();msg.q_des[i]=a.q_des;msg.qd_des[i]=a.qd_des;msg.kp[i]=a.kp;msg.kd[i]=a.kd;msg.tau_ff[i]=a.feed_forward_effort;}
 return msg;
}
Ros2RobotHWInterface::ObservationMsg Ros2RobotHWInterface::observation(double received,double sent) const{
 ObservationMsg observation;auto& msg=observation.state;msg=latest_;
 msg.sim_time=state_.getTime();msg.joint_names.assign(canonical_names.begin(),canonical_names.end());
 auto p=state_.getRootPositionInWorldFrame();auto q=state_.getRootRotationLocalToWorldFrame();auto v=state_.getRootLinearVelocityInLocalFrame();auto w=state_.getRootAngularVelocityInLocalFrame();
 for(size_t i=0;i<3;++i){msg.base_position_world[i]=p[i];msg.base_linear_velocity_local[i]=v[i];msg.base_angular_velocity_local[i]=w[i];}
 msg.base_orientation_wxyz={q.w(),q.x(),q.y(),q.z()};
 for(size_t i=0;i<29;++i){msg.joint_position[i]=state_.getJointPosition(ids_[i]);msg.joint_velocity[i]=state_.getJointVelocity(ids_[i]);}
 observation.controller_contact_flags={state_.getContactFlag(0),state_.getContactFlag(1)};
 observation.receive_wall_time=received;observation.send_wall_time=sent;return observation;
}
}
