// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "franka_semantic_components/franka_robot_state.hpp"

#include <cstring>
#include <optional>
#include <stack>

#include <realtime_tools/realtime_buffer.hpp>
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "translation_utils.hpp"

namespace {

constexpr size_t kBaseLinkIndex = 0;
constexpr size_t kFlangeLinkIndex = 8;
constexpr size_t kLoadLinkIndex = 8;
const std::string kTCPFrameName = "_hand_tcp";

// Example implementation of bit_cast: https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable<From>::value &&
                     std::is_trivially_copyable<To>::value,
                 To>
bit_cast(const From& src) noexcept {
  static_assert(std::is_trivially_constructible<To>::value,
                "This implementation additionally requires "
                "destination type to be trivially constructible");

  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

}  // anonymous namespace

namespace franka_semantic_components {

FrankaRobotState::FrankaRobotState(const std::string& name, const std::string& robot_description)
    : SemanticComponentInterface(name, 1), model_(std::make_shared<urdf::Model>()) {
  robot_description_ = robot_description;
  if (!model_->initString(robot_description_)) {
    throw std::runtime_error("Failed to parse URDF.");
  }

  if (name.empty()) {
    robot_name_ = get_robot_name_from_urdf();
  } else {
    size_t slash_pos = name.find('/');
    if (slash_pos != std::string::npos) {
      robot_name_ = name.substr(0, slash_pos);
    } else {
      robot_name_ = name;
    }
  }
  full_robot_state_interface_name_ = robot_name_ + "/" + state_interface_name_;
  interface_names_.emplace_back(full_robot_state_interface_name_);

  gripper_loaded_ = is_gripper_loaded();

  set_links_from_urdf();
  set_joints_from_urdf();

  if (gripper_loaded_) {
    kEndEffectorLinkIndex = get_link_index(robot_name_ + kTCPFrameName);
    kStiffnessLinkIndex = kEndEffectorLinkIndex;
  } else {
    kEndEffectorLinkIndex = kFlangeLinkIndex;
    kStiffnessLinkIndex = kEndEffectorLinkIndex;
  }
}

auto FrankaRobotState::get_link_index(const std::string& link_name) -> size_t {
  auto link_index = std::find(link_names.cbegin(), link_names.cend(), link_name);
  if (link_index != link_names.end()) {
    return std::distance(link_names.cbegin(), link_index);
  } else {
    throw std::runtime_error("Link name not found in URDF.");
  }
}

auto FrankaRobotState::is_gripper_loaded() -> bool {
  const auto& links = model_->links_;
  bool gripper_loaded = links.find(robot_name_ + kTCPFrameName) != links.end();

  return gripper_loaded;
}

auto FrankaRobotState::get_robot_name_from_urdf() -> std::string {
  return model_->name_;
}

void FrankaRobotState::set_child_links(const std::shared_ptr<const urdf::Link>& link) {
  // Create a stack and push the root node
  std::stack<std::shared_ptr<const urdf::Link>> stack;
  stack.push(link);

  // Iterate while the stack is not empty
  while (!stack.empty()) {
    // Pop a link from the stack and add its name to link_names
    std::shared_ptr<const urdf::Link> current_link = stack.top();
    stack.pop();
    link_names.push_back(current_link->name);

    // Push the children of the current link to the stack
    for (const auto& child_link : current_link->child_links) {
      stack.push(child_link);
    }
  }
}

auto FrankaRobotState::set_links_from_urdf() -> void {
  auto root_link = model_->getRoot();
  link_names.push_back(root_link->name);
  set_child_links(root_link);
}

auto FrankaRobotState::set_joints_from_urdf() -> void {
  auto& joints = model_->joints_;
  for (const auto& [name, joint] : joints) {
    if (joint->type == urdf::Joint::REVOLUTE) {
      joint_names.push_back(name);
    }
  }
}

auto FrankaRobotState::initialize_robot_state_msg(franka_msgs::msg::FrankaRobotState& message)
    -> void {
  // The joint state - joint 1 is the first joint while joint 7 is the last revolute joint
  message.measured_joint_state.name =
      std::vector<std::string>(joint_names.cbegin(), joint_names.cend());
  message.desired_joint_state.name =
      std::vector<std::string>(joint_names.cbegin(), joint_names.cend());
  message.measured_joint_motor_state.name =
      std::vector<std::string>(joint_names.cbegin(), joint_names.cend());
  message.tau_ext_hat_filtered.name =
      std::vector<std::string>(joint_names.cbegin(), joint_names.cend());

  message.measured_joint_state.header.frame_id = link_names[kBaseLinkIndex];
  message.desired_joint_state.header.frame_id = link_names[kBaseLinkIndex];
  message.measured_joint_motor_state.header.frame_id = link_names[kBaseLinkIndex];
  message.tau_ext_hat_filtered.header.frame_id = link_names[kBaseLinkIndex];

  // Active wrenches
  message.o_f_ext_hat_k.header.frame_id = link_names[kBaseLinkIndex];
  message.k_f_ext_hat_k.header.frame_id = link_names[kStiffnessLinkIndex];

  // Current EE Pose
  message.o_t_ee.header.frame_id = link_names[kBaseLinkIndex];
  // Desired EE Pose
  message.o_t_ee_d.header.frame_id = link_names[kBaseLinkIndex];
  // Commanded EE Pose
  message.o_t_ee_c.header.frame_id = link_names[kBaseLinkIndex];

  message.f_t_ee.header.frame_id = link_names[kFlangeLinkIndex];
  message.ee_t_k.header.frame_id = link_names[kEndEffectorLinkIndex];

  message.o_dp_ee_d.header.frame_id = link_names[kBaseLinkIndex];
  message.o_dp_ee_c.header.frame_id = link_names[kBaseLinkIndex];
  message.o_ddp_ee_c.header.frame_id = link_names[kBaseLinkIndex];

  // The inertias are with respect to the Center Of Mass.
  // TODO(yazi_ba) frame ids should be referenced to the Center Of Mass
  message.inertia_ee.header.frame_id = link_names[kEndEffectorLinkIndex];
  message.inertia_load.header.frame_id = link_names[kLoadLinkIndex];
  message.inertia_total.header.frame_id = link_names[kEndEffectorLinkIndex];

  // Resize dynamic vectors
  message.measured_joint_state.position.resize(joint_names.size(), 0.0);
  message.measured_joint_state.velocity.resize(joint_names.size(), 0.0);
  message.measured_joint_state.effort.resize(joint_names.size(), 0.0);

  message.desired_joint_state.position.resize(joint_names.size(), 0.0);
  message.desired_joint_state.velocity.resize(joint_names.size(), 0.0);
  message.desired_joint_state.effort.resize(joint_names.size(), 0.0);

  message.measured_joint_motor_state.position.resize(joint_names.size(), 0.0);
  message.measured_joint_motor_state.velocity.resize(joint_names.size(), 0.0);
  message.measured_joint_motor_state.effort.resize(joint_names.size(), 0.0);

  message.tau_ext_hat_filtered.position.resize(joint_names.size(), 0.0);
  message.tau_ext_hat_filtered.velocity.resize(joint_names.size(), 0.0);
  message.tau_ext_hat_filtered.effort.resize(joint_names.size(), 0.0);
}

auto FrankaRobotState::get_values_as_message(franka_msgs::msg::FrankaRobotState& message) -> bool {
  auto franka_state_interface = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(), [&](const auto& interface) {
        return interface.get().get_name() == full_robot_state_interface_name_;
      });
  if (franka_state_interface != state_interfaces_.end()) {
    auto* buffer_ptr = bit_cast<realtime_tools::RealtimeBuffer<franka::RobotState>*>(
        (*franka_state_interface).get().get_optional().value());
    robot_state_ptr = buffer_ptr->readFromRT();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("franka_state_semantic_component"),
                 "Franka state interface does not exist! Did you assign the loaned state in the "
                 "controller?");
    return false;
  }

  // Update the time stamps of the data
  translation::updateTimeStamps(message.header.stamp, message);

  // Collision and contact indicators
  message.collision_indicators = translation::toCollisionIndicators(
      robot_state_ptr->cartesian_collision, robot_state_ptr->cartesian_contact,
      robot_state_ptr->joint_collision, robot_state_ptr->joint_contact);

  // The joint states
  const auto n_joints = message.measured_joint_state.position.size();
  std::copy_n(robot_state_ptr->q.cbegin(), n_joints, message.measured_joint_state.position.begin());
  std::copy_n(robot_state_ptr->dq.cbegin(), n_joints,
              message.measured_joint_state.velocity.begin());
  std::copy_n(robot_state_ptr->tau_J.cbegin(), n_joints,
              message.measured_joint_state.effort.begin());

  std::copy_n(robot_state_ptr->q_d.cbegin(), n_joints,
              message.desired_joint_state.position.begin());
  std::copy_n(robot_state_ptr->dq_d.cbegin(), n_joints,
              message.desired_joint_state.velocity.begin());
  std::copy_n(robot_state_ptr->tau_J_d.cbegin(), n_joints,
              message.desired_joint_state.effort.begin());

  std::copy_n(robot_state_ptr->theta.cbegin(), n_joints,
              message.measured_joint_motor_state.position.begin());
  std::copy_n(robot_state_ptr->dtheta.cbegin(), n_joints,
              message.measured_joint_motor_state.velocity.begin());

  std::copy_n(robot_state_ptr->tau_ext_hat_filtered.cbegin(), n_joints,
              message.tau_ext_hat_filtered.effort.begin());

  message.ddq_d = robot_state_ptr->ddq_d;
  message.dtau_j = robot_state_ptr->dtau_J;

  // Output for the elbow
  message.elbow = translation::toElbow(robot_state_ptr->elbow, robot_state_ptr->elbow_d,
                                       robot_state_ptr->elbow_c, robot_state_ptr->delbow_c,
                                       robot_state_ptr->ddelbow_c);

  // Active wrenches on the stiffness frame
  message.k_f_ext_hat_k.wrench = translation::toWrench(robot_state_ptr->K_F_ext_hat_K);
  message.o_f_ext_hat_k.wrench = translation::toWrench(robot_state_ptr->O_F_ext_hat_K);

  // The transformations between different frames
  message.o_t_ee.pose = translation::toPose(robot_state_ptr->O_T_EE);
  message.o_t_ee_d.pose = translation::toPose(robot_state_ptr->O_T_EE_d);
  message.o_t_ee_c.pose = translation::toPose(robot_state_ptr->O_T_EE_c);

  message.f_t_ee.pose = translation::toPose(robot_state_ptr->F_T_EE);
  message.ee_t_k.pose = translation::toPose(robot_state_ptr->EE_T_K);

  message.o_dp_ee_d.twist = translation::toTwist(robot_state_ptr->O_dP_EE_d);
  message.o_dp_ee_c.twist = translation::toTwist(robot_state_ptr->O_dP_EE_c);
  message.o_ddp_ee_c.accel = translation::toAccel(robot_state_ptr->O_ddP_EE_c);

  // The inertias of the robot
  message.inertia_ee.inertia = translation::toInertia(
      robot_state_ptr->m_ee, robot_state_ptr->F_x_Cee, robot_state_ptr->I_ee);
  message.inertia_load.inertia = translation::toInertia(
      robot_state_ptr->m_load, robot_state_ptr->F_x_Cload, robot_state_ptr->I_load);
  message.inertia_total.inertia = translation::toInertia(
      robot_state_ptr->m_total, robot_state_ptr->F_x_Ctotal, robot_state_ptr->I_total);

  // Errors and more
  message.time = robot_state_ptr->time.toSec();
  message.control_command_success_rate = robot_state_ptr->control_command_success_rate;
  message.current_errors = translation::errorsToMessage(robot_state_ptr->current_errors);
  message.last_motion_errors = translation::errorsToMessage(robot_state_ptr->last_motion_errors);

  switch (robot_state_ptr->robot_mode) {
    case franka::RobotMode::kOther:
      message.robot_mode = franka_msgs::msg::FrankaRobotState::ROBOT_MODE_OTHER;
      break;

    case franka::RobotMode::kIdle:
      message.robot_mode = franka_msgs::msg::FrankaRobotState::ROBOT_MODE_IDLE;
      break;

    case franka::RobotMode::kMove:
      message.robot_mode = franka_msgs::msg::FrankaRobotState::ROBOT_MODE_MOVE;
      break;

    case franka::RobotMode::kGuiding:
      message.robot_mode = franka_msgs::msg::FrankaRobotState::ROBOT_MODE_GUIDING;
      break;

    case franka::RobotMode::kReflex:
      message.robot_mode = franka_msgs::msg::FrankaRobotState::ROBOT_MODE_REFLEX;
      break;

    case franka::RobotMode::kUserStopped:
      message.robot_mode = franka_msgs::msg::FrankaRobotState::ROBOT_MODE_USER_STOPPED;
      break;

    case franka::RobotMode::kAutomaticErrorRecovery:
      message.robot_mode = franka_msgs::msg::FrankaRobotState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY;
      break;
  }
  return true;
}

auto FrankaRobotState::get_robot_state() -> franka::RobotState* {
  return robot_state_ptr;
}

}  // namespace franka_semantic_components
