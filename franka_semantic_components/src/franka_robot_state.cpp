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

#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "translation_utils.hpp"

namespace {

constexpr size_t kBaseLinkIndex = 0;
constexpr size_t kFlangeLinkIndex = 8;
constexpr size_t kLoadLinkIndex = 8;
constexpr size_t kAccelerometerCount = 6;
// libfranka reports exactly this many joints per arm; every std::array in
// franka::RobotState that this component copies from is this wide.
constexpr size_t kJointCount = 7;
const std::string kTCPFrameName = "_hand_tcp";

// franka::RobotMode and FrankaRobotState.robot_mode are intentionally isomorphic.
// Keep the hot path as a single cast; fail the build if libfranka or the .msg drifts.
static_assert(static_cast<uint8_t>(franka::RobotMode::kOther) ==
              franka_msgs::msg::FrankaRobotState::ROBOT_MODE_OTHER);
static_assert(static_cast<uint8_t>(franka::RobotMode::kIdle) ==
              franka_msgs::msg::FrankaRobotState::ROBOT_MODE_IDLE);
static_assert(static_cast<uint8_t>(franka::RobotMode::kMove) ==
              franka_msgs::msg::FrankaRobotState::ROBOT_MODE_MOVE);
static_assert(static_cast<uint8_t>(franka::RobotMode::kGuiding) ==
              franka_msgs::msg::FrankaRobotState::ROBOT_MODE_GUIDING);
static_assert(static_cast<uint8_t>(franka::RobotMode::kReflex) ==
              franka_msgs::msg::FrankaRobotState::ROBOT_MODE_REFLEX);
static_assert(static_cast<uint8_t>(franka::RobotMode::kUserStopped) ==
              franka_msgs::msg::FrankaRobotState::ROBOT_MODE_USER_STOPPED);
static_assert(static_cast<uint8_t>(franka::RobotMode::kAutomaticErrorRecovery) ==
              franka_msgs::msg::FrankaRobotState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY);
static_assert(static_cast<uint8_t>(franka::RobotMode::kAutomaticErrorRecovery) == 6u);

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
  // This component is bound to a single arm's state interface, and libfranka reports exactly
  // kJointCount values for that arm. The robot_description, however, is the controller
  // manager's global one and may describe several arms, so it cannot be used to size or label
  // the published arrays. Take this arm's joints by name, in joint order, the same way the
  // accelerometer frames are resolved in initialize_robot_state_msg().
  joint_names.reserve(kJointCount);
  for (size_t i = 0; i < kJointCount; ++i) {
    const auto joint_name = robot_name_ + "_joint" + std::to_string(i + 1);
    if (model_->joints_.find(joint_name) == model_->joints_.end()) {
      throw std::runtime_error("Joint '" + joint_name + "' not found in URDF.");
    }
    joint_names.push_back(joint_name);
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

  // libfranka's accelerometer index i belongs to joint i+1; a joint's sensors are mounted
  // on its parent link and named "<parent link>_accelerometer_<side>" in the URDF.
  for (size_t i = 0; i < kAccelerometerCount; ++i) {
    const auto joint_name = robot_name_ + "_joint" + std::to_string(i + 1);
    const auto joint_it = model_->joints_.find(joint_name);
    if (joint_it == model_->joints_.end()) {
      throw std::runtime_error("Joint '" + joint_name + "' not found in URDF.");
    }
    const auto top_frame = joint_it->second->parent_link_name + "_accelerometer_top";
    const auto bottom_frame = joint_it->second->parent_link_name + "_accelerometer_bottom";
    if (model_->links_.find(top_frame) == model_->links_.end()) {
      throw std::runtime_error("Accelerometer frame '" + top_frame + "' not found in URDF.");
    }
    if (model_->links_.find(bottom_frame) == model_->links_.end()) {
      throw std::runtime_error("Accelerometer frame '" + bottom_frame + "' not found in URDF.");
    }
    message.accelerometer_top[i].header.frame_id = top_frame;
    message.accelerometer_bottom[i].header.frame_id = bottom_frame;
  }
}

auto FrankaRobotState::assign_loaned_state_interfaces(
    std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) -> bool {
  if (!Base::assign_loaned_state_interfaces(state_interfaces)) {
    return false;
  }
  if (!initialize_state_buffer()) {
    release_interfaces();
    return false;
  }
  return true;
}

auto FrankaRobotState::release_interfaces() -> void {
  robot_state_box_ = nullptr;
  cached_robot_state_valid_ = false;
  Base::release_interfaces();
}

auto FrankaRobotState::initialize_state_buffer() -> bool {
  // assign_loaned_state_interfaces() claims the interfaces named in the constructor and
  // orders them to match, so a complete claim leaves the robot state interface first.
  if (state_interfaces_.size() != interface_names_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_robot_state_semantic_component"),
                 "Franka state interface '%s' was not claimed! Did you assign the loaned state in "
                 "the controller?",
                 full_robot_state_interface_name_.c_str());
    return false;
  }

  // The hardware hands over the address of its state box through the interface value.
  // By default, the robot state interface is the first and only interface.
  const auto interface_value = state_interfaces_.front().get().get_optional();
  if (!interface_value.has_value()) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_robot_state_semantic_component"),
                 "Could not read the Franka state interface.");
    return false;
  }

  robot_state_box_ =
      bit_cast<realtime_tools::RealtimeThreadSafeBox<franka::RobotState>*>(interface_value.value());
  if (robot_state_box_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_robot_state_semantic_component"),
                 "The Franka state interface carries a null robot state box.");
    return false;
  }
  return true;
}

auto FrankaRobotState::get_values_as_message(franka_msgs::msg::FrankaRobotState& message) -> bool {
  // Safety net if interfaces were claimed without going through our assign() wrapper.
  if (robot_state_box_ == nullptr && !initialize_state_buffer()) {
    return false;
  }

  // Robot state consumers are assuming this is reliable.
  // Don't drop any robot state the hardware has written to the box.
  // Warning: this blocks until the hardware releases the box!
  robot_state_ = robot_state_box_->get();
  cached_robot_state_valid_ = true;

  // Update the time stamps of the data
  translation::updateTimeStamps(message.header.stamp, message);

  // Collision and contact indicators
  message.collision_indicators = translation::toCollisionIndicators(
      robot_state_.cartesian_collision, robot_state_.cartesian_contact,
      robot_state_.joint_collision, robot_state_.joint_contact);

  // The joint states
  const auto n_joints = message.measured_joint_state.position.size();
  std::copy_n(robot_state_.q.cbegin(), n_joints, message.measured_joint_state.position.begin());
  std::copy_n(robot_state_.dq.cbegin(), n_joints, message.measured_joint_state.velocity.begin());
  std::copy_n(robot_state_.tau_J.cbegin(), n_joints, message.measured_joint_state.effort.begin());

  std::copy_n(robot_state_.q_d.cbegin(), n_joints, message.desired_joint_state.position.begin());
  std::copy_n(robot_state_.dq_d.cbegin(), n_joints, message.desired_joint_state.velocity.begin());
  std::copy_n(robot_state_.tau_J_d.cbegin(), n_joints, message.desired_joint_state.effort.begin());

  std::copy_n(robot_state_.theta.cbegin(), n_joints,
              message.measured_joint_motor_state.position.begin());
  std::copy_n(robot_state_.dtheta.cbegin(), n_joints,
              message.measured_joint_motor_state.velocity.begin());

  std::copy_n(robot_state_.tau_ext_hat_filtered.cbegin(), n_joints,
              message.tau_ext_hat_filtered.effort.begin());

  message.ddq_d = robot_state_.ddq_d;
  message.dtau_j = robot_state_.dtau_J;

  // Output for the elbow
  message.elbow =
      translation::toElbow(robot_state_.elbow, robot_state_.elbow_d, robot_state_.elbow_c,
                           robot_state_.delbow_c, robot_state_.ddelbow_c);

  // Joint-mounted accelerometer data
  for (size_t i = 0; i < message.accelerometer_top.size(); ++i) {
    message.accelerometer_top[i].vector = translation::toVector3(robot_state_.accelerometer_top[i]);
    message.accelerometer_bottom[i].vector =
        translation::toVector3(robot_state_.accelerometer_bottom[i]);
  }

  // Active wrenches on the stiffness frame
  message.k_f_ext_hat_k.wrench = translation::toWrench(robot_state_.K_F_ext_hat_K);
  message.o_f_ext_hat_k.wrench = translation::toWrench(robot_state_.O_F_ext_hat_K);

  // The transformations between different frames
  message.o_t_ee.pose = translation::toPose(robot_state_.O_T_EE);
  message.o_t_ee_d.pose = translation::toPose(robot_state_.O_T_EE_d);
  message.o_t_ee_c.pose = translation::toPose(robot_state_.O_T_EE_c);

  message.f_t_ee.pose = translation::toPose(robot_state_.F_T_EE);
  message.ee_t_k.pose = translation::toPose(robot_state_.EE_T_K);

  message.o_dp_ee_d.twist = translation::toTwist(robot_state_.O_dP_EE_d);
  message.o_dp_ee_c.twist = translation::toTwist(robot_state_.O_dP_EE_c);
  message.o_ddp_ee_c.accel = translation::toAccel(robot_state_.O_ddP_EE_c);

  // The inertias of the robot
  message.inertia_ee.inertia =
      translation::toInertia(robot_state_.m_ee, robot_state_.F_x_Cee, robot_state_.I_ee);
  message.inertia_load.inertia =
      translation::toInertia(robot_state_.m_load, robot_state_.F_x_Cload, robot_state_.I_load);
  message.inertia_total.inertia =
      translation::toInertia(robot_state_.m_total, robot_state_.F_x_Ctotal, robot_state_.I_total);

  // Errors and more
  message.time = robot_state_.time.toSec();
  message.control_command_success_rate = robot_state_.control_command_success_rate;
  message.current_errors = translation::errorsToMessage(robot_state_.current_errors);
  message.last_motion_errors = translation::errorsToMessage(robot_state_.last_motion_errors);

  message.robot_mode = static_cast<uint8_t>(robot_state_.robot_mode);

  return true;
}

auto FrankaRobotState::get_robot_state() -> franka::RobotState* {
  return cached_robot_state_valid_ ? &robot_state_ : nullptr;
}

}  // namespace franka_semantic_components
