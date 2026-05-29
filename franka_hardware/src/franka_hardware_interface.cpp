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

#include <fmt/core.h>
#include <algorithm>
#include <cmath>
#include <exception>

#include <franka/exception.h>
#include <franka/logging/logger.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_hardware/franka_hardware_interface.hpp"
#include "franka_hardware/ros_libfranka_logger.hpp"

const std::string kVersionName = "version";
const std::string kRobotIpName = "robot_ip";
const std::string kArmIdName = "robot_type";

namespace {

auto parseVersion(const std::string& version_str) {
  std::vector<std::string> version_parts;
  std::stringstream ss(version_str);
  std::string item;
  while (std::getline(ss, item, '.')) {
    version_parts.push_back(item);
  }

  if (version_parts.size() != 3) {
    throw std::invalid_argument(
        "\033[1;31mInvalid version structure in URDF. Please update your URDF (aka "
        "franka_description).\033[0m");
  }

  return std::make_tuple(std::stoi(version_parts[0]), std::stoi(version_parts[1]),
                         std::stoi(version_parts[2]));
}

}  // namespace
namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

FrankaHardwareInterface::FrankaHardwareInterface(const std::shared_ptr<Robot>& robot,
                                                 const std::string& robot_type)
    : FrankaHardwareInterface() {
  robot_ = robot;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  robot_type_ = robot_type;
}

FrankaHardwareInterface::FrankaHardwareInterface()
    : command_interfaces_info_({
          {hardware_interface::HW_IF_EFFORT, kNumberOfJoints, effort_interface_claimed_},
          {hardware_interface::HW_IF_VELOCITY, kNumberOfJoints, velocity_joint_interface_claimed_},
          {hardware_interface::HW_IF_POSITION, kNumberOfJoints, position_joint_interface_claimed_},
          {k_HW_IF_ELBOW_COMMAND, hw_elbow_command_names_.size(), elbow_command_interface_claimed_},
          {k_HW_IF_CARTESIAN_VELOCITY, hw_cartesian_velocities_.size(),
           velocity_cartesian_interface_claimed_},
          {k_HW_IF_CARTESIAN_POSE_COMMAND, hw_cartesian_pose_commands_.size(),
           pose_cartesian_interface_claimed_},
      }) {
  // Allow libfranka to use the ROS logger
  franka::logging::addLogger(std::make_shared<RosLibfrankaLogger>(getLogger()));
}

std::vector<StateInterface> FrankaHardwareInterface::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_.at(i)));
  }

  state_interfaces.emplace_back(StateInterface(
      prefix_ + robot_type_, k_robot_state_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_robot_state_addr_)));
  state_interfaces.emplace_back(StateInterface(
      prefix_ + robot_type_, k_robot_model_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_model_ptr_)));

  // cartesian pose state interface 16 element pose matrix
  for (auto i = 0U; i < 16; i++) {
    state_interfaces.emplace_back(StateInterface(
        prefix_ + std::to_string(i), k_HW_IF_CARTESIAN_POSE_STATE, &cartesian_pose_state_.at(i)));
  }

  // elbow state interface
  for (auto i = 0U; i < elbow_state_names_.size(); i++) {
    state_interfaces.emplace_back(StateInterface(prefix_ + elbow_state_names_.at(i),
                                                 k_HW_IF_ELBOW_STATE, &elbow_state_.at(i)));
  }

  state_interfaces.emplace_back(
      StateInterface(prefix_ + robot_type_, "robot_time", &robot_time_state_));

  // Force/torque sensor state interfaces (loaded from URDF sensor declarations)
  for (const auto& sensor : info_.sensors) {
    for (size_t i = 0; i < sensor.state_interfaces.size(); i++) {
      state_interfaces.emplace_back(StateInterface(sensor.name, sensor.state_interfaces[i].name,
                                                   &force_torque_sensor_state_[i]));
    }
  }

  return state_interfaces;
}

std::vector<CommandInterface> FrankaHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  // Register all command interfaces defined in the URDF
  RCLCPP_INFO(getLogger(), "Register joint-based command interfaces");
  for (auto joint_index = 0U; joint_index < info_.joints.size(); joint_index++) {
    const auto& joint = info_.joints[joint_index];
    for (const auto& command_interface : joint.command_interfaces) {
      command_interfaces.emplace_back(
          CommandInterface(joint.name, command_interface.name,
                           &command_interface_map_.at(command_interface.name)[joint_index]));

      RCLCPP_INFO(getLogger(),
                  "Registering command interface: %s for command interface %s with index %d",
                  joint.name.c_str(), command_interface.name.c_str(), joint_index);
    }
  }

  RCLCPP_INFO(getLogger(), "Register general purpose command interfaces");
  for (const auto& gpio : info_.gpios) {
    for (const auto& command_interface : gpio.command_interfaces) {
      auto vector_index = std::stoul(gpio.parameters.at("index"));
      command_interfaces.emplace_back(
          CommandInterface(gpio.name, command_interface.name,
                           &command_interface_map_.at(command_interface.name)[vector_index]));

      RCLCPP_INFO(getLogger(),
                  "Registering command interface: %s for command interface %s with index %ld",
                  gpio.name.c_str(), command_interface.name.c_str(), vector_index);
    }
  }

  return command_interfaces;
}

CallbackReturn FrankaHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  active_mode_ = ControlInterface::None;
  needs_initial_command_ = true;
  hw_franka_model_ptr_ = nullptr;

  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  return CallbackReturn::SUCCESS;
}

FrankaHardwareInterface::~FrankaHardwareInterface() {
  // Ensure executor is fully stopped and nodes are removed before members are destroyed.
  // This prevents races where executor worker threads are still running callbacks
  // that reference nodes or robot_ during destruction.
  if (executor_) {
    if (action_node_) {
      executor_->remove_node(action_node_);
    }
    if (service_node_) {
      executor_->remove_node(service_node_);
    }
    executor_.reset();
  }
  action_node_.reset();
  service_node_.reset();
}

CallbackReturn FrankaHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  robot_->stopRobot();

  active_mode_ = ControlInterface::None;
  needs_initial_command_ = true;
  return CallbackReturn::SUCCESS;
}

void FrankaHardwareInterface::initializePositionCommands(const franka::RobotState& robot_state) {
  if (!needs_initial_command_ || active_mode_ == ControlInterface::None) {
    return;
  }

  switch (active_mode_) {
    case ControlInterface::JointPosition:
      std::copy(robot_state.q.begin(), robot_state.q.end(), hw_position_commands_.begin());
      break;
    case ControlInterface::CartesianPose:
      std::copy(robot_state.O_T_EE.begin(), robot_state.O_T_EE.end(),
                hw_cartesian_pose_commands_.begin());
      break;
    case ControlInterface::CartesianPoseWithElbow:
      std::copy(robot_state.O_T_EE.begin(), robot_state.O_T_EE.end(),
                hw_cartesian_pose_commands_.begin());
      std::copy(robot_state.elbow.begin(), robot_state.elbow.end(), hw_elbow_command_.begin());
      break;
    case ControlInterface::CartesianVelocityWithElbow:
      std::copy(robot_state.elbow.begin(), robot_state.elbow.end(), hw_elbow_command_.begin());
      break;
    default:
      break;
  }

  needs_initial_command_ = false;
}

hardware_interface::return_type FrankaHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  if (hw_franka_model_ptr_ == nullptr) {
    hw_franka_model_ptr_ = robot_->getModel();
  }

  franka::RobotState robot_state;
  try {
    // Write new state into the RealtimeBuffer for thread-safe access by consumers
    robot_state = robot_->readOnce();
  } catch (const franka::ControlException& e) {
    RCLCPP_ERROR(getLogger(), "%s", e.what());
    robot_->stopRobot();
    return hardware_interface::return_type::ERROR;
  }

  hw_franka_robot_state_ = robot_state;
  robot_time_state_ = robot_state.time.toSec();
  initializePositionCommands(robot_state);

  hw_positions_ = hw_franka_robot_state_.q;
  hw_velocities_ = hw_franka_robot_state_.dq;
  hw_efforts_ = hw_franka_robot_state_.tau_J;
  elbow_state_ = hw_franka_robot_state_.elbow;
  cartesian_pose_state_ = hw_franka_robot_state_.O_T_EE;
  force_torque_sensor_state_ = hw_franka_robot_state_.K_F_ext_hat_K;

  return hardware_interface::return_type::OK;
}

template <typename CommandType>
bool hasInfinite(const CommandType& commands) {
  return std::any_of(commands.begin(), commands.end(),
                     [](double command) { return !std::isfinite(command); });
}

hardware_interface::return_type FrankaHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  if (hasInfinite(hw_position_commands_) || hasInfinite(hw_effort_commands_) ||
      hasInfinite(hw_velocity_commands_) || hasInfinite(hw_cartesian_velocities_) ||
      hasInfinite(hw_elbow_command_) || hasInfinite(hw_cartesian_pose_commands_)) {
    return hardware_interface::return_type::ERROR;
  }

  if (needs_initial_command_) {
    return hardware_interface::return_type::OK;
  }

  try {
    switch (active_mode_) {
      case ControlInterface::Effort:
        robot_->writeOnce(hw_effort_commands_);
        break;
      case ControlInterface::JointVelocity:
        robot_->writeOnce(hw_velocity_commands_);
        break;
      case ControlInterface::JointPosition:
        robot_->writeOnce(hw_position_commands_);
        break;
      case ControlInterface::CartesianVelocity:
        robot_->writeOnce(hw_cartesian_velocities_);
        break;
      case ControlInterface::CartesianVelocityWithElbow:
        robot_->writeOnce(hw_cartesian_velocities_, hw_elbow_command_);
        break;
      case ControlInterface::CartesianPose:
        robot_->writeOnce(hw_cartesian_pose_commands_);
        break;
      case ControlInterface::CartesianPoseWithElbow:
        robot_->writeOnce(hw_cartesian_pose_commands_, hw_elbow_command_);
        break;
      case ControlInterface::None:
        break;
    }
  } catch (const std::runtime_error& e) {
    // Transient race during mode switch — the RT write() can overlap with
    // perform_command_mode_switch on the non-RT thread.  Warn instead of
    // returning ERROR so the controller_manager does not cascade-deactivate
    // all hardware and controllers.
    RCLCPP_WARN(getLogger(), "Write skipped during mode switch: %s", e.what());
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn FrankaHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Build set of exported command interfaces for direct lookup
  exported_command_interfaces_.clear();
  for (const auto& joint : info.joints) {
    for (const auto& cmd_interface : joint.command_interfaces) {
      exported_command_interfaces_.insert(joint.name + "/" + cmd_interface.name);
    }
  }
  for (const auto& gpio : info.gpios) {
    for (const auto& cmd_interface : gpio.command_interfaces) {
      exported_command_interfaces_.insert(gpio.name + "/" + cmd_interface.name);
    }
  }

  try {
    auto version_str = info_.hardware_parameters.at(kVersionName);
    auto [major, minor, patch] = parseVersion(version_str);

    RCLCPP_INFO(getLogger(), "Parsed Franka ros2_control interface version: %d.%d.%d", major, minor,
                patch);

    if (kSupportedControlInterfaceMajor != major) {
      RCLCPP_FATAL(getLogger(),
                   "Unsupported major version of the Franka ros2_control interface. Expected "
                   "major version %d, got %d. Please update your URDF (aka franka_description).",
                   kSupportedControlInterfaceMajor, major);
      return CallbackReturn::ERROR;
    }
  } catch (const std::out_of_range& ex) {
    std::cout << "Parameter 'version' is not set. Please update your URDF (aka franka_description)."
              << std::endl;
    RCLCPP_FATAL(getLogger(),
                 "Parameter '%s' is not set. Please update your URDF (aka franka_description).",
                 kVersionName.c_str());
    return CallbackReturn::ERROR;
  }

  try {
    robot_ip_ = info_.hardware_parameters.at(kRobotIpName);
  } catch (const std::out_of_range& ex) {
    RCLCPP_FATAL(getLogger(), "Parameter '%s' is not set", kRobotIpName.c_str());
    return CallbackReturn::ERROR;
  }

  try {
    robot_type_ = info_.hardware_parameters.at(kArmIdName);
  } catch (const std::out_of_range& ex) {
    RCLCPP_WARN(getLogger(), "Parameter '%s' is not set.", kArmIdName.c_str());
    RCLCPP_WARN(getLogger(),
                "Deprecation Warning: In the next release, 'robot_type' should be set in the URDF. "
                "Using 'panda' as default 'robot_type' will not be supported."
                "Please use the latest franka_description package from: "
                "https://github.com/frankarobotics/franka_description");
  }

  try {
    prefix_ = info_.hardware_parameters.at("prefix");
  } catch (const std::out_of_range& ex) {
    RCLCPP_INFO(getLogger(), "Parameter 'prefix' is not set. Using empty prefix.");
    prefix_ = "";
  }

  if (!robot_) {
    try {
      RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", robot_ip_.c_str());
      robot_ = std::make_shared<Robot>(robot_ip_, getLogger());
    } catch (const franka::Exception& e) {
      RCLCPP_FATAL(getLogger(), "Could not connect to robot");
      RCLCPP_FATAL(getLogger(), "%s", fmt::format("{}", e.what()).c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
  }

  service_node_ = std::make_shared<FrankaParamServiceServer>(rclcpp::NodeOptions(), robot_);
  executor_ = std::make_shared<FrankaExecutor>();
  executor_->add_node(service_node_);

  action_node_ = std::make_shared<ActionServer>(rclcpp::NodeOptions(), robot_);
  executor_->add_node(action_node_);

  return CallbackReturn::SUCCESS;
}

rclcpp::Logger FrankaHardwareInterface::getLogger() {
  return rclcpp::get_logger("FrankaHardwareInterface");
}

hardware_interface::return_type FrankaHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  if (elbow_command_interface_claimed_ &&
      !(velocity_cartesian_interface_claimed_ || pose_cartesian_interface_claimed_)) {
    RCLCPP_FATAL(getLogger(),
                 "Elbow cannot be commanded without cartesian velocity or pose interface");
    return hardware_interface::return_type::ERROR;
  }

  ControlInterface desired = ControlInterface::None;

  if (effort_interface_claimed_) {
    desired = ControlInterface::Effort;
  } else if (velocity_joint_interface_claimed_) {
    desired = ControlInterface::JointVelocity;
  } else if (position_joint_interface_claimed_) {
    desired = ControlInterface::JointPosition;
  } else if (pose_cartesian_interface_claimed_ && elbow_command_interface_claimed_) {
    desired = ControlInterface::CartesianPoseWithElbow;
  } else if (pose_cartesian_interface_claimed_) {
    desired = ControlInterface::CartesianPose;
  } else if (velocity_cartesian_interface_claimed_ && elbow_command_interface_claimed_) {
    desired = ControlInterface::CartesianVelocityWithElbow;
  } else if (velocity_cartesian_interface_claimed_) {
    desired = ControlInterface::CartesianVelocity;
  }

  if (desired == active_mode_) {
    return hardware_interface::return_type::OK;
  }

  // Set mode to None BEFORE stopping, so that a concurrent write() from the
  // RT thread sees ControlInterface::None and skips the writeOnce() call.
  active_mode_ = ControlInterface::None;
  needs_initial_command_ = true;
  robot_->stopRobot();

  switch (desired) {
    case ControlInterface::Effort:
      std::fill(hw_effort_commands_.begin(), hw_effort_commands_.end(), 0);
      robot_->initializeTorqueInterface();
      needs_initial_command_ = false;
      break;
    case ControlInterface::JointVelocity:
      std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0);
      robot_->initializeJointVelocityInterface();
      needs_initial_command_ = false;
      break;
    case ControlInterface::JointPosition:
      robot_->initializeJointPositionInterface();
      needs_initial_command_ = true;
      break;
    case ControlInterface::CartesianVelocity:
      std::fill(hw_cartesian_velocities_.begin(), hw_cartesian_velocities_.end(), 0);
      robot_->initializeCartesianVelocityInterface();
      needs_initial_command_ = false;
      break;
    case ControlInterface::CartesianVelocityWithElbow:
      std::fill(hw_cartesian_velocities_.begin(), hw_cartesian_velocities_.end(), 0);
      robot_->initializeCartesianVelocityInterface();
      needs_initial_command_ = true;
      break;
    case ControlInterface::CartesianPose:
      robot_->initializeCartesianPoseInterface();
      needs_initial_command_ = true;
      break;
    case ControlInterface::CartesianPoseWithElbow:
      robot_->initializeCartesianPoseInterface();
      needs_initial_command_ = true;
      break;
    case ControlInterface::None:
      needs_initial_command_ = true;
      break;
  }

  active_mode_ = desired;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  auto contains_interface_type = [this](const std::string& interface,
                                        const std::string& interface_type) {
    // Check if we export this exact interface AND it matches the requested type
    if (exported_command_interfaces_.count(interface) > 0) {
      // Verify the interface type matches
      size_t slash_pos = interface.find('/');
      if (slash_pos != std::string::npos && slash_pos + 1 < interface.size()) {
        std::string actual_type = interface.substr(slash_pos + 1);
        return actual_type == interface_type;
      }
    }
    return false;
  };

  // Check if the number of start and stop interfaces is valid
  if (start_interfaces.size() == max_number_start_interfaces) {
    RCLCPP_FATAL(getLogger(),
                 "Invalid number of start interface. Do you return empty array in your controllers "
                 "command_interface_configuration?");
    return hardware_interface::return_type::ERROR;
  }

  auto generate_error_message = [this](const std::string& start_stop_command,
                                       const std::string& interface_name,
                                       size_t actual_interface_size,
                                       size_t expected_interface_size) {
    std::string error_message =
        fmt::format("Invalid number of {} interfaces to {}. Expected {}, given {}", interface_name,
                    start_stop_command, expected_interface_size, actual_interface_size);
    RCLCPP_FATAL(this->getLogger(), "%s", error_message.c_str());

    throw std::invalid_argument(error_message);
  };

  for (const auto& interface : command_interfaces_info_) {
    size_t num_stop_interface =
        std::count_if(stop_interfaces.begin(), stop_interfaces.end(),
                      [contains_interface_type, &interface](const std::string& interface_given) {
                        return contains_interface_type(interface_given, interface.interface_type);
                      });
    size_t num_start_interface =
        std::count_if(start_interfaces.begin(), start_interfaces.end(),
                      [contains_interface_type, &interface](const std::string& interface_given) {
                        return contains_interface_type(interface_given, interface.interface_type);
                      });

    if (num_stop_interface == interface.size) {
      interface.claim_flag = false;
    } else if (num_stop_interface != 0U) {
      generate_error_message("stop", interface.interface_type, num_stop_interface, interface.size);
    }
    if (num_start_interface == interface.size) {
      interface.claim_flag = true;
    } else if (num_start_interface != 0U) {
      generate_error_message("start", interface.interface_type, num_start_interface,
                             interface.size);
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaHardwareInterface,
                       hardware_interface::SystemInterface)
