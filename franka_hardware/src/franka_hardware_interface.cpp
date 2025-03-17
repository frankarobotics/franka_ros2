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
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_hardware/franka_hardware_interface.hpp"
#include "franka_hardware/ros_libfranka_logger.hpp"

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

FrankaHardwareInterface::FrankaHardwareInterface(std::shared_ptr<Robot> robot,
                                                 const std::string& arm_id)
    : FrankaHardwareInterface() {
  robot_ = std::move(robot);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  arm_id_ = arm_id;
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
      arm_id_, k_robot_state_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_robot_state_addr_)));
  state_interfaces.emplace_back(StateInterface(
      arm_id_, k_robot_model_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_model_ptr_)));

  // cartesian pose state interface 16 element pose matrix
  for (auto i = 0U; i < 16; i++) {
    state_interfaces.emplace_back(StateInterface(std::to_string(i), k_HW_IF_CARTESIAN_POSE_STATE,
                                                 &cartesian_pose_state_.at(i)));
  }

  // elbow state interface
  for (auto i = 0U; i < elbow_state_names_.size(); i++) {
    state_interfaces.emplace_back(
        StateInterface(elbow_state_names_.at(i), k_HW_IF_ELBOW_STATE, &elbow_state_.at(i)));
  }

  state_interfaces.emplace_back(StateInterface(arm_id_, "robot_time", &robot_time_state_));

  return state_interfaces;
}

std::vector<CommandInterface> FrankaHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (auto i = 0U; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_.at(i)));
    command_interfaces.emplace_back(CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_.at(i)));
    command_interfaces.emplace_back(CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_.at(i)));
  }

  // cartesian velocity command interface 6 in order: dx, dy, dz, wx, wy, wz
  for (auto i = 0U; i < hw_cartesian_velocities_.size(); i++) {
    command_interfaces.emplace_back(CommandInterface(hw_cartesian_velocities_names_.at(i),
                                                     k_HW_IF_CARTESIAN_VELOCITY,
                                                     &hw_cartesian_velocities_.at(i)));
  }

  // cartesian pose command interface 16 element pose matrix
  for (auto i = 0U; i < 16; i++) {
    command_interfaces.emplace_back(CommandInterface(
        std::to_string(i), k_HW_IF_CARTESIAN_POSE_COMMAND, &hw_cartesian_pose_commands_.at(i)));
  }

  // elbow command interface
  for (auto i = 0U; i < hw_elbow_command_names_.size(); i++) {
    command_interfaces.emplace_back(CommandInterface(
        hw_elbow_command_names_.at(i), k_HW_IF_ELBOW_COMMAND, &hw_elbow_command_.at(i)));
  }

  return command_interfaces;
}

CallbackReturn FrankaHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  read(rclcpp::Time(0),
       rclcpp::Duration(0, 0));  // makes sure that the robot state is properly initialized.
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  robot_->stopRobot();
  RCLCPP_INFO(getLogger(), "Stopped");
  return CallbackReturn::SUCCESS;
}

template <typename CommandType>
void initializeCommand(bool& first_update,
                       const bool& interface_running,
                       CommandType& hw_command,
                       const CommandType& new_command) {
  if (first_update && interface_running) {
    hw_command = new_command;
    first_update = false;
  }
}

void FrankaHardwareInterface::initializePositionCommands(const franka::RobotState& robot_state) {
  initializeCommand(first_elbow_update_, elbow_command_interface_running_, hw_elbow_command_,
                    robot_state.elbow);
  initializeCommand(first_position_update_, position_joint_interface_running_,
                    hw_position_commands_, robot_state.q);
  initializeCommand(first_cartesian_pose_update_, pose_cartesian_interface_running_,
                    hw_cartesian_pose_commands_, robot_state.O_T_EE);
}

hardware_interface::return_type FrankaHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  if (hw_franka_model_ptr_ == nullptr) {
    hw_franka_model_ptr_ = robot_->getModel();
  }
  hw_franka_robot_state_ = robot_->readOnce();
  robot_time_state_ = hw_franka_robot_state_.time.toSec();
  initializePositionCommands(hw_franka_robot_state_);

  hw_positions_ = hw_franka_robot_state_.q;
  hw_velocities_ = hw_franka_robot_state_.dq;
  hw_efforts_ = hw_franka_robot_state_.tau_J;
  elbow_state_ = hw_franka_robot_state_.elbow;
  cartesian_pose_state_ = hw_franka_robot_state_.O_T_EE;

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

  if (velocity_joint_interface_running_) {
    robot_->writeOnce(hw_velocity_commands_);
  } else if (effort_interface_running_) {
    robot_->writeOnce(hw_effort_commands_);
  } else if (position_joint_interface_running_ && !first_position_update_) {
    robot_->writeOnce(hw_position_commands_);
  } else if (velocity_cartesian_interface_running_ && elbow_command_interface_running_ &&
             !first_elbow_update_) {
    // Wait until the first read pass after robot controller is activated to write the elbow
    // command to the robot
    robot_->writeOnce(hw_cartesian_velocities_, hw_elbow_command_);
  } else if (pose_cartesian_interface_running_ && elbow_command_interface_running_ &&
             !first_cartesian_pose_update_ && !first_elbow_update_) {
    // Wait until the first read pass after robot controller is activated to write the elbow
    // command to the robot
    robot_->writeOnce(hw_cartesian_pose_commands_, hw_elbow_command_);
  } else if (pose_cartesian_interface_running_ && !first_cartesian_pose_update_) {
    // Wait until the first read pass after robot controller is activated to write the cartesian
    // pose
    robot_->writeOnce(hw_cartesian_pose_commands_);
  } else if (velocity_cartesian_interface_running_ && !elbow_command_interface_running_) {
    robot_->writeOnce(hw_cartesian_velocities_);
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn FrankaHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                 kNumberOfJoints);
    return CallbackReturn::ERROR;
  }

  for (const auto& joint : info_.joints) {
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT &&
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY &&
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(),
                   "Joint '%s' has unexpected command interface '%s'. Expected '%s' and '%s' ",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
    }
  }
  if (!robot_) {
    std::string robot_ip;
    try {
      robot_ip = info_.hardware_parameters.at("robot_ip");
    } catch (const std::out_of_range& ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' is not set");
      return CallbackReturn::ERROR;
    }
    try {
      arm_id_ = info_.hardware_parameters.at("arm_id");
    } catch (const std::out_of_range& ex) {
      RCLCPP_WARN(getLogger(), "Parameter 'arm_id' is not set.");
      RCLCPP_WARN(getLogger(),
                  "Deprecation Warning: In the next release, 'arm_id' should be set in the URDF. "
                  "Using 'panda' as default 'arm_id' will not be supported."
                  "Please use the latest franka_description package from: "
                  "https://github.com/frankaemika/franka_description");
    }
    try {
      RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", robot_ip.c_str());
      robot_ = std::make_shared<Robot>(robot_ip, getLogger());
    } catch (const franka::Exception& e) {
      RCLCPP_FATAL(getLogger(), "Could not connect to robot");
      RCLCPP_FATAL(getLogger(), "%s", e.what());
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
  if (!effort_interface_running_ && effort_interface_claimed_) {
    hw_effort_commands_.fill(0);
    robot_->stopRobot();
    robot_->initializeTorqueInterface();
    effort_interface_running_ = true;
  } else if (effort_interface_running_ && !effort_interface_claimed_) {
    robot_->stopRobot();
    effort_interface_running_ = false;
  }

  if (!velocity_joint_interface_running_ && velocity_joint_interface_claimed_) {
    hw_velocity_commands_.fill(0);
    robot_->stopRobot();
    robot_->initializeJointVelocityInterface();
    velocity_joint_interface_running_ = true;
  } else if (velocity_joint_interface_running_ && !velocity_joint_interface_claimed_) {
    robot_->stopRobot();
    velocity_joint_interface_running_ = false;
  }

  if (!position_joint_interface_running_ && position_joint_interface_claimed_) {
    robot_->stopRobot();
    robot_->initializeJointPositionInterface();
    position_joint_interface_running_ = true;
    first_position_update_ = true;
  } else if (position_joint_interface_running_ && !position_joint_interface_claimed_) {
    robot_->stopRobot();
    position_joint_interface_running_ = false;
  }

  if (!velocity_cartesian_interface_running_ && velocity_cartesian_interface_claimed_) {
    hw_cartesian_velocities_.fill(0);
    robot_->stopRobot();
    robot_->initializeCartesianVelocityInterface();
    if (!elbow_command_interface_running_ && elbow_command_interface_claimed_) {
      elbow_command_interface_running_ = true;
      first_elbow_update_ = true;
    }
    velocity_cartesian_interface_running_ = true;
  } else if (velocity_cartesian_interface_running_ && !velocity_cartesian_interface_claimed_) {
    robot_->stopRobot();
    // Elbow command interface can't be commanded without cartesian velocity or pose interface
    if (elbow_command_interface_running_) {
      elbow_command_interface_running_ = false;
      elbow_command_interface_claimed_ = false;
    }
    velocity_cartesian_interface_running_ = false;
  }

  if (!pose_cartesian_interface_running_ && pose_cartesian_interface_claimed_) {
    robot_->stopRobot();
    robot_->initializeCartesianPoseInterface();
    if (!elbow_command_interface_running_ && elbow_command_interface_claimed_) {
      elbow_command_interface_running_ = true;
      first_elbow_update_ = true;
    }
    pose_cartesian_interface_running_ = true;
    initial_robot_state_update_ = true;
    first_cartesian_pose_update_ = true;
  } else if (pose_cartesian_interface_running_ && !pose_cartesian_interface_claimed_) {
    robot_->stopRobot();
    // Elbow command interface can't be commanded without cartesian pose or pose interface
    if (elbow_command_interface_running_) {
      elbow_command_interface_running_ = false;
      elbow_command_interface_claimed_ = false;
    }
    pose_cartesian_interface_running_ = false;
  }

  // check if the elbow command is activated without cartesian command interface
  if (elbow_command_interface_claimed_ &&
      !(velocity_cartesian_interface_claimed_ || pose_cartesian_interface_claimed_)) {
    RCLCPP_FATAL(getLogger(),
                 "Elbow cannot be commanded without cartesian velocity or pose interface");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  auto contains_interface_type = [this](const std::string& interface,
                                        const std::string& interface_type) {
    size_t slash_position = interface.find('/');
    if (slash_position != std::string::npos && slash_position + 1 < interface.size()) {
      std::string after_slash = interface.substr(slash_position + 1);
      std::string before_slash = interface.substr(0, slash_position);

      if (after_slash == interface_type) {
        if (interface_type != "position" && interface_type != "velocity" &&
            interface_type != "effort" && interface_type != "acceleration") {
          return true;
        }

        for (const auto& joint : this->info_.joints) {
          if (before_slash == joint.name) {
            return true;
          }
        }
      }
    }
    return false;
  };

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
