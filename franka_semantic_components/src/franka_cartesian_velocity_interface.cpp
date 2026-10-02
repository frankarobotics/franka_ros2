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

#include "franka_semantic_components/franka_cartesian_velocity_interface.hpp"

#include <cstring>
#include <iostream>
#include <string>
#include "rclcpp/logging.hpp"

namespace {
std::vector<double> combineArraysToVector(const Eigen::Vector3d& linear_velocity_command,
                                          const Eigen::Vector3d& angular_velocity_command,
                                          const std::array<double, 2>& elbow_command) {
  std::vector<double> full_command{linear_velocity_command.x(),
                                   linear_velocity_command.y(),
                                   linear_velocity_command.z(),
                                   angular_velocity_command.x(),
                                   angular_velocity_command.y(),
                                   angular_velocity_command.z(),
                                   elbow_command[0],
                                   elbow_command[1]};

  return full_command;
}
}  // namespace

namespace franka_semantic_components {

FrankaCartesianVelocityInterface::FrankaCartesianVelocityInterface(bool command_elbow_active)
    : FrankaCartesianVelocityInterface("", command_elbow_active) {}

FrankaCartesianVelocityInterface::FrankaCartesianVelocityInterface(const std::string& arm_prefix,
                                                                   bool command_elbow_active)
    : FrankaSemanticComponentInterface(arm_prefix + "cartesian_velocity_command", 0, 6),
      command_elbow_active_(command_elbow_active) {
  if (command_elbow_active_) {
    command_interface_names_.reserve(full_command_interface_size_);
    command_interfaces_.reserve(full_command_interface_size_);
    state_interface_names_.reserve(elbow_state_names_.size());
    state_interfaces_.reserve(elbow_state_names_.size());
  }

  for (const auto& velocity_command_name : hw_cartesian_velocities_names_) {
    auto full_interface_name =
        arm_prefix + velocity_command_name + "/" + cartesian_velocity_command_interface_name_;
    command_interface_names_.emplace_back(full_interface_name);
  }
  if (command_elbow_active_) {
    for (const auto& elbow_command_name : hw_elbow_command_names_) {
      auto full_elbow_command_name =
          arm_prefix + elbow_command_name + "/" + elbow_command_interface_name_;
      command_interface_names_.emplace_back(full_elbow_command_name);
    }
    for (const auto& elbow_state_name : elbow_state_names_) {
      auto full_elbow_state_name =
          arm_prefix + elbow_state_name + "/" + elbow_state_interface_name_;
      state_interface_names_.emplace_back(full_elbow_state_name);
    }
  }
}

bool FrankaCartesianVelocityInterface::setCommand(const Eigen::Vector3d& linear_velocity_command,
                                                  const Eigen::Vector3d& angular_velocity_command,
                                                  const std::array<double, 2>& elbow_command) {
  if (!command_elbow_active_) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_cartesian_velocity_interface"),
                 "Elbow command interface must be claimed to command elbow.");
    return false;
  }
  auto full_command =
      combineArraysToVector(linear_velocity_command, angular_velocity_command, elbow_command);

  return set_values(full_command);
}

bool FrankaCartesianVelocityInterface::setCommand(const Eigen::Vector3d& linear_velocity_command,
                                                  const Eigen::Vector3d& angular_velocity_command) {
  if (command_elbow_active_) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_cartesian_velocity_interface"),
                 "Elbow command interface must not claimed, if elbow is not commanded.");
    return false;
  }

  const std::array<double, 6> full_command{
      linear_velocity_command.x(),  linear_velocity_command.y(),  linear_velocity_command.z(),
      angular_velocity_command.x(), angular_velocity_command.y(), angular_velocity_command.z()};

  return set_values(full_command);
}

std::array<double, 2> FrankaCartesianVelocityInterface::getCommandedElbowConfiguration() {
  if (!command_elbow_active_) {
    throw std::runtime_error(
        "Elbow command interface must be claimed to receive elbow command state.");
  }
  std::array<double, 2> elbow_configuration;
  auto full_configuration = get_values_command_interfaces();

  std::copy_n(full_configuration.begin() + hw_cartesian_velocities_names_.size(),
              hw_elbow_command_names_.size(), elbow_configuration.begin());

  return elbow_configuration;
};

std::array<double, 2> FrankaCartesianVelocityInterface::getCurrentElbowConfiguration() {
  if (!command_elbow_active_) {
    throw std::runtime_error("Elbow command interface must be claimed to receive elbow state.");
  }

  std::array<double, 2> elbow_configuration;
  auto full_configuration = get_values_state_interfaces();
  std::copy_n(full_configuration.begin(), elbow_state_names_.size(), elbow_configuration.begin());

  return elbow_configuration;
};

}  // namespace franka_semantic_components
