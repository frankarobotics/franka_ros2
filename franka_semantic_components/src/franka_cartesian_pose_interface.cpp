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

#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include "rclcpp/logging.hpp"

#include <iostream>

namespace {
std::vector<double> combineArraysToVector(const std::array<double, 16>& cartesian_pose_command,
                                          const std::array<double, 2>& elbow_command) {
  std::vector<double> full_command;
  full_command.reserve(cartesian_pose_command.size() + elbow_command.size());
  full_command.insert(full_command.end(), cartesian_pose_command.begin(),
                      cartesian_pose_command.end());
  full_command.insert(full_command.end(), elbow_command.begin(), elbow_command.end());

  return full_command;
}
}  // namespace

namespace franka_semantic_components {

FrankaCartesianPoseInterface::FrankaCartesianPoseInterface(bool command_elbow_active)
    : FrankaCartesianPoseInterface("", command_elbow_active) {}

FrankaCartesianPoseInterface::FrankaCartesianPoseInterface(const std::string& arm_prefix,
                                                           bool command_elbow_active)
    : FrankaSemanticComponentInterface(arm_prefix + "cartesian_pose", 16, 16),
      command_elbow_active_(command_elbow_active) {
  if (command_elbow_active_) {
    command_interface_names_.reserve(full_command_interface_size_);
    command_interfaces_.reserve(full_command_interface_size_);
    state_interface_names_.reserve(full_command_interface_size_);
    state_interfaces_.reserve(full_command_interface_size_);
  }

  for (auto i = 0U; i < 16; i++) {
    auto full_interface_name =
        arm_prefix + std::to_string(i) + "/" + cartesian_pose_command_interface_name_;
    auto state_interface_name =
        arm_prefix + std::to_string(i) + "/" + cartesian_pose_state_interface_name_;
    command_interface_names_.emplace_back(full_interface_name);
    state_interface_names_.emplace_back(state_interface_name);
  }
  if (command_elbow_active_) {
    for (const auto& elbow_command_name : hw_elbow_names_) {
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

std::vector<double> FrankaCartesianPoseInterface::createColumnMajorTransformationMatrix(
    const Eigen::Quaterniond& quaternion,
    const Eigen::Vector3d& translation) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  pose.block<3, 1>(0, 3) = translation;

  std::vector<double> full_command;

  full_command.reserve(16);

  Eigen::Map<Eigen::VectorXd>(full_command.data(), pose.size()) =
      Eigen::VectorXd::Map(pose.data(), pose.size());

  return full_command;
}

bool FrankaCartesianPoseInterface::setCommand(const Eigen::Quaterniond& quaternion,
                                              const Eigen::Vector3d& translation) {
  if (command_elbow_active_) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_cartesian_velocity_interface"),
                 "Elbow command interface must not claimed, if elbow is not commanded.");
    return false;
  }

  auto full_command = createColumnMajorTransformationMatrix(quaternion, translation);

  return set_values(full_command);
}

bool FrankaCartesianPoseInterface::setCommand(const Eigen::Quaterniond& quaternion,
                                              const Eigen::Vector3d& translation,
                                              const std::array<double, 2>& elbow_command) {
  if (!command_elbow_active_) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_cartesian_pose_interface"),
                 "Elbow command interface must be claimed to command elbow.");
    return false;
  }

  auto pose = createColumnMajorTransformationMatrix(quaternion, translation);

  auto pose_array = std::array<double, 16>();
  std::copy_n(pose.begin(), pose_array.size(), pose_array.begin());

  auto full_command = combineArraysToVector(pose_array, elbow_command);

  return set_values(full_command);
}

bool FrankaCartesianPoseInterface::setCommand(const std::array<double, 16>& cartesian_pose_command,
                                              const std::array<double, 2>& elbow_command) {
  if (!command_elbow_active_) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_cartesian_pose_interface"),
                 "Elbow command interface must be claimed to command elbow.");
    return false;
  }
  auto full_command = combineArraysToVector(cartesian_pose_command, elbow_command);

  return set_values(full_command);
}

bool FrankaCartesianPoseInterface::setCommand(
    const std::array<double, 16>& cartesian_pose_command) {
  if (command_elbow_active_) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_cartesian_velocity_interface"),
                 "Elbow command interface must not claimed, if elbow is not commanded. If elbow is "
                 "activated, Use "
                 "setCommand(pose_command, elbow_command) interface.");
    return false;
  }

  std::vector<double> full_command;

  full_command.insert(full_command.end(), cartesian_pose_command.begin(),
                      cartesian_pose_command.end());

  return set_values(full_command);
}

std::array<double, 16> FrankaCartesianPoseInterface::getCommandedPoseMatrix() {
  std::vector<double> full_configuration;
  std::array<double, 16> pose_configuration{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  full_configuration = get_values_command_interfaces();

  std::copy_n(full_configuration.begin(), pose_configuration.size(), pose_configuration.begin());
  return pose_configuration;
};

std::array<double, 2> FrankaCartesianPoseInterface::getCommandedElbowConfiguration() {
  if (!command_elbow_active_) {
    throw std::runtime_error(
        "Elbow command interface must be claimed to receive elbow command state.");
  }
  std::array<double, 2> elbow_configuration{0, 0};
  auto full_configuration = get_values_command_interfaces();

  std::copy_n(full_configuration.begin() +
                  command_interface_size_,  // NOLINT(cppcoreguidelines-narrowing-conversions)
              elbow_command_interface_size_, elbow_configuration.begin());

  return elbow_configuration;
};

std::array<double, 16> FrankaCartesianPoseInterface::getCurrentPoseMatrix() {
  std::array<double, 16> current_pose{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  auto pose_from_state = get_values_state_interfaces();

  std::copy_n(pose_from_state.begin(), command_interface_size_, current_pose.begin());

  return current_pose;
}

std::tuple<Eigen::Quaterniond, Eigen::Vector3d>
FrankaCartesianPoseInterface::getCurrentOrientationAndTranslation() {
  std::array<double, 16> current_pose{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  auto pose_from_state = get_values_state_interfaces();
  std::copy_n(pose_from_state.begin(), command_interface_size_, current_pose.begin());

  Eigen::Matrix4d pose =
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(current_pose.data());
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
  Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

  return std::make_tuple(quaternion, translation);
};

bool FrankaCartesianPoseInterface::getCurrentOrientationAndTranslation(
    Eigen::Quaterniond& quaternion,
    Eigen::Vector3d& translation) const {
  if (state_interfaces_.size() < command_interface_size_) {
    return false;
  }
  std::array<double, 16> current_pose{};
  try {
    for (size_t index = 0; index < command_interface_size_; ++index) {
      const auto value = state_interfaces_[index].get().get_optional();
      if (!value.has_value() || !std::isfinite(*value)) {
        return false;
      }
      current_pose[index] = *value;
    }
  } catch (...) {
    return false;
  }

  const Eigen::Matrix4d pose =
      Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(current_pose.data());
  quaternion = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
  translation = pose.block<3, 1>(0, 3);
  return true;
}

std::tuple<Eigen::Quaterniond, Eigen::Vector3d>
FrankaCartesianPoseInterface::getCommandedOrientationAndTranslation() {
  std::array<double, 16> pose_matrix = getCommandedPoseMatrix();

  Eigen::Matrix4d pose =
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(pose_matrix.data());

  Eigen::Quaterniond quaternion = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
  Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

  return std::make_tuple(quaternion, translation);
}

std::array<double, 2> FrankaCartesianPoseInterface::getCurrentElbowConfiguration() {
  if (!command_elbow_active_) {
    throw std::runtime_error("Elbow command interface must be claimed to receive elbow state.");
  }

  std::array<double, 2> elbow_configuration{0, 0};
  auto full_configuration = get_values_state_interfaces();

  std::copy_n(full_configuration.begin() +
                  command_interface_size_,  // NOLINT(cppcoreguidelines-narrowing-conversions)
              elbow_command_interface_size_, elbow_configuration.begin());

  return elbow_configuration;
}

}  // namespace franka_semantic_components
