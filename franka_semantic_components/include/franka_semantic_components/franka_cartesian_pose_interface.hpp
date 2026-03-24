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

#pragma once

#include <Eigen/Dense>
#include <limits>
#include <string>
#include <vector>

#include "franka/control_types.h"
#include "franka/robot_state.h"
#include "franka_semantic_components/franka_semantic_component_interface.hpp"

namespace franka_semantic_components {
/**
 * @brief Franka Cartesian Pose interface abstraction on top of hardware_interface to set the
 * full cartesian pose. The Command should either have the form of a 4x4 column major homogenous
 * transformation matrix or a quaternion and translation vector. Optionally, the elbow can be
 * commanded. [joint_3_position, joint_4_sign]
 */
class FrankaCartesianPoseInterface
    : public FrankaSemanticComponentInterface {  // NOLINT(cppcoreguidelines-special-member-functions)
 public:
  /**
   * Initializes the franka cartesian velocity interface with access to the hardware
   * interface command interfaces.
   *
   * @param[in] command_elbow_active insert true to activate the elbow commanding together with the
   * cartesian velocity input, otherwise the elbow commanding is not allowed.
   * @param[in] arm_prefix prefix prepended to the robot hardware interfaces.
   *
   */
  explicit FrankaCartesianPoseInterface(bool command_elbow_activate);
  FrankaCartesianPoseInterface(const std::string& arm_prefix, bool command_elbow_active);
  FrankaCartesianPoseInterface(const FrankaCartesianPoseInterface&) = delete;
  FrankaCartesianPoseInterface& operator=(const FrankaCartesianPoseInterface& other) = delete;
  FrankaCartesianPoseInterface& operator=(FrankaCartesianPoseInterface&& other) = delete;
  FrankaCartesianPoseInterface(FrankaCartesianPoseInterface&& other) = default;

  ~FrankaCartesianPoseInterface() override = default;

  /**
   * Sets the given orientation and translation command, when elbow is not activated.
   *
   * @param[in] quaternion rotation represented in quaternion format [x,y,z,w]
   * @param[in] translation translation represented in Vector3d format [x,y,z]
   *
   * @return if command was set successfully true, else when elbow is activated false.
   */
  bool setCommand(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& translation);

  /**
   * @brief Sets the given command. Based on rotation and translation and the elbow command.
   *
   * @param[in] quaternion rotation represented in quaternion format [x,y,z,w]
   * @param[in] translation translation represented in Vector3d format [x,y,z]
   * @param[in] elbow_command elbow format [joint_3_position, joint_4_sign]
   * @return true when command was set successfully
   * @return false when elbow is not activated
   */
  bool setCommand(const Eigen::Quaterniond& quaternion,
                  const Eigen::Vector3d& translation,
                  const std::array<double, 2>& elbow_command);

  /**
   * Sends the given pose command to the robot.
   *
   * @param[in] pose_command column major homogenous transformation matrix.
   *
   * @return true when command was set successfully
   * @return false when elbow is activated
   */
  bool setCommand(const std::array<double, 16>& pose_command);

  /**
   * Sets the given command.
   *
   * @param[in] cartesian_pose_command Rotation plus translation commands in column major homogenous
   * transformation matrix.
   * @param[in] elbow elbow format [joint_3_position, joint_4_sign]
   *
   * @return true when command was set successfully
   * @return false when elbow is not activated
   */
  bool setCommand(const std::array<double, 16>& pose_command, const std::array<double, 2>& elbow);

  /**
   * Get the commanded elbow interface elbow values.

   * @throws std::runtime_error if the elbow is not activated.

   * @return elbow_configuration [joint3_position, joint4_sign]
   */
  std::array<double, 2> getCommandedElbowConfiguration();

  /**
   * Get the commanded pose interface values.
   *
   * @return pose_configuration commanded pose values in column major homogenous transformation
   *
   */
  std::array<double, 16> getCommandedPoseMatrix();

  /**
   * Get the commanded orientation and translation values.
   *
   * @return std::tuple<Eigen::Quaterniond, Eigen::Vector3d> commanded orientation values in
   * quaternion format. Commanded translation values in Vector3d format [x,y,z].
   */
  std::tuple<Eigen::Quaterniond, Eigen::Vector3d> getCommandedOrientationAndTranslation();

  /**
   * @brief Get the current elbow configuration
   *
   * @throws std::runtime_error if the elbow is not activated.
   *
   * @return std::array<double, 2> elbow configuration [joint_3_position, joint_4_sign]
   */
  std::array<double, 2> getCurrentElbowConfiguration();

  /**
   * @brief Get the current Orientation And Translation
   *
   * @return std::tuple<Eigen::Quaterniond, Eigen::Vector3d> current orientation values in
   * quaternion format. current translation values in Vector3d format [x,y,z].
   */
  std::tuple<Eigen::Quaterniond, Eigen::Vector3d> getCurrentOrientationAndTranslation();

  /**
   * @brief Get the current pose matrix
   *
   * @return std::array<double, 16> Current pose matrix column major homogenous transformation
   */
  std::array<double, 16> getCurrentPoseMatrix();

  /**
   * @brief returns the column major transformation matrix from the given quaternion and translation
   *
   * @param quaternion rotation represented in quaternion format [x,y,z,w]
   * @param translation translation represented in Vector3d format [x,y,z]
   * @return std::vector<double> column major transformation matrix [4x4]
   */
  static std::vector<double> createColumnMajorTransformationMatrix(
      const Eigen::Quaterniond& quaternion,
      const Eigen::Vector3d& translation);

 private:
  const std::array<std::string, 2> hw_elbow_names_{"joint_3_position", "joint_4_sign"};
  const std::array<std::string, 2> elbow_state_names_{"joint_3_position", "joint_4_sign"};

  const size_t full_command_interface_size_{18};
  const size_t command_interface_size_{16};
  const size_t elbow_command_interface_size_{2};

  bool command_elbow_active_;

  const std::string cartesian_pose_command_interface_name_{"cartesian_pose_command"};
  const std::string elbow_command_interface_name_{"elbow_command"};

  const std::string cartesian_pose_state_interface_name_{"cartesian_pose_state"};
  const std::string elbow_state_interface_name_{"elbow_state"};
};

}  // namespace franka_semantic_components
