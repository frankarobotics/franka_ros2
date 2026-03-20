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

#include "franka/control_types.h"
#include "franka/robot_state.h"
#include "franka_semantic_components/franka_semantic_component_interface.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace franka_semantic_components {
/**
 * @brief Franka Cartesian Velocity interface abstraction on top of hardware_interface to set the
 * full cartesian velocity. The Command should have the form [linear_velocity, angular_velocity] =
 * [vx, vy, vz, wx, wy, wz]. Optionally, the elbow can be commanded. [joint_3_position,
 * joint_4_sign]
 */
class FrankaCartesianVelocityInterface : public FrankaSemanticComponentInterface {
 public:
  enum { VX, VY, VZ, WX, WY, WZ };

  /**
   * Initializes the franka cartesian velocity interface with access to the hardware
   * interface command interfaces.
   *
   * @param[in] command_elbow_active if true to activates the elbow commanding together with the
   * cartesian velocity input, else elbow commanding is not allowed.
   * @param[in] arm_prefix prefix prepended to the robot hardware interfaces.
   *
   */
  explicit FrankaCartesianVelocityInterface(bool command_elbow_activate);
  FrankaCartesianVelocityInterface(const std::string& arm_prefix, bool command_elbow_active);
  FrankaCartesianVelocityInterface(const FrankaCartesianVelocityInterface&) = delete;
  FrankaCartesianVelocityInterface& operator=(FrankaCartesianVelocityInterface const&) = delete;
  FrankaCartesianVelocityInterface(FrankaCartesianVelocityInterface&&) = default;

  ~FrankaCartesianVelocityInterface() override = default;

  /**
   * @param twist_command The velocity command in Cartesian coordinates
   * @return true if successul
   *
   * @return if successful true, else when elbow is activated false.
   */
  bool setCommand(const Eigen::Vector3d& linear_velocity_command,
                  const Eigen::Vector3d& angular_velocity_command);

  /**
   * @param twist_command The velocity command in Cartesian coordinates
   * @param elbow_command The elbow command: {joint_3, sign(joint_4)}
   * @return true if successul
   *
   * @return if successful true, else when elbow is not activated false.
   */
  bool setCommand(const Eigen::Vector3d& linear_velocity_command,
                  const Eigen::Vector3d& angular_velocity_command,
                  const std::array<double, 2>& elbow_command);

  /**
   * Get the commanded elbow interface elbow values.
   *.
   * @throws std::runtime_error if the elbow is not activated.
   *
   * @return elbow_configuration [joint3_position, joint4_sign]
   */
  std::array<double, 2> getCommandedElbowConfiguration();

  /**
   * @brief Get the current elbow configuration
   *
   * @return elbow_configuration [joint3_position, joint4_sign]
   */
  std::array<double, 2> getCurrentElbowConfiguration();

 private:
  const std::array<std::string, 6> hw_cartesian_velocities_names_{"vx", "vy", "vz",
                                                                  "wx", "wy", "wz"};
  const std::array<std::string, 2> hw_elbow_command_names_{"joint_3_position", "joint_4_sign"};
  const std::array<std::string, 2> elbow_state_names_{"joint_3_position", "joint_4_sign"};

  const size_t full_command_interface_size_{8};
  bool command_elbow_active_;

  const std::string cartesian_velocity_command_interface_name_{"cartesian_velocity"};
  const std::string elbow_command_interface_name_{"elbow_command"};
  const std::string elbow_state_interface_name_{"elbow_state"};
};

}  // namespace franka_semantic_components
