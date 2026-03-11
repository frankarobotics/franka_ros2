// Copyright (c) 2026 Franka Robotics GmbH
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

#include <Eigen/Eigen>

namespace franka_example_controllers {

struct WheelCommand {
  double steering_angle;
  double wheel_velocity;
};

/**
 * @brief Computes the inverse kinematics for a swerve drive system.
 *
 * @param vx Desired linear velocity in the x-direction (m/s).
 * @param vy Desired linear velocity in the y-direction (m/s).
 * @param wz Desired angular velocity around the z-axis (rad/s).
 * @param wheel_positions A 4-element vector containing the (x, y) positions of each wheel relative
 * to the robot's center (m).
 * @param wheel_radius The radius of the wheels (m).
 * @param[out] steering_angles A 4-element vector to store the computed steering angles for each
 * wheel (rad).
 * @param[out] wheel_velocities A 4-element vector to store the computed wheel velocities (rad/s).
 * @param[out] commands An array of 2 WheelCommand objects to store control commands for each wheel.
 */
void computeSwerveIK(double vx,
                     double vy,
                     double wz,
                     const Eigen::Vector4d& wheel_positions,
                     double wheel_radius,
                     Eigen::Vector4d& steering_angles,
                     Eigen::Vector4d& wheel_velocities,
                     std::array<WheelCommand, 2>& commands);

}  // namespace franka_example_controllers