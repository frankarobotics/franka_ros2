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

#include "franka/robot_state.h"

#include "franka_msgs/msg/collision_indicators.hpp"
#include "franka_msgs/msg/elbow.hpp"
#include "franka_msgs/msg/errors.hpp"
#include "franka_msgs/msg/franka_robot_state.hpp"

#include "builtin_interfaces/msg/time.hpp"

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/inertia.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"

namespace franka_semantic_components {
namespace translation {

/**
 * @param error The internal error buffer
 * @return franka_msgs::msg::Errors The translated errors
 */
auto errorsToMessage(const franka::Errors& error) -> franka_msgs::msg::Errors;

/**
 * @param input_wrench The wrench which should be translated
 * @return geometry_msgs::msg::Wrench The translated wrench
 */
auto toWrench(const std::array<double, 6>& input_wrench) -> geometry_msgs::msg::Wrench;

/**
 * @param input_twist The twist which should be translated
 * @return geometry_msgs::msg::Twist The translated twist
 */
auto toTwist(const std::array<double, 6>& input_twist) -> geometry_msgs::msg::Twist;

/**
 * @param input_accel The acceleration which should be translated
 * @return geometry_msgs::msg::Accel The translated acceleration
 */
auto toAccel(const std::array<double, 6>& input_accel) -> geometry_msgs::msg::Accel;

/**
 * @param input_vector The 3D acceleration vector which should be translated
 * @return geometry_msgs::msg::Vector3 The translated vector
 */
template <typename T>
auto toVector3(const std::array<T, 3>& input_vector) -> geometry_msgs::msg::Vector3 {
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
      .x(static_cast<double>(input_vector[0]))
      .y(static_cast<double>(input_vector[1]))
      .z(static_cast<double>(input_vector[2]));
}

/**
 * @param input_pose The pose which should be translated
 * @return geometry_msgs::msg::Pose The translated pose
 */
auto toPose(const std::array<double, 16>& input_pose) -> geometry_msgs::msg::Pose;

/**
 * @param mass The mass for the inertia
 * @param center_of_mass The center of mass for the inertia
 * @param inertia_matrix The inertia matrix
 * @return geometry_msgs::msg::Inertia The translated inertia
 */
auto toInertia(double mass,
               const std::array<double, 3>& center_of_mass,
               const std::array<double, 9>& inertia_matrix) -> geometry_msgs::msg::Inertia;

/**
 * The indicators within this message represent if a collision/contact is active for Cartesian/joint
 * space
 * @param cartesian_collision The Cartesian collision flags
 * @param cartesian_contact The Cartesian contact flags
 * @param joint_collision The joint collision flags
 * @param joint_contact The joint contact flags
 * @return franka_msgs::msg::CollisionIndicators The translated CollisionIndicator message
 */
auto toCollisionIndicators(const std::array<double, 6>& cartesian_collision,
                           const std::array<double, 6>& cartesian_contact,
                           const std::array<double, 7>& joint_collision,
                           const std::array<double, 7>& joint_contact)
    -> franka_msgs::msg::CollisionIndicators;

/**
 * @param elbow The elbow position
 * @param elbow_d The desired elbow position
 * @param elbow_c The commanded elbow position
 * @param delbow_c The commanded elbow velocity
 * @param ddelbow_c The commanded elbow acceleration
 * @return franka_msgs::msg::Elbow The translated elbow message
 */
auto toElbow(const std::array<double, 2>& elbow,
             const std::array<double, 2>& elbow_d,
             const std::array<double, 2>& elbow_c,
             const std::array<double, 2>& delbow_c,
             const std::array<double, 2>& ddelbow_c) -> franka_msgs::msg::Elbow;

/**
 * @param robot_state Updates the the
 */
auto updateTimeStamps(const builtin_interfaces::msg::Time& time_stamps,
                      franka_msgs::msg::FrankaRobotState& robot_state) -> void;

}  // namespace translation
}  // namespace franka_semantic_components
