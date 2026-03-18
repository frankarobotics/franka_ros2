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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "franka_selfcollision/self_collision_checker.hpp"

namespace franka_selfcollision {

class CollisionMonitorNode : public rclcpp::Node {
 public:
  explicit CollisionMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Initializes the underlying collision checker.
   * @param robot_description The XML string content of the robot's URDF.
   */
  void setup_collision_monitor(const std::string& robot_description);

 private:
  /**
   * @brief Uses incoming joint data to execute collision check and broadcasts collision status.
   * @param msg The incoming joint state message containing names and positions.
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool print_collisions_;

  std::shared_ptr<franka_selfcollision::SelfCollisionChecker> collision_checker_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;

  std::vector<double> current_joint_positions_;
  std::unordered_map<std::string, size_t> joint_map_;
  const std::string kBaseLink = "universe";
};

}  // namespace franka_selfcollision