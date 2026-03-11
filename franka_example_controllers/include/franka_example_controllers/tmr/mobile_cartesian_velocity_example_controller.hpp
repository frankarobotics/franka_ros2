// Copyright (c) 2025 Franka Robotics GmbH
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

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The cartesian velocity example controller
 */
class MobileCartesianVelocityExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface>
      franka_cartesian_velocity_;
  geometry_msgs::msg::TwistStamped::SharedPtr last_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  std::string reference_controller_ = "";

  double prev_linear_velocity_x_ = 0.0;
  double prev_linear_velocity_y_ = 0.0;
  double prev_angular_velocity_z_ = 0.0;
  double prev_linear_acceleration_x_ = 0.0;
  double prev_linear_acceleration_y_ = 0.0;
  double prev_angular_acceleration_z_ = 0.0;

  double max_acceleration_linear_ = 2.5;
  double max_acceleration_angular_ = 7.5;
  double max_jerk_linear_ = 3000.0;
  double max_jerk_angular_ = 5000.0;

  double last_cmd_time_;
  double timeout_sec = 0.2;
  double dt = 0.001;
  size_t queue_size_ = 10;
};

}  // namespace franka_example_controllers