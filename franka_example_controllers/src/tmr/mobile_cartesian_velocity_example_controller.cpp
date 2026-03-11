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

#include <franka_example_controllers/tmr/mobile_cartesian_velocity_example_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "fmt/format.h"

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
MobileCartesianVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
MobileCartesianVelocityExampleController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type MobileCartesianVelocityExampleController::update(
    const rclcpp::Time&,
    const rclcpp::Duration& period) {
  last_cmd_time_ += period.seconds();

  double target_linear_velocity_x = 0.0;
  double target_linear_velocity_y = 0.0;
  double target_angular_velocity_z = 0.0;

  if (last_cmd_vel_ && last_cmd_time_ < timeout_sec) {
    target_linear_velocity_x = last_cmd_vel_->twist.linear.x;
    target_linear_velocity_y = last_cmd_vel_->twist.linear.y;
    target_angular_velocity_z = last_cmd_vel_->twist.angular.z;
  };

  auto limit_acc = [&](double target_a, double prev_a, double max_jerk, double max_acc) {
    double delta_a = target_a - prev_a;
    double max_delta = max_jerk * dt;
    if (std::abs(delta_a) > max_delta) {
      delta_a = std::copysign(max_delta, delta_a);
    }
    double limited_a = prev_a + delta_a;
    if (std::abs(limited_a) > max_acc) {
      return std::copysign(max_acc, limited_a);
    }
    return limited_a;
  };

  double target_linear_acceleration_x = (target_linear_velocity_x - prev_linear_velocity_x_) / dt;
  double target_linear_acceleration_y = (target_linear_velocity_y - prev_linear_velocity_y_) / dt;
  double target_angular_acceleration_z =
      (target_angular_velocity_z - prev_angular_velocity_z_) / dt;

  prev_linear_acceleration_x_ = limit_acc(target_linear_acceleration_x, prev_linear_acceleration_x_,
                                          max_jerk_linear_, max_acceleration_linear_);
  prev_linear_acceleration_y_ = limit_acc(target_linear_acceleration_y, prev_linear_acceleration_y_,
                                          max_jerk_linear_, max_acceleration_linear_);
  prev_angular_acceleration_z_ =
      limit_acc(target_angular_acceleration_z, prev_angular_acceleration_z_, max_jerk_angular_,
                max_acceleration_angular_);

  prev_linear_velocity_x_ += prev_linear_acceleration_x_ * dt;
  prev_linear_velocity_y_ += prev_linear_acceleration_y_ * dt;
  prev_angular_velocity_z_ += prev_angular_acceleration_z_ * dt;

  Eigen::Vector3d cartesian_linear_velocity(prev_linear_velocity_x_, prev_linear_velocity_y_, 0.0);
  Eigen::Vector3d cartesian_angular_velocity(0.0, 0.0, prev_angular_velocity_z_);

  if (franka_cartesian_velocity_->setCommand(cartesian_linear_velocity,
                                             cartesian_angular_velocity)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn MobileCartesianVelocityExampleController::on_init() {
  if(!get_node()->has_parameter("cartesian_velocity_interface_prefix")){
    get_node()->declare_parameter("cartesian_velocity_interface_prefix", "");
  }
  cartesian_velocity_interface_prefix_ =
      get_node()->get_parameter("cartesian_velocity_interface_prefix").as_string();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileCartesianVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  const std::string ns = get_node()->get_namespace();
  const std::string cmd_vel_topic = ns == "/"
                                        ? "/mobile_cartesian_velocity_controller/cmd_vel"
                                        : ns + "/mobile_cartesian_velocity_controller/cmd_vel";

  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          cartesian_velocity_interface_prefix_, false);

  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic, queue_size_, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_cmd_vel_ = msg;
        last_cmd_time_ = 0.0;
      });

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileCartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  last_cmd_vel_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MobileCartesianVelocityExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MobileCartesianVelocityExampleController,
                       controller_interface::ControllerInterface)