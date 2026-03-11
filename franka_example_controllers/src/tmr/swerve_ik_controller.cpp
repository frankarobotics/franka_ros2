// Copyright (c) 2023, PAL Robotics
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

#include <franka_example_controllers/tmr/swerve_ik_controller.hpp>
#include <controller_interface/helpers.hpp>

#include <algorithm>

#include "swerve_ik.hpp"


namespace franka_example_controllers
{

controller_interface::CallbackReturn SwerveIKController::on_init()
{
  // TODO pick up correct ik configuration from the TMR urdf tfs
  wheel_positions_ << 0.3, -0.2, -0.3, 0.2;
  steering_angles_.setZero();
  wheel_velocities_.setZero();
  wheel_radius_ = 0.05;

  get_node()->declare_parameter("prefix", "");
  const std::string prefix = this->get_node()->get_parameter("prefix").as_string();
  prefix_ = prefix.empty() ? "" : prefix + "_";

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveIKController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = {
    prefix_ + "joint_0/position", 
    prefix_ + "joint_1/velocity",
    prefix_ + "joint_2/position", 
    prefix_ + "joint_3/velocity"
  };

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SwerveIKController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_configuration;
    for (int i = 0; i < num_base_joints; ++i) {
      if (i % 2 == 0) {
        state_interface_configuration.names.push_back(prefix_ + "joint_" + std::to_string(i) + "/position");
      } else {
        state_interface_configuration.names.push_back(prefix_ + "joint_" + std::to_string(i) + "/velocity");
      }
    }
  return state_interface_configuration;
}

controller_interface::CallbackReturn SwerveIKController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/)
{
  // get some params here
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveIKController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveIKController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool SwerveIKController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::return_type SwerveIKController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const double vx = reference_interfaces_[0];
  const double vy = reference_interfaces_[1];
  const double wz = reference_interfaces_[5];

  std::array<WheelCommand, 2> commands;
  if(std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz)){
    computeSwerveIK(vx, vy, wz, wheel_positions_, wheel_radius_,
                                                steering_angles_, wheel_velocities_, commands);
  }else{
    RCLCPP_WARN(get_node()->get_logger(), "NaN/inf values command references.");
  }

  for (size_t i = 0; i < 2; ++i) {
    if (!command_interfaces_[2 * i].set_value(commands[i].steering_angle)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set steering angle for wheel %zu: %f", i,
                  commands[i].steering_angle);
    }
    if (!command_interfaces_[2 * i + 1].set_value(commands[i].wheel_velocity)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set wheel velocity for wheel %zu: %f", i,
                  commands[i].wheel_velocity);
    }
  }


  return controller_interface::return_type::OK;
}

  // Matches the command interface FrankaCartesianVelocityController for chaining
std::vector<hardware_interface::CommandInterface>
SwerveIKController::on_export_reference_interfaces()
{
  
  reference_interfaces_.resize(6, 0.0);

  const std::array<std::string, 6> names{"vx","vy","vz","wx","wy","wz"};
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t i = 0; i < names.size(); ++i) {
    interfaces.emplace_back(
      get_node()->get_name(),
      names[i] + "/cartesian_velocity", // TODO link this to be the same as the FrankaCartesianVelocityCommandInterfaceName
      &reference_interfaces_[i]);
  }

  return interfaces;
}

// NO OP, we are chaining
controller_interface::return_type SwerveIKController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  franka_example_controllers::SwerveIKController, controller_interface::ChainableControllerInterface)