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

#include <franka_example_controllers/fr3/gravity_compensation_example_controller.hpp>

#include <exception>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
GravityCompensationExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
GravityCompensationExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  auto ft_interface_names = force_torque_sensor_->get_state_interface_names();
  config.names.insert(config.names.end(), ft_interface_names.begin(), ft_interface_names.end());
  return config;
}

controller_interface::return_type GravityCompensationExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  for (auto& command_interface : command_interfaces_) {
    if (!command_interface.set_value(0.)) {
      return controller_interface::return_type::ERROR;
    }
  }

  constexpr size_t kLogEveryN = 100;
  if (++log_counter_ >= kLogEveryN) {
    log_counter_ = 0;
    geometry_msgs::msg::Wrench wrench;
    force_torque_sensor_->get_values_as_message(wrench);
    RCLCPP_INFO(get_node()->get_logger(),
                "External wrench (stiffness frame): F=[%.2f, %.2f, %.2f] T=[%.2f, %.2f, %.2f]",
                wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y,
                wrench.torque.z);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn GravityCompensationExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";

  force_torque_sensor_ =
      std::make_unique<semantic_components::ForceTorqueSensor>(arm_prefix_ + robot_type_ + "_tcp");

  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  force_torque_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationExampleController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::GravityCompensationExampleController,
                       controller_interface::ControllerInterface)
