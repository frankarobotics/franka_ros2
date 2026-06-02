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
#include <rclcpp/wait_for_message.hpp>
#include <std_msgs/msg/string.hpp>

#include <franka_example_controllers/fr3/model_example_controller.hpp>
#include <franka_example_controllers/robot_model_franka.hpp>
#include <franka_example_controllers/robot_model_pinocchio.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

namespace franka_example_controllers {

controller_interface::CallbackReturn ModelExampleController::on_init() {
  try {
    auto_declare("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");

    if (!get_node()->get_parameter("robot_type", robot_type_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get robot_type parameter");
      get_node()->shutdown();
      return CallbackReturn::ERROR;
    }
    if (!get_node()->get_parameter("arm_prefix", arm_prefix_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get arm_prefix parameter");
      get_node()->shutdown();
      return CallbackReturn::ERROR;
    }
    arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
    
    auto_declare<bool>("gazebo", false);
    if (!get_node()->get_parameter("gazebo", gazebo)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get gazebo parameter");
      get_node()->shutdown();
      return CallbackReturn::ERROR;
    }

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ModelExampleController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ModelExampleController::state_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (gazebo) {
    // In simulation, we need joint position and velocity state interfaces
    for (int i = 1; i <= 7; i++) {
      state_interfaces_config.names.push_back(
          arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/position");
      state_interfaces_config.names.push_back(
          arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/velocity");
    }
  } else {
    // On hardware, request the FCI model/state interfaces
    for (const auto& name : robot_model_->get_state_interface_names()) {
      state_interfaces_config.names.push_back(name);
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn ModelExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  if (gazebo) {
    auto temp_node = std::make_shared<rclcpp::Node>("_urdf_fetcher");
    std_msgs::msg::String msg;

    rclcpp::QoS qos(1);
    qos.transient_local();

    if (!rclcpp::wait_for_message(msg, temp_node, "/robot_description",
            std::chrono::seconds(5), qos)) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Timed out waiting for /robot_description topic");
        return CallbackReturn::ERROR;
    }

    robot_model_ = std::make_unique<robot_model_interface::RobotModelPinocchio>(msg.data);
    RCLCPP_INFO(get_node()->get_logger(), "Using Pinocchio model backend (simulation)");
  } 
  else {
    robot_model_ = std::make_unique<robot_model_interface::RobotModelFranka>(
        arm_prefix_ + robot_type_);
    RCLCPP_INFO(get_node()->get_logger(), "Using Franka FCI model backend (hardware)");
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ModelExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ModelExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}



controller_interface::return_type ModelExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  if (gazebo) {
    robot_model_interface::Vector7d q, dq;
    for (int i = 0; i < 7; i++) {
      q(i) = state_interfaces_[2 * i].get_value();
      dq(i) = state_interfaces_[2 * i + 1].get_value();
    }
    robot_model_->updateState(q, dq);
  }

  auto mass = robot_model_->getMassMatrix();
  auto coriolis = robot_model_->getCoriolis();
  auto gravity = robot_model_->getGravity();
  auto pose = robot_model_->getPose("end_effector");
  auto jacobian = robot_model_->getJacobian("end_effector");

  Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "-------------------------------------------------------------");
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "mass :\n" << mass.format(fmt));
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "coriolis :" << coriolis.transpose().format(fmt));
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "gravity :" << gravity.transpose().format(fmt));
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "ee_pose :\n" << pose.matrix().format(fmt));
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "ee_jacobian :\n" << jacobian.format(fmt));
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "-------------------------------------------------------------");

  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelExampleController,
                       controller_interface::ControllerInterface)   