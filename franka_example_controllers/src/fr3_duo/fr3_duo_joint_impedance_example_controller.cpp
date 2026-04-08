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

#include <franka_example_controllers/fr3_duo/fr3_duo_joint_impedance_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceFr3DuoExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
      config.names.push_back(arm_prefixes_[robot_index] + "_" + robot_types_[robot_index] +
                             "_joint" + std::to_string(i) + "/effort");
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceFr3DuoExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
      config.names.push_back(arm_prefixes_[robot_index] + "_" + robot_types_[robot_index] +
                             "_joint" + std::to_string(i) + "/position");
      config.names.push_back(arm_prefixes_[robot_index] + "_" + robot_types_[robot_index] +
                             "_joint" + std::to_string(i) + "/velocity");
    }
  }
  return config;
}

controller_interface::return_type JointImpedanceFr3DuoExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  updateJointStates();

  auto time_since_last_collision_msg =
      (this->get_node()->now() - last_collision_msg_time_).seconds();
  if (time_since_last_collision_msg > 0.5) {
    RCLCPP_FATAL(get_node()->get_logger(), "Connection to collision node timed out!");
    return controller_interface::return_type::ERROR;
  }
  if (collision_detected_) {
    RCLCPP_FATAL(get_node()->get_logger(), "Collision detected! Stopping controller.");
    return controller_interface::return_type::ERROR;
  }

  elapsed_time_ = elapsed_time_ + period.seconds();
  for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
    Vector7d q_goal = initial_q_[robot_index];

    double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * elapsed_time_));
    q_goal(3) += delta_angle;
    q_goal(4) += delta_angle;

    const double kAlpha = 0.99;
    dq_filtered_[robot_index] =
        (1 - kAlpha) * dq_filtered_[robot_index] + kAlpha * dq_[robot_index];
    Vector7d tau_d_calculated = k_gains_.cwiseProduct(q_goal - q_[robot_index]) +
                                d_gains_.cwiseProduct(-dq_filtered_[robot_index]);

    for (int i = 0; i < num_joints; ++i) {
      if (!command_interfaces_[i * robot_types_.size() + robot_index].set_value(
              tau_d_calculated(i))) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value");
      }
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceFr3DuoExampleController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("robot_types", std::vector<std::string>{});
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<std::string>>("arm_prefixes", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceFr3DuoExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_types_ = get_node()->get_parameter("robot_types").as_string_array();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }

  arm_prefixes_ = get_node()->get_parameter("arm_prefixes").as_string_array();
  for (const auto& prefix : arm_prefixes_) {
    RCLCPP_INFO(get_node()->get_logger(), "Received Arm Prefix: %s", prefix.c_str());
  }

  collision_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      kCollisionTopic, 1, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        this->collision_detected_ = msg->data;
        this->last_collision_msg_time_ = this->get_node()->now();
      });
  RCLCPP_INFO(get_node()->get_logger(), "Self-collision checking enabled on topic: %s",
              kCollisionTopic);

  for (auto& dq : dq_filtered_) {
    dq.setZero();
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  // robot_types_ = robot_utils::getRobotNameFromDescription(robot_description_,
  // get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceFr3DuoExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Ensure vectors are sized for the number of robots
  q_.resize(robot_types_.size(), Vector7d::Zero());
  dq_.resize(robot_types_.size(), Vector7d::Zero());
  dq_filtered_.resize(robot_types_.size(), Vector7d::Zero());
  initial_q_.resize(robot_types_.size(), Vector7d::Zero());

  updateJointStates();
  for (auto& dq : dq_filtered_) {
    dq.setZero();
  }
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  if (get_node()->count_publishers(kCollisionTopic) == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Self Collision Node not detected on %s",
                 kCollisionTopic);
    return CallbackReturn::FAILURE;
  }
  last_collision_msg_time_ = this->get_node()->now();
  collision_detected_ = false;

  return CallbackReturn::SUCCESS;
}

void JointImpedanceFr3DuoExampleController::updateJointStates() {
  for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
    for (auto i = 0; i < num_joints; ++i) {
      // Joint i, robot robot_index
      size_t pos_index = i * robot_types_.size() * 2 + robot_index * 2;
      size_t vel_index = pos_index + 1;

      const auto& position_interface = state_interfaces_.at(pos_index);
      const auto& velocity_interface = state_interfaces_.at(vel_index);

      auto position_value = position_interface.get_optional();
      auto velocity_value = velocity_interface.get_optional();

      if (position_value.has_value()) {
        q_[robot_index](i) = position_value.value();
      } else {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Failed to get position value for joint %d of robot %zu", i, robot_index);
      }

      if (velocity_value.has_value()) {
        dq_[robot_index](i) = velocity_value.value();
      } else {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Failed to get velocity value for joint %d of robot %zu", i, robot_index);
      }
    }
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceFr3DuoExampleController,
                       controller_interface::ControllerInterface)
