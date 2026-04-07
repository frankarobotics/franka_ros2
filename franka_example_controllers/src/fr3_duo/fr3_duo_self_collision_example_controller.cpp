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

#include <franka_example_controllers/fr3_duo/fr3_duo_self_collision_example_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
SelfCollisionFR3DuoExampleController::command_interface_configuration() const {
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
SelfCollisionFR3DuoExampleController::state_interface_configuration() const {
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

controller_interface::return_type SelfCollisionFR3DuoExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();

  auto trajectory_time = this->get_node()->now() - start_time_;
  auto time_since_last_collision_msg =
      (this->get_node()->now() - last_collision_msg_time_).seconds();
  bool all_finished = true;

  if (time_since_last_collision_msg > 0.5) {
    RCLCPP_FATAL(get_node()->get_logger(), "Connection to collision node timed out!");
    return controller_interface::return_type::ERROR;
  }

  if (phase_ == ControlPhase::MOVE_TO_COLLISION) {
    if (collision_detected_) {
      RCLCPP_WARN(get_node()->get_logger(), "Retreating...");

      phase_ = ControlPhase::RETREAT;
      start_time_ = this->get_node()->now();

      for (size_t robot_index = 0; robot_index < robot_types_.size(); ++robot_index) {
        motion_generators_[robot_index] = std::make_unique<MotionGenerator>(
            kSpeedMotionGenerators, q_[robot_index], q_start_[robot_index]);
      }
      return controller_interface::return_type::OK;
    }
  }

  for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
    auto motion_generator_output =
        motion_generators_[robot_index]->getDesiredJointPositions(trajectory_time);
    Vector7d q_desired = motion_generator_output.first;
    bool finished = motion_generator_output.second;

    all_finished &= finished;

    if (!finished) {
      const double kAlpha = 0.99;
      dq_filtered_[robot_index] =
          (1 - kAlpha) * dq_filtered_[robot_index] + kAlpha * dq_[robot_index];

      Vector7d tau_d_calculated = k_gains_.cwiseProduct(q_desired - q_[robot_index]) +
                                  d_gains_.cwiseProduct(-dq_filtered_[robot_index]);

      for (int i = 0; i < num_joints; ++i) {
        size_t cmd_idx = i * robot_types_.size() + robot_index;
        if (!command_interfaces_[cmd_idx].set_value(tau_d_calculated(i))) {
          RCLCPP_FATAL(get_node()->get_logger(), "Failed to set command interface value");
          return controller_interface::return_type::ERROR;
        }
      }
    } else {
      for (int i = 0; i < num_joints; ++i) {
        size_t cmd_idx = i * robot_types_.size() + robot_index;
        if (!command_interfaces_[cmd_idx].set_value(0.0)) {
          RCLCPP_FATAL(get_node()->get_logger(),
                       "Failure to clear the torque at the end of control. ");
        }
      }
    }
  }

  if (all_finished) {
    if (phase_ == ControlPhase::MOVE_TO_START) {
      RCLCPP_INFO(get_node()->get_logger(), "Start Reached. Move to Collision Configuration.");
      phase_ = ControlPhase::MOVE_TO_COLLISION;
      start_time_ = this->get_node()->now();

      for (size_t robot_index = 0; robot_index < robot_types_.size(); ++robot_index) {
        motion_generators_[robot_index] = std::make_unique<MotionGenerator>(
            kSpeedMotionGenerators, q_start_[robot_index], q_collision_[robot_index]);
      }
    } else if (phase_ == ControlPhase::RETREAT || phase_ == ControlPhase::MOVE_TO_COLLISION) {
      if (phase_ != ControlPhase::FINISHED) {
        RCLCPP_INFO(get_node()->get_logger(), "Sequence Finished.");
        this->get_node()->set_parameter({"process_finished", true});
        phase_ = ControlPhase::FINISHED;
      }
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SelfCollisionFR3DuoExampleController::on_init() {
  try {
    auto_declare<bool>("process_finished", false);
    auto_declare<std::vector<std::string>>("robot_types", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("arm_prefixes", {});
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>(
        "start_joint_configuration",
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4,    // Robot 1
         0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});  // Robot 2

    auto_declare<std::vector<double>>(
        "collision_joint_configuration",
        {0.0, 0.2, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4,    // Robot 1
         0.0, 0.2, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});  // Robot 2

    auto_declare<std::string>("collision_topic", "/self_collision_node/collision_detected");

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SelfCollisionFR3DuoExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_types_ = get_node()->get_parameter("robot_types").as_string_array();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  auto start_joint_configuration_vector =
      get_node()->get_parameter("start_joint_configuration").as_double_array();

  auto collision_joint_configuration_vector =
      get_node()->get_parameter("collision_joint_configuration").as_double_array();

  q_start_.resize(robot_types_.size(), Vector7d::Zero());
  q_collision_.resize(robot_types_.size(), Vector7d::Zero());
  dq_filtered_.resize(robot_types_.size(), Vector7d::Zero());

  for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
    size_t offset = robot_index * num_joints;
    q_start_[robot_index] =
        Eigen::Map<const Vector7d>(start_joint_configuration_vector.data() + offset);

    q_collision_[robot_index] =
        Eigen::Map<const Vector7d>(collision_joint_configuration_vector.data() + offset);
  }

  collision_topic_ = get_node()->get_parameter("collision_topic").as_string();

  collision_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      collision_topic_, 1, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        this->collision_detected_ = msg->data;
        this->last_collision_msg_time_ = this->get_node()->now();
      });

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

  for (auto& dq : dq_filtered_) {
    dq.setZero();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SelfCollisionFR3DuoExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  q_.resize(robot_types_.size(), Vector7d::Zero());
  dq_.resize(robot_types_.size(), Vector7d::Zero());

  updateJointStates();
  motion_generators_.clear();
  for (size_t robot_index = 0; robot_index < robot_types_.size(); ++robot_index) {
    motion_generators_.push_back(std::make_unique<MotionGenerator>(
        kSpeedMotionGenerators, q_[robot_index], q_start_[robot_index]));

    dq_filtered_[robot_index].setZero();
  }

  if (get_node()->count_publishers(collision_topic_) == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Self Collision Node not detected on %s",
                 collision_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  phase_ = ControlPhase::MOVE_TO_START;
  start_time_ = this->get_node()->now();
  last_collision_msg_time_ = this->get_node()->now();
  collision_detected_ = false;

  return CallbackReturn::SUCCESS;
}

void SelfCollisionFR3DuoExampleController::updateJointStates() {
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
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::SelfCollisionFR3DuoExampleController,
                       controller_interface::ControllerInterface)
