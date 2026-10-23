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

#include <franka_example_controllers/fr3/move_to_start_example_controller.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
MoveToStartExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
MoveToStartExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type MoveToStartExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  auto trajectory_time = this->get_node()->now() - start_time_;
  auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
  Vector7d q_desired = motion_generator_output.first;
  bool finished = motion_generator_output.second;
  if (not finished) {
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    Vector7d tau_d_calculated =
        k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < 7; ++i) {
      if (!command_interfaces_[i].set_value(tau_d_calculated(i))) {
        RCLCPP_FATAL(get_node()->get_logger(), "Failed to set command interface value");
        return controller_interface::return_type::ERROR;
      }
    }
  } else {
    bool ret = true;
    for (auto& command_interface : command_interfaces_) {
      ret &= command_interface.set_value(0.0);
    }
    if (!ret) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failure to clear the torque at the end of control. ");
    }
    // Realtime path: only flip the flag. The wall timer created in on_configure()
    // publishes it as the `process_finished` parameter off this thread.
    process_finished_.store(true, std::memory_order_release);
  }
  return controller_interface::return_type::OK;
}

CallbackReturn MoveToStartExampleController::on_init() {
  try {
    auto_declare<bool>("process_finished", false);
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>("start_joint_configuration",
                                      {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MoveToStartExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  auto start_joint_configuration_vector =
      get_node()->get_parameter("start_joint_configuration").as_double_array();

  Eigen::Map<Eigen::VectorXd>(q_goal_.data(), num_joints) =
      Eigen::Map<Eigen::VectorXd>(start_joint_configuration_vector.data(), num_joints);

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
  dq_filtered_.setZero();
  // Non-realtime context: single reset of the observable contract; every later
  // change is published by the wall timer below. The atomic must go false too, or the
  // new timer republishes the previous run's true on its first tick. Under the mutex:
  // a callback of the dropped timer may still be executing on another thread.
  const std::lock_guard<std::mutex> lock(publish_mutex_);
  process_finished_.store(false, std::memory_order_release);
  get_node()->set_parameter({"process_finished", false});
  published_process_finished_ = false;
  process_finished_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(20),
                                                          [this]() { publishProcessFinished(); });
  return CallbackReturn::SUCCESS;
}

CallbackReturn MoveToStartExampleController::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // The wall timer belongs to the configured state: drop it here so no further callback
  // is scheduled. reset() does not join one already executing — that single stale
  // callback can still publish, but nothing after it.
  process_finished_timer_.reset();
  return CallbackReturn::SUCCESS;
}

void MoveToStartExampleController::publishProcessFinished() {
  const std::lock_guard<std::mutex> lock(publish_mutex_);
  const bool finished = process_finished_.load(std::memory_order_acquire);
  if (published_process_finished_ != finished &&
      get_node()->set_parameter({"process_finished", finished}).successful) {
    published_process_finished_ = finished;
  }
}

CallbackReturn MoveToStartExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_goal_);
  start_time_ = this->get_node()->now();
  // Realtime-safe reset: the timer picks it up on its next tick (see header contract).
  process_finished_.store(false, std::memory_order_release);
  return CallbackReturn::SUCCESS;
}

void MoveToStartExampleController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_optional().value();
    dq_(i) = velocity_interface.get_optional().value();
  }
}
}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MoveToStartExampleController,
                       controller_interface::ControllerInterface)
