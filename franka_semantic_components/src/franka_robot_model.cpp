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

#include "franka_semantic_components/franka_robot_model.hpp"

#include <cstring>
#include <iostream>

#include <realtime_tools/realtime_buffer.hpp>
#include "rclcpp/logging.hpp"
namespace {

// Example implementation of bit_cast: https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable<From>::value &&
                     std::is_trivially_copyable<To>::value,
                 To>
bit_cast(const From& src) noexcept {
  static_assert(std::is_trivially_constructible<To>::value,
                "This implementation additionally requires "
                "destination type to be trivially constructible");

  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

}  // namespace

namespace franka_semantic_components {

FrankaRobotModel::FrankaRobotModel(const std::string& franka_model_interface_name,
                                   const std::string& franka_state_interface_name)
    : SemanticComponentInterface(franka_model_interface_name, 2) {
  franka_model_interface_name_ = franka_model_interface_name;
  franka_state_interface_name_ = franka_state_interface_name;
  interface_names_.emplace_back(franka_model_interface_name);
  interface_names_.emplace_back(franka_state_interface_name);
}

auto FrankaRobotModel::initialize() -> void {
  auto franka_state_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
        return interface.get().get_name() == franka_state_interface_name_;
      });

  auto franka_model_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
        return interface.get().get_name() == franka_model_interface_name_;
      });

  if (franka_state_interface != state_interfaces_.end() &&
      franka_model_interface != state_interfaces_.end()) {
    robot_model_ =
        bit_cast<franka_hardware::Model*>((*franka_model_interface).get().get_optional().value());
    robot_state_buffer_ = bit_cast<realtime_tools::RealtimeBuffer<franka::RobotState>*>(
        (*franka_state_interface).get().get_optional().value());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("franka_model_semantic_component"),
                 "Franka interface does not exist! Did you assign the loaned state in the "
                 "controller?");
    throw std::runtime_error("Franka state interfaces does not exist");
  }
  initialized_ = true;
}

auto FrankaRobotModel::getMassMatrix() -> std::array<double, 49> {
  if (!initialized_) {
    initialize();
  }
  return robot_model_->mass(getCachedRobotState());
}

auto FrankaRobotModel::getCoriolisForceVector() -> std::array<double, 7> {
  if (!initialized_) {
    initialize();
  }
  return robot_model_->coriolis(getCachedRobotState());
}

auto FrankaRobotModel::getGravityForceVector() -> std::array<double, 7> {
  if (!initialized_) {
    initialize();
  }
  return robot_model_->gravity(getCachedRobotState());
}

auto FrankaRobotModel::getPoseMatrix(const franka::Frame& frame) -> std::array<double, 16> {
  if (!initialized_) {
    initialize();
  }
  return robot_model_->pose(frame, getCachedRobotState());
}

auto FrankaRobotModel::getBodyJacobian(const franka::Frame& frame) -> std::array<double, 42> {
  if (!initialized_) {
    initialize();
  }
  return robot_model_->bodyJacobian(frame, getCachedRobotState());
}

auto FrankaRobotModel::getZeroJacobian(const franka::Frame& frame) -> std::array<double, 42> {
  if (!initialized_) {
    initialize();
  }
  return robot_model_->zeroJacobian(frame, getCachedRobotState());
}

auto FrankaRobotModel::refreshRobotState() -> void {
  if (!initialized_) {
    initialize();
  }
  cached_robot_state_ = *robot_state_buffer_->readFromRT();
  last_refresh_time_ = std::chrono::steady_clock::now();
}

auto FrankaRobotModel::getCachedRobotState() -> const franka::RobotState& {
  auto now = std::chrono::steady_clock::now();
  if (now - last_refresh_time_ > kCacheMaxAge) {
    refreshRobotState();
  }
  return cached_robot_state_;
}

}  // namespace franka_semantic_components
