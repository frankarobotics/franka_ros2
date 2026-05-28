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

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <franka_spine_msgs/action/move_absolute.hpp>
#include <franka_spine_msgs/srv/get_position.hpp>
#include <franka_spine_msgs/srv/get_spine_state.hpp>
#include <franka_spine_msgs/srv/switch_off.hpp>
#include <franka_spine_msgs/srv/switch_on.hpp>

namespace franka_spine_examples {

/**
 * Example node that demonstrates how to use the Franka Spine action/service server.
 *
 * The node provides methods to:
 * - Query the current spine state and position
 * - Switch the spine on and off
 * - Send a MoveAbsolute goal with position feedback
 */
class SpineClientExample : public rclcpp::Node {
 public:
  using MoveAbsolute = franka_spine_msgs::action::MoveAbsolute;
  using GetSpineState = franka_spine_msgs::srv::GetSpineState;
  using GetPosition = franka_spine_msgs::srv::GetPosition;
  using SwitchOn = franka_spine_msgs::srv::SwitchOn;
  using SwitchOff = franka_spine_msgs::srv::SwitchOff;

  explicit SpineClientExample(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Runs the example sequence: query state, switch on, move, switch off.
   */
  auto run() -> void;

 private:
  /**
   * @brief Get the current state of the spine
   *
   * @return true if the service call was successful and the spine state was retrieved
   */
  auto getState() -> bool;

  /**
   * @brief Get the position of the spine
   *
   * @return true if the service call was successful and the position was retrieved
   */
  auto getPosition() -> bool;

  /**
   * @return true if the spine was successfully switched on
   */
  auto switchOn() -> bool;

  /**
   * @return true if the spine was successfully switched off
   */
  auto switchOff() -> bool;

  /**
   * @brief Move to the given coordinates
   *
   * @param position Target position in m
   * @param velocity Maximum velocity in m/s (default: 0.1)
   * @param acceleration Maximum acceleration in m/s² (default: 0.2)
   * @param deceleration Maximum deceleration in m/s² (default: 0.2)
   * @return true if the motion was successful
   */
  auto moveTo(double position,
              double velocity = 0.1,
              double acceleration = 0.2,
              double deceleration = 0.2) -> bool;

  // Callbacks for the action client
  auto handleMoveGoalResponse(
      const std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveAbsolute>>& goal_handle) -> void;
  auto handleMoveFeedback(const std::shared_ptr<const MoveAbsolute::Feedback>& feedback) -> void;
  auto handleMoveResult(const rclcpp_action::ClientGoalHandle<MoveAbsolute>::WrappedResult& result)
      -> void;

  rclcpp::Client<GetSpineState>::SharedPtr state_client_;
  rclcpp::Client<GetPosition>::SharedPtr position_client_;
  rclcpp::Client<SwitchOn>::SharedPtr switch_on_client_;
  rclcpp::Client<SwitchOff>::SharedPtr switch_off_client_;
  rclcpp_action::Client<MoveAbsolute>::SharedPtr move_client_;
};

}  // namespace franka_spine_examples
