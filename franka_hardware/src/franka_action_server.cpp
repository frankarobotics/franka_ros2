// Copyright (c) 2024 Franka Robotics GmbH
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

#include "franka_hardware/franka_action_server.hpp"

namespace franka_hardware {

ActionServer::ActionServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot)
    : rclcpp::Node("action_server", options), ptp_motion_handler_(robot), robot_(robot) {
  error_recovery_action_server_ = rclcpp_action::create_server<franka_msgs::action::ErrorRecovery>(
      this, "~/error_recovery",
      [](auto /*uuid*/, auto /*goal*/) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](const auto& /*goal_handle*/) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto& goal_handle) { errorRecoveryAction(goal_handle); });

  ptp_motion_action_server_ = rclcpp_action::create_server<franka_msgs::action::PTPMotion>(
      this, "~/ptp_motion",
      [](auto /*uuid*/, auto /*goal*/) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [this](const auto& /*goal_handle*/) {
        ptp_motion_handler_.cancelMotion();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const auto& goal_handle) { ptpMotionAction(goal_handle); });

  RCLCPP_INFO(get_logger(), "Action server started");
}

auto ActionServer::errorRecoveryAction(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>>&
        goal_handle) -> void {
  auto result = std::make_shared<franka_msgs::action::ErrorRecovery::Result>();
  try {
    robot_->automaticErrorRecovery();
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Automatic recovery succeeded");
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during automatic error recovery: %s", e.what());
    goal_handle->abort(result);
  }
}

auto ActionServer::ptpMotionAction(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::PTPMotion>>&
        goal_handle) -> void {
  try {
    auto command_result =
        ptp_motion_handler_.startNewPTPMotion(robot_->getRobot(), goal_handle->get_goal());
    if (command_result.result->target_status.status == franka_msgs::msg::TargetStatus::ABORTED) {
      goal_handle->abort(command_result.result);
      RCLCPP_ERROR(this->get_logger(), "Could not start new PTP motion: %s",
                   command_result.result->error_message.c_str());
      return;
    }

    auto current_motion = command_result.motion_id;

    auto is_finished = false;
    auto feedback_message = std::make_shared<franka_msgs::action::PTPMotion::Feedback>();
    while (!is_finished) {
      rclcpp::sleep_for(std::chrono::milliseconds(1));
      auto feedback = ptp_motion_handler_.getFeedback(current_motion);
      switch (feedback.status) {
        case franka::TargetStatus::kAborted:
          RCLCPP_ERROR(this->get_logger(), "PTP motion aborted: %s",
                       feedback.error_message.value_or("unknown reason").c_str());
          command_result.result->target_status.status = franka_msgs::msg::TargetStatus::ABORTED;
          command_result.result->error_message =
              feedback.error_message.value_or("PTP motion aborted");
          goal_handle->abort(command_result.result);
          return;
        case franka::TargetStatus::kIdle:
        case franka::TargetStatus::kExecuting: {
          feedback_message->target_status.status = static_cast<uint8_t>(feedback.status);
          goal_handle->publish_feedback(feedback_message);
          break;
        }
        case franka::TargetStatus::kTargetReached: {
          is_finished = true;
          break;
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "PTP motion completed successfully");
    command_result.result->target_status.status = franka_msgs::msg::TargetStatus::TARGET_REACHED;
    goal_handle->succeed(command_result.result);
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during PTP motion: %s", e.what());
    auto result = std::make_shared<franka_msgs::action::PTPMotion::Result>();
    result->target_status.status = franka_msgs::msg::TargetStatus::ABORTED;
    result->error_message = e.what();
    goal_handle->abort(result);
  }
}

}  // namespace franka_hardware
