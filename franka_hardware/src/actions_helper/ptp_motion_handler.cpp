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

#include "franka_hardware/actions_helper/ptp_motion_handler.hpp"

namespace franka_hardware {

PTPMotionHandler::PTPMotionHandler(const std::shared_ptr<Robot>& robot)
    : franka_hardware_robot_(robot) {}

PTPMotionHandler::~PTPMotionHandler() {
  cancelMotion();
}

auto PTPMotionHandler::startNewPTPMotion(
    const std::shared_ptr<franka::Robot>& robot,
    const std::shared_ptr<const franka_msgs::action::PTPMotion::Goal>& goal) -> CommandResult {
  CommandResult command_result = CommandResult{
      .motion_id = "", .result = std::make_shared<franka_msgs::action::PTPMotion::Result>()};

  // Don't construct a new position control handler if a motion is already ongoing - we will
  // overwrite the target
  if (position_control_handler_ == nullptr) {
    auto configuration = franka::AsyncPositionControlHandler::Configuration{
        .maximum_joint_velocities = goal->maximum_joint_velocities,
        .goal_tolerance = goal->goal_tolerance};

    const auto& configuration_result =
        franka::AsyncPositionControlHandler::configure(robot, configuration);
    if (configuration_result.error_message.has_value()) {
      command_result.result->target_status.status = franka_msgs::msg::TargetStatus::ABORTED;
      command_result.result->error_message = configuration_result.error_message.value();

      return command_result;
    }
    position_control_handler_ = configuration_result.handler;
  }

  auto [new_motion_id, execute_motion_feedback] = executeMotion(goal->goal_joint_configuration);
  if (execute_motion_feedback.has_value() && execute_motion_feedback->error_message.has_value()) {
    command_result.result->target_status.status = franka_msgs::msg::TargetStatus::ABORTED;
    command_result.result->error_message = execute_motion_feedback->error_message.value_or(
        "PTP motion aborted without specific error message.");

    // Reset the position control handler on failure to avoid calling stopControl()
    // on a potentially invalid handler during destruction.
    position_control_handler_.reset();

    return command_result;
  }

  command_result.motion_id = new_motion_id;
  command_result.result->target_status.status = franka_msgs::msg::TargetStatus::IDLE;

  return command_result;
}

auto PTPMotionHandler::executeMotion(const std::vector<double>& goal_joint_configuration)
    -> std::tuple<std::string, std::optional<franka::AsyncPositionControlHandler::TargetFeedback>> {
  if (position_control_handler_ == nullptr) {
    franka::AsyncPositionControlHandler::TargetFeedback feedback{};
    feedback.status = franka::TargetStatus::kAborted;
    feedback.error_message = "Position control handler is not initialized.";

    return {"", feedback};
  }

  franka::AsyncPositionControlHandler::JointPositionTarget target;
  if (target.joint_positions.size() != goal_joint_configuration.size()) {
    franka::AsyncPositionControlHandler::TargetFeedback feedback{};
    feedback.status = franka::TargetStatus::kAborted;
    feedback.error_message = "Goal joint configuration size does not match number of joints.";

    return {"", feedback};
  }

  std::copy(goal_joint_configuration.cbegin(), goal_joint_configuration.cend(),
            target.joint_positions.begin());

  auto target_feedback = std::optional<franka::AsyncPositionControlHandler::TargetFeedback>{};
  // If we already have an ongoing motion, we clear the map
  if (feedback_futures_.count(std::to_string(motion_id_)) > 0) {
    feedback_futures_.clear();
  }

  auto new_motion_id = std::to_string(++motion_id_);

  feedback_futures_[new_motion_id] = std::async(
      std::launch::async, [this, robot_ptr = franka_hardware_robot_,
                           handler_ptr = position_control_handler_, target = std::move(target)]() {
        auto command_result = handler_ptr->setJointPositionTarget(target);

        if (!command_result.was_successful && command_result.error_message.has_value()) {
          franka::AsyncPositionControlHandler::TargetFeedback feedback{};
          feedback.status = franka::TargetStatus::kAborted;
          feedback.error_message = command_result.error_message;

          return feedback;
        }

        while (running_ && handler_ptr != nullptr) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));

          auto target_feedback = handler_ptr->getTargetFeedback(robot_ptr->getCurrentState());

          {
            std::lock_guard<std::mutex> lock(control_mutex_);
            last_feedback_ = target_feedback;

            if (last_feedback_.error_message.has_value()) {
              return last_feedback_;
            }

            switch (last_feedback_.status) {
              case franka::TargetStatus::kTargetReached:
              case franka::TargetStatus::kAborted:
                handler_ptr->stopControl();
                return last_feedback_;
              default:
                break;
            }
          }
        }

        return last_feedback_;
      });

  return {new_motion_id, target_feedback};
}

auto PTPMotionHandler::cancelMotion() -> void {
  running_ = false;

  feedback_futures_.clear();

  if (position_control_handler_ != nullptr) {
    try {
      position_control_handler_->stopControl();
    } catch (const std::exception& e) {
      // stopControl may fail if the control handler was never fully initialized
      // (e.g. in tests with mocked robots). This is safe to ignore during cleanup.
    }
    position_control_handler_.reset();
  }
}

auto PTPMotionHandler::getFeedback(const std::string& motion_id)
    -> franka::AsyncPositionControlHandler::TargetFeedback {
  if (feedback_futures_.count(motion_id) > 0 && feedback_futures_[motion_id].valid()) {
    auto status = feedback_futures_[motion_id].wait_for(std::chrono::milliseconds(0));
    std::lock_guard<std::mutex> lock(control_mutex_);
    switch (status) {
      case std::future_status::deferred:
        throw std::logic_error("feedback_future_ is deferred; async execution expected");
      case std::future_status::timeout:
        // Still working
        if (last_feedback_.status == franka::TargetStatus::kTargetReached) {
          // Race condition: The target can be reached even if the future is not ready yet (aka
          // lambda didn't finish yet). Cycle another time.
          return franka::AsyncPositionControlHandler::TargetFeedback{
              .status = franka::TargetStatus::kExecuting};
        }
        return last_feedback_;
      case std::future_status::ready: {
        auto feedback = feedback_futures_[motion_id].get();
        feedback_futures_.erase(motion_id);
        position_control_handler_.reset();
        return feedback;
      }
    }
  }

  auto target_feedback = franka::AsyncPositionControlHandler::TargetFeedback{};
  target_feedback.status = franka::TargetStatus::kAborted;
  target_feedback.error_message =
      "No motion found with the given motion ID. Might be already finished or aborted.";
  return target_feedback;
}

}  // namespace franka_hardware
