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

#include "franka_spine_examples/spine_client_example.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <stdexcept>

using namespace std::chrono_literals;

static constexpr const char* kGetStateServiceName = "/franka_spine_node/get_state";
static constexpr const char* kGetPositionServiceName = "/franka_spine_node/get_position";
static constexpr const char* kSwitchOnServiceName = "/franka_spine_node/switch_on";
static constexpr const char* kSwitchOffServiceName = "/franka_spine_node/switch_off";
static constexpr const char* kMoveActionName = "/franka_spine_node/move_absolute";
static constexpr std::chrono::seconds kDefaultTimeout{5s};

namespace {

template <typename T>
auto waitForService(const rclcpp::Logger& logger,
                    const typename rclcpp::Client<T>::SharedPtr& client,
                    const std::string& name) -> bool {
  if (!client->wait_for_service(kDefaultTimeout)) {
    RCLCPP_ERROR(logger, "Service '%s' not available.", name.c_str());
    return false;
  }

  return true;
}

}  // namespace

namespace franka_spine_examples {

SpineClientExample::SpineClientExample(const rclcpp::NodeOptions& options)
    : rclcpp::Node("spine_client_cpp", options) {
  state_client_ = create_client<GetSpineState>(kGetStateServiceName);
  position_client_ = create_client<GetPosition>(kGetPositionServiceName);
  switch_on_client_ = create_client<SwitchOn>(kSwitchOnServiceName);
  switch_off_client_ = create_client<SwitchOff>(kSwitchOffServiceName);
  move_client_ = rclcpp_action::create_client<MoveAbsolute>(this, kMoveActionName);
  if (!waitForService<GetSpineState>(get_logger(), state_client_, kGetStateServiceName) ||
      !waitForService<GetPosition>(get_logger(), position_client_, kGetPositionServiceName) ||
      !waitForService<SwitchOn>(get_logger(), switch_on_client_, kSwitchOnServiceName) ||
      !waitForService<SwitchOff>(get_logger(), switch_off_client_, kSwitchOffServiceName)) {
    throw std::runtime_error("Timed out waiting for Franka Spine services.");
  }

  if (!move_client_->wait_for_action_server(kDefaultTimeout)) {
    throw std::runtime_error("Timed out waiting for Franka Spine MoveAbsolute action server.");
  }
}

auto SpineClientExample::run() -> void {
  getState();
  getPosition();
  switchOn();
  moveTo(0.4, 0.1, 0.2, 0.2);
  moveTo(0.05, 0.1, 0.2, 0.2);
  switchOff();

  RCLCPP_INFO(get_logger(), "Spine client example completed.");
}

auto SpineClientExample::getState() -> bool {
  auto future = state_client_->async_send_request(std::make_shared<GetSpineState::Request>());
  if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to call get_state.");
    return false;
  }

  auto result = future.get();
  RCLCPP_INFO(get_logger(), "State: %s, success: %s", result->state.c_str(),
              result->success ? "true" : "false");
  return result->success;
}

auto SpineClientExample::getPosition() -> bool {
  auto future = position_client_->async_send_request(std::make_shared<GetPosition::Request>());
  if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to call get_position.");
    return false;
  }

  auto result = future.get();
  RCLCPP_INFO(get_logger(), "Position: %.4f m, success: %s", result->position,
              result->success ? "true" : "false");
  return result->success;
}

auto SpineClientExample::switchOn() -> bool {
  auto future = switch_on_client_->async_send_request(std::make_shared<SwitchOn::Request>());
  if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to call switch_on.");
    return false;
  }

  auto result = future.get();
  RCLCPP_INFO(get_logger(), "Switch on: %s", result->message.c_str());
  return result->success;
}

auto SpineClientExample::switchOff() -> bool {
  auto future = switch_off_client_->async_send_request(std::make_shared<SwitchOff::Request>());
  if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to call switch_off.");
    return false;
  }

  auto result = future.get();
  RCLCPP_INFO(get_logger(), "Switch off: %s", result->message.c_str());
  return result->success;
}

auto SpineClientExample::moveTo(double position,
                                double velocity,
                                double acceleration,
                                double deceleration) -> bool {
  auto goal = MoveAbsolute::Goal();
  goal.position = position;
  goal.velocity = velocity;
  goal.acceleration = acceleration;
  goal.deceleration = deceleration;

  RCLCPP_INFO(get_logger(),
              "Sending move goal: position=%.4f m, velocity=%.4f m/s, "
              "acceleration=%.4f m/s², deceleration=%.4f m/s²",
              position, velocity, acceleration, deceleration);

  rclcpp_action::Client<MoveAbsolute>::SendGoalOptions options;
  options.goal_response_callback =
      [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveAbsolute>> handle) {
        handleMoveGoalResponse(handle);
      };
  options.feedback_callback = [this](
                                  std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveAbsolute>>,
                                  const std::shared_ptr<const MoveAbsolute::Feedback>& feedback) {
    handleMoveFeedback(feedback);
  };

  std::atomic<bool> motion_finished{false};
  bool motion_success = false;
  options.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<MoveAbsolute>::WrappedResult& result) {
        handleMoveResult(result);
        motion_success =
            (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success);
        motion_finished.store(true);
      };

  auto future_handle = move_client_->async_send_goal(goal, options);
  const auto& goal_handle = future_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Move goal was rejected by server.");
    return false;
  }

  while (!motion_finished.load()) {
    rclcpp::sleep_for(10ms);
  }

  return motion_success;
}

auto SpineClientExample::handleMoveGoalResponse(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveAbsolute>>& goal_handle) -> void {
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Move goal was rejected.");
  } else {
    RCLCPP_INFO(get_logger(), "Move goal accepted.");
  }
}

auto SpineClientExample::handleMoveFeedback(
    const std::shared_ptr<const MoveAbsolute::Feedback>& feedback) -> void {
  RCLCPP_INFO(get_logger(), "Position: %.4f m", feedback->current_position);
}

auto SpineClientExample::handleMoveResult(
    const rclcpp_action::ClientGoalHandle<MoveAbsolute>::WrappedResult& result) -> void {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (result.result->success) {
        RCLCPP_INFO(get_logger(), "Motion completed successfully.");
      } else {
        RCLCPP_ERROR(get_logger(), "Motion finished with error: %s", result.result->error.c_str());
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Motion aborted: %s", result.result->error.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "Motion canceled.");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code.");
      break;
  }
}

}  // namespace franka_spine_examples

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<franka_spine_examples::SpineClientExample>();
  auto run_node = std::async(std::launch::async, [&node]() { node->run(); });

  while (run_node.wait_for(1ms) != std::future_status::ready) {
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
