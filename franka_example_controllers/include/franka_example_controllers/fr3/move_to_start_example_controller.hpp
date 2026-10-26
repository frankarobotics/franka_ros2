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

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_example_controllers/motion_generator.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/// The move to start example controller moves the robot into default pose.
class MoveToStartExampleController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string robot_type_;
  std::string arm_prefix_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d q_goal_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  rclcpp::Time start_time_;
  std::unique_ptr<MotionGenerator> motion_generator_;

  // Requested value. Written on the realtime thread (update() on finish, on_activate()
  // on (re-)activation — both on the controller_manager update thread, so lock-free)
  // and in on_configure() under publish_mutex_. A wall timer publishes it as the
  // `process_finished` parameter off the realtime thread: set_parameter() takes the
  // node parameter mutex and emits /parameter_events over DDS, which blocked the 1 kHz update for
  // ~337 ms when a ros2 CLI participant joined and tripped a
  // communication_constraints_violation reflex.
  //
  // Contract: on_configure() resets the parameter to false synchronously. After a
  // bare deactivate/activate without reconfigure, the previous value can persist for
  // up to one timer period (~20 ms, longer if the executor is busy) before it reads
  // false; then true once the motion finishes.
  std::atomic<bool> process_finished_{false};
  // Guards published_process_finished_ and the parameter write below. The timer
  // callback and on_configure() run on different executor threads, and reset() does
  // not join a callback already executing; without this, a stale callback could
  // republish true after the reset. Never taken on the realtime thread.
  std::mutex publish_mutex_;
  // Last value the timer published successfully; published on difference with retry,
  // so a failed write is reported by the next period.
  std::optional<bool> published_process_finished_;
  rclcpp::TimerBase::SharedPtr process_finished_timer_;

  void updateJointStates();
  void publishProcessFinished();
};
}  // namespace franka_example_controllers
