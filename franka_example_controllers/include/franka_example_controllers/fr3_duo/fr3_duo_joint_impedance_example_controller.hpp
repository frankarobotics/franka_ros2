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

#pragma once

#include <atomic>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The joint impedance FR3 duo example controller moves joint 4 and 5 in a very compliant periodic
 * movement.
 */
class JointImpedanceFr3DuoExampleController : public controller_interface::ControllerInterface {
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

 private:
  std::vector<std::string> robot_types_;
  std::vector<std::string> arm_prefixes_;
  std::string robot_description_;
  const int num_joints = 7;

  std::vector<Vector7d> q_;
  std::vector<Vector7d> initial_q_;
  std::vector<Vector7d> dq_;
  std::vector<Vector7d> dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  double elapsed_time_{0.0};

  // Self-collision checking
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  std::atomic<bool> collision_detected_{false};
  std::atomic<int64_t> last_collision_msg_time_ns_{0};
  static constexpr auto kCollisionTopic = "collision_detected";

  void updateJointStates();
};

}  // namespace franka_example_controllers
