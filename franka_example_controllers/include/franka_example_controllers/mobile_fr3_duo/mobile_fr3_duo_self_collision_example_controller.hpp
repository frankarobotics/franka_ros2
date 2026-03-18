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

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "franka_example_controllers/motion_generator.hpp"

#include "franka_example_controllers/fr3_duo/fr3_duo_self_collision_example_controller.hpp"  // for ControlPhase
#include "franka_example_controllers/motion_generator.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

// ControlPhase is shared with the fr3 duo controller — no redefinition needed
// if both headers are included together, but each translation unit is independent.

/// Example controller for the mobile FR3 duo that demonstrates self-collision
/// avoidance: moves both arms toward a colliding configuration, detects the
/// collision via the mobile self-collision node, then retreats to start.
/// The spine joint is included in the command/state interfaces and held at a
/// fixed position. Base and drivetrain joints are not controlled here.
class SelfCollisionMobileFR3DuoExampleController
    : public controller_interface::ControllerInterface {
 public:
  // Arm joints only — spine is handled separately as a scalar.
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
  // Arm prefixes: expected to be {"left", "right"} for the mobile duo.
  std::vector<std::string> arm_prefixes_;
  // Robot type suffix, e.g. "fr3v2" — shared across both arms.
  std::string robot_type_;
  const int num_arm_joints_ = 7;

  // Per-arm state
  std::vector<Vector7d> q_;
  std::vector<Vector7d> dq_;
  std::vector<Vector7d> dq_filtered_;

  std::vector<Vector7d> q_start_;
  std::vector<Vector7d> q_collision_;

  // Spine joint — single prismatic/revolute joint shared between arms.
  double q_spine_{0.0};
  double dq_spine_{0.0};
  double dq_spine_filtered_{0.0};
  double q_spine_start_{0.0};
  double k_gain_spine_{0.0};
  double d_gain_spine_{0.0};

  // Arm gains
  Vector7d k_gains_;
  Vector7d d_gains_;

  rclcpp::Time start_time_;
  std::vector<std::unique_ptr<MotionGenerator>> motion_generators_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Time last_collision_msg_time_;
  bool collision_detected_{false};
  ControlPhase phase_;

  const double kSpeedMotionGenerators = 0.2;
  std::string collision_topic_;

  // Name of the spine joint as it appears in the hardware interface.
  std::string spine_joint_name_;

  void updateJointStates();
};

}  // namespace franka_example_controllers