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
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The mobile FR3 duo joint impedance example controller combines:
 * - Dual arm joint impedance control (like fr3_duo)
 * - Mobile base cartesian velocity control
 * This controller moves joint 4 and 5 of both arms in a compliant periodic movement
 * while controlling the mobile base velocity.
 */
class MobileFr3DuoJointImpedanceExampleController
    : public controller_interface::ControllerInterface {
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
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // Dual arm parameters
  std::vector<std::string> robot_types_;
  std::vector<std::string> arm_prefixes_;
  std::vector<std::string> robot_prefixes_;
  std::string robot_description_;

  // TMR cartesian velocity interface
  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface> franka_cartesian_velocity_;
  const int num_arm_joints = 7;
  const int num_base_joints = 4;
  const int kArmStateInterfaces = 7 * 2;
  const int kArmCommandInterfaces = 7;
  const int kBaseCommandInterfacesSimulation = 4;
  const int kBaseStateInterfacesSimulation = 4;
  const int kBaseCommandInterfacesHardware = 6;
  const int kBaseStateInterfacesHardware = 8;
  int kBaseCommandInterfaces_;
  int kBaseStateInterfaces_;

  std::vector<Vector7d> q_;
  std::vector<Vector7d> initial_q_;
  std::vector<Vector7d> dq_;
  std::vector<Vector7d> dq_filtered_;

  Vector7d k_gains_;
  Vector7d d_gains_;
  double elapsed_time_{0.0};

  // Mobile base velocity parameters
  const double k_mobile_time_max_{8.0};  // Longer period for mobile base
  const double k_mobile_v_max_{0.1};     // Max linear velocity (m/s)
  const double k_mobile_angle_{0.0};     // Move forward/backward

  // Helper methods
  void updateJointStates();
  void updateMobileBaseCommand(const rclcpp::Duration& period);
};

}  // namespace franka_example_controllers
