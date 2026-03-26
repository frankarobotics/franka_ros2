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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <controller_interface/chainable_controller_interface.hpp>

namespace franka_mobile {

class Odometry;
class SwerveKinematics;
/**
 * SwerveIK is a simple chainable controller that performs IK for TMR. Currently only used for
 * gazebo simulation.
 */
class SwerveIKController : public controller_interface::ChainableControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_and_write_commands(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

 protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
#if RCLCPP_VERSION_MAJOR > 16 // for humble, this is not available
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
  controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;
#else
  controller_interface::return_type update_reference_from_subscribers() override;
#endif

 private:
  std::unique_ptr<SwerveKinematics> swerve_kinematics_;
  std::unique_ptr<Odometry> odometry_;

  std::string prefix_;
};

}  // namespace franka_mobile