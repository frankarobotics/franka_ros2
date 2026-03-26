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

#include <controller_interface/helpers.hpp>
#include <franka_mobile/swerve_ik_controller.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>

#include <algorithm>

#include <franka_mobile/odometry.hpp>
#include <franka_mobile/swerve_kinematics.hpp>
#include "urdf_utils.hpp"

namespace franka_mobile {

using franka_semantic_components::FrankaCartesianVelocityInterface;

controller_interface::CallbackReturn SwerveIKController::on_init() {
  prefix_ = auto_declare<std::string>("prefix", "");

  const std::string wheel_1_link_name = auto_declare("wheel_1_link_name", "argo_drive_front_link");
  const std::string wheel_2_link_name = auto_declare("wheel_2_link_name", "argo_drive_rear_link");
  const std::string base_link_name = auto_declare("base_link_name", "base_link");
  const size_t velocity_rolling_window_size = auto_declare("velocity_rolling_window_size", 10);

  const std::string robot_description = auto_declare("robot_description", "");
  const SE3 wheel_position_1 =
      getSe3FromDescription(robot_description, base_link_name, wheel_1_link_name);
  const SE3 wheel_position_2 =
      getSe3FromDescription(robot_description, base_link_name, wheel_2_link_name);
  const double wheel_radius = getWheelRadiusFromDescription(robot_description, wheel_1_link_name);
  const std::array<Eigen::Vector2d, 2> wheel_positions{wheel_position_1.p.head<2>(),
                                                       wheel_position_2.p.head<2>()};
  swerve_kinematics_ = std::make_unique<SwerveKinematics>(wheel_positions, wheel_radius);
  odometry_ = std::make_unique<Odometry>(velocity_rolling_window_size);

  odometry_->init(get_node()->now());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveIKController::command_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = {prefix_ + "joint_0/position", prefix_ + "joint_1/velocity",
                                     prefix_ + "joint_2/position", prefix_ + "joint_3/velocity"};

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SwerveIKController::state_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration state_interface_configuration;
  state_interface_configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_configuration.names = {prefix_ + "joint_0/position", prefix_ + "joint_1/velocity",
                                         prefix_ + "joint_2/position",
                                         prefix_ + "joint_3/velocity"};

  return state_interface_configuration;
}

controller_interface::CallbackReturn SwerveIKController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveIKController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(),
            std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveIKController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

bool SwerveIKController::on_set_chained_mode(bool /*chained_mode*/) {
  return true;
}

controller_interface::return_type SwerveIKController::update_and_write_commands(
    const rclcpp::Time& ,
    const rclcpp::Duration& /*period*/) {
  // update commands
  {
    const double vx = reference_interfaces_[FrankaCartesianVelocityInterface::VX];
    const double vy = reference_interfaces_[FrankaCartesianVelocityInterface::VY];
    const double wz = reference_interfaces_[FrankaCartesianVelocityInterface::WZ];
    std::array<double, 2> steering_angles{0, 0}, wheel_speeds{0, 0};
    if (!swerve_kinematics_->inverseKinematics(vx, vy, wz, steering_angles, wheel_speeds)) {
      return controller_interface::return_type::OK;  // do nothing
    }

    for (size_t i = 0; i < 2; ++i) {
      command_interfaces_[2 * i].set_value(steering_angles[i]);
      command_interfaces_[2 * i + 1].set_value(wheel_speeds[i]);
    }
  }

// in humble we cannot export state interfaces, thus we cannot update 
// the mocked cartesian pose state interface
#if RCLCPP_VERSION_MAJOR > 16 
  // update states
  {
    const double estimate_steering_position_wheel_1 = state_interfaces_[0].get_value();
    const double estimate_drive_velocity_wheel_1 = state_interfaces_[1].get_value();

    const double estimate_steering_position_wheel_2 = state_interfaces_[2].get_value();
    const double estimate_drive_velocity_wheel_2 = state_interfaces_[3].get_value();

    const std::array<double, 2> steerings{estimate_steering_position_wheel_1,
                                          estimate_steering_position_wheel_2};
    const std::array<double, 2> velocities{estimate_drive_velocity_wheel_1,
                                           estimate_drive_velocity_wheel_2};

    double vx = 0, vy = 0, wz = 0;
    swerve_kinematics_->forwardKinematics(steerings, velocities, vx, vy, wz);
    odometry_->update(vx, vy, wz, time);

    const Eigen::Vector3d p{odometry_->getX(), odometry_->getY(), 0};
    const Eigen::Quaterniond q{
        Eigen::AngleAxisd{odometry_->getHeading(), Eigen::Vector3d::UnitZ()}};
    const std::vector<double> values = franka_semantic_components::FrankaCartesianPoseInterface::
        createColumnMajorTransformationMatrix(q, p);
    std::copy(values.begin(), values.begin() + 16, state_interfaces_values_.begin());
  }
#endif

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
SwerveIKController::on_export_reference_interfaces() {
  reference_interfaces_.resize(6, 0.0);

  std::vector<hardware_interface::CommandInterface> interfaces;
  franka_semantic_components::FrankaCartesianVelocityInterface interface(false);
  const std::vector<std::string> command_interface_names = interface.get_command_interface_names();
  if (command_interface_names.size() != 6) {
    throw std::invalid_argument("Exported reference interfaces must be 6 for cartesian velocity");
  }

  for (size_t i = 0; i < command_interface_names.size(); ++i) {
    interfaces.emplace_back(get_node()->get_name(), command_interface_names[i],
                            &reference_interfaces_[i]);
  }

  return interfaces;
}

#if RCLCPP_VERSION_MAJOR > 16 // for humble, this is not available
std::vector<hardware_interface::StateInterface> SwerveIKController::on_export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  franka_semantic_components::FrankaCartesianPoseInterface interface(false);
  const std::vector<std::string> state_interface_names = interface.get_state_interface_names();
  state_interfaces_values_.resize(state_interface_names.size());

  if (state_interface_names.size() != 16) {
    throw std::invalid_argument("Exported reference interfaces must be 16 for cartesian pose");
  }

  for (size_t i = 0; i < state_interface_names.size(); ++i) {
    interfaces.emplace_back(get_node()->get_name(), state_interface_names[i],
                            &state_interfaces_values_[i]);
  }

  return interfaces;
}

controller_interface::return_type SwerveIKController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  // NO OP, we are always chaining this controller for simulation on gazebo
  return controller_interface::return_type::OK;
}
#else

controller_interface::return_type SwerveIKController::update_reference_from_subscribers() {
  // NO OP, we are always chaining this controller for simulation on gazebo
  return controller_interface::return_type::OK;
}
#endif

}  // namespace franka_mobile

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_mobile::SwerveIKController,
                       controller_interface::ChainableControllerInterface)