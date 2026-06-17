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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <franka_gazebo_hardware/gravity_compensation_model.hpp>
#include <gz/sim/EntityComponentManager.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace franka_gazebo_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * Gazebo ros2_control hardware component that wraps the stock
 * gz_ros2_control::GazeboSimSystem and only adds model-based gravity
 * compensation on top of the effort commands of the simulated joints.
 *
 * The wrapped system is loaded through pluginlib and handles all joint and
 * sensor logic. Every hardware-interface call is forwarded to it unchanged;
 * this class only builds the gravity model in initSim() and adds the gravity
 * torque to the effort command of every effort-controlled joint in write().
 */
class GazeboGravityCompensationSystem : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  auto initSim(rclcpp::Node::SharedPtr& model_nh,
               std::map<std::string, sim::Entity>& joints,
               const hardware_interface::HardwareInfo& hardware_info,
               sim::EntityComponentManager& ecm,
               unsigned int update_rate) -> bool override;

  auto on_init(const hardware_interface::HardwareInfo& system_info) -> CallbackReturn override;

  auto on_configure(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;

  auto on_deactivate(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;

  auto export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;

  auto export_command_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                   const std::vector<std::string>& stop_interfaces)
      -> hardware_interface::return_type override;

  auto read(const rclcpp::Time& time,
            const rclcpp::Duration& period) -> hardware_interface::return_type override;

  auto write(const rclcpp::Time& time,
             const rclcpp::Duration& period) -> hardware_interface::return_type override;

 private:
  /**
   * Builds the gravity model and the mapping from simulated joint entities to
   * their position/effort role in that model.
   *
   * @param hardware_info Parsed ros2_control hardware description (URDF + interfaces)
   * @param joints Map from simulated joint name to its Gazebo entity
   */
  auto initGravityModel(const hardware_interface::HardwareInfo& hardware_info,
                        const std::map<std::string, sim::Entity>& joints) -> void;

  // Loader must outlive the wrapped system, so it is declared first.
  std::shared_ptr<pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>> system_loader_;
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> wrapped_system_;

  sim::EntityComponentManager* ecm_ = nullptr; /**< Not owned; valid for the sim's lifetime. */
  rclcpp::Node::SharedPtr model_node_; /**< Owns the logger/clock for write()'s error log. */
  GravityCompensationModel gravity_model_;
  // Aligned with gravity_model_.configurationJoints(): each entity's position is
  // read from the ECM and fed to the gravity model in the same order.
  std::vector<sim::Entity> configuration_entities_;
  // Aligned with gravity_model_.effortJoints(): each entity receives the computed
  // gravity torque on its effort command in the same order.
  std::vector<sim::Entity> effort_entities_;
  // Per-cycle buffers, sized once in initGravityModel() so write() never allocates.
  std::vector<double> joint_positions_;
  std::vector<double> gravity_torques_;
};

}  // namespace franka_gazebo_hardware
