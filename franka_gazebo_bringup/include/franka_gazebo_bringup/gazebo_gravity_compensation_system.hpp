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

#include <Eigen/Core>
#include <gz/sim/EntityComponentManager.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace franka_gazebo_bringup {

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
  bool initSim(rclcpp::Node::SharedPtr& model_nh,
               std::map<std::string, sim::Entity>& joints,
               const hardware_interface::HardwareInfo& hardware_info,
               sim::EntityComponentManager& ecm,
               unsigned int update_rate) override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  // Builds the pinocchio gravity model and the mapping from simulated joints to
  // their pinocchio configuration/velocity indices.
  void initGravityModel(const hardware_interface::HardwareInfo& hardware_info,
                        const std::map<std::string, sim::Entity>& joints);

  // A simulated joint whose position contributes to the gravity configuration.
  struct PositionJoint {
    sim::Entity entity = sim::kNullEntity;
    int configuration_index = 0;
    bool is_continuous = false;
  };

  // A simulated joint that receives gravity compensation on its effort command.
  struct EffortJoint {
    sim::Entity entity = sim::kNullEntity;
    int velocity_index = 0;
  };

  // Loader must outlive the wrapped system, so it is declared first.
  std::shared_ptr<pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>> system_loader_;
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> wrapped_system_;

  sim::EntityComponentManager* ecm_ = nullptr;
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd joint_configuration_;
  std::vector<PositionJoint> position_joints_;
  std::vector<EffortJoint> effort_joints_;
};

}  // namespace franka_gazebo_bringup