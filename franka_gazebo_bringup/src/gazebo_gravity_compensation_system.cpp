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

#include <franka_gazebo_bringup/gazebo_gravity_compensation_system.hpp>

#include <cmath>
#include <set>
#include <string>

#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace franka_gazebo_bringup {

namespace {
constexpr char kSystemPackage[] = "gz_ros2_control";
constexpr char kSystemBaseClass[] = "gz_ros2_control::GazeboSimSystemInterface";
constexpr char kWrappedSystemClass[] = "gz_ros2_control/GazeboSimSystem";
}  // namespace

bool GazeboGravityCompensationSystem::initSim(rclcpp::Node::SharedPtr& model_nh,
                                              std::map<std::string, sim::Entity>& joints,
                                              const hardware_interface::HardwareInfo& hardware_info,
                                              sim::EntityComponentManager& ecm,
                                              unsigned int update_rate) {
  ecm_ = &ecm;

  system_loader_ =
      std::make_shared<pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>>(
          kSystemPackage, kSystemBaseClass);
  wrapped_system_ = system_loader_->createSharedInstance(kWrappedSystemClass);

  if (!wrapped_system_->initSim(model_nh, joints, hardware_info, ecm, update_rate)) {
    return false;
  }

  initGravityModel(hardware_info, joints);
  return true;
}

void GazeboGravityCompensationSystem::initGravityModel(
    const hardware_interface::HardwareInfo& hardware_info,
    const std::map<std::string, sim::Entity>& joints) {
  std::set<std::string> effort_joint_names;
  for (const auto& joint : hardware_info.joints) {
    for (const auto& command_interface : joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
        effort_joint_names.insert(joint.name);
      }
    }
  }

  if (effort_joint_names.empty()) {
    return;
  }

  pinocchio::urdf::buildModelFromXML(hardware_info.original_xml, model_);
  data_ = pinocchio::Data(model_);
  joint_configuration_ = Eigen::VectorXd::Zero(model_.nq);

  int configuration_index = 0;
  int velocity_index = 0;
  for (std::size_t i = 1; i < model_.names.size(); ++i) {
    const std::string& joint_name = model_.names[i];

    auto entity_it = joints.find(joint_name);
    if (entity_it != joints.end()) {
      position_joints_.push_back({entity_it->second, configuration_index, model_.nqs[i] == 2});

      if (effort_joint_names.count(joint_name) != 0) {
        effort_joints_.push_back({entity_it->second, velocity_index});
      }
    }

    configuration_index += model_.nqs[i];
    velocity_index += model_.nvs[i];
  }
}

CallbackReturn GazeboGravityCompensationSystem::on_init(
    const hardware_interface::HardwareInfo& system_info) {
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return wrapped_system_->on_init(system_info);
}

CallbackReturn GazeboGravityCompensationSystem::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  return wrapped_system_->on_configure(previous_state);
}

CallbackReturn GazeboGravityCompensationSystem::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  return wrapped_system_->on_activate(previous_state);
}

CallbackReturn GazeboGravityCompensationSystem::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  return wrapped_system_->on_deactivate(previous_state);
}

std::vector<hardware_interface::StateInterface>
GazeboGravityCompensationSystem::export_state_interfaces() {
  return wrapped_system_->export_state_interfaces();
}

std::vector<hardware_interface::CommandInterface>
GazeboGravityCompensationSystem::export_command_interfaces() {
  return wrapped_system_->export_command_interfaces();
}

hardware_interface::return_type GazeboGravityCompensationSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  return wrapped_system_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

hardware_interface::return_type GazeboGravityCompensationSystem::read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  return wrapped_system_->read(time, period);
}

hardware_interface::return_type GazeboGravityCompensationSystem::write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  const auto result = wrapped_system_->write(time, period);
  if (result != hardware_interface::return_type::OK || effort_joints_.empty()) {
    return result;
  }

  for (const auto& joint : position_joints_) {
    const auto* position = ecm_->Component<sim::components::JointPosition>(joint.entity);
    const double q = (position != nullptr && !position->Data().empty()) ? position->Data()[0] : 0.0;
    if (joint.is_continuous) {
      joint_configuration_(joint.configuration_index) = std::cos(q);
      joint_configuration_(joint.configuration_index + 1) = std::sin(q);
    } else {
      joint_configuration_(joint.configuration_index) = q;
    }
  }

  const Eigen::VectorXd gravity_torque =
      pinocchio::computeGeneralizedGravity(model_, data_, joint_configuration_);

  for (const auto& joint : effort_joints_) {
    auto* effort_command = ecm_->Component<sim::components::JointForceCmd>(joint.entity);
    if (effort_command != nullptr && !effort_command->Data().empty()) {
      effort_command->Data()[0] += gravity_torque(joint.velocity_index);
    }
  }

  return result;
}

}  // namespace franka_gazebo_bringup

PLUGINLIB_EXPORT_CLASS(franka_gazebo_bringup::GazeboGravityCompensationSystem,
                       gz_ros2_control::GazeboSimSystemInterface)
