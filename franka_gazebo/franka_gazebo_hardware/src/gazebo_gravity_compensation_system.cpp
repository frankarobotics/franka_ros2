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

#include <franka_gazebo_hardware/gazebo_gravity_compensation_system.hpp>

#include <cstddef>
#include <exception>
#include <set>
#include <string>
#include <vector>

#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace franka_gazebo_hardware {

namespace {
// Tight coupling to gz_ros2_control internals: these strings name the upstream
// pluginlib base interface and the stock system we decorate. If upstream renames
// or refactors GazeboSimSystem or its base interface, pluginlib can no longer
// resolve them and createSharedInstance throws at load time. The try/catch in
// initSim() now surfaces that as a logged error instead of crashing sim bring-up,
// but the names must still be kept in sync with gz_ros2_control by hand.
constexpr char kSystemPackage[] = "gz_ros2_control";
constexpr char kSystemBaseClass[] = "gz_ros2_control::GazeboSimSystemInterface";
constexpr char kWrappedSystemClass[] = "gz_ros2_control/GazeboSimSystem";
}  // namespace

auto GazeboGravityCompensationSystem::initSim(rclcpp::Node::SharedPtr& model_nh,
                                              std::map<std::string, sim::Entity>& joints,
                                              const hardware_interface::HardwareInfo& hardware_info,
                                              sim::EntityComponentManager& ecm,
                                              int& update_rate) -> bool {
  ecm_ = &ecm;
  model_node_ = model_nh;

  try {
    system_loader_ =
        std::make_shared<pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>>(
            kSystemPackage, kSystemBaseClass);
    wrapped_system_ = system_loader_->createSharedInstance(kWrappedSystemClass);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(model_nh->get_logger(), "Failed to load wrapped gz_ros2_control system '%s': %s",
                 kWrappedSystemClass, exception.what());
    return false;
  }

  if (!wrapped_system_->initSim(model_nh, joints, hardware_info, ecm, update_rate)) {
    return false;
  }

  try {
    initGravityModel(hardware_info, joints);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(model_nh->get_logger(), "Failed to build the gravity compensation model: %s",
                 exception.what());
    return false;
  }
  return true;
}

auto GazeboGravityCompensationSystem::initGravityModel(
    const hardware_interface::HardwareInfo& hardware_info,
    const std::map<std::string, sim::Entity>& joints) -> void {
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

  std::set<std::string> simulated_joint_names;
  for (const auto& [joint_name, entity] : joints) {
    simulated_joint_names.insert(joint_name);
  }

  gravity_model_.build(hardware_info.original_xml, simulated_joint_names, effort_joint_names);

  for (const auto& configuration_joint : gravity_model_.configurationJoints()) {
    configuration_entities_.push_back(joints.at(configuration_joint.joint_name));
  }
  for (const auto& effort_joint : gravity_model_.effortJoints()) {
    effort_entities_.push_back(joints.at(effort_joint.joint_name));
  }

  // Pre-allocate the per-cycle buffers once, so write() never touches the heap.
  joint_positions_.assign(configuration_entities_.size(), 0.0);
  gravity_torques_.assign(effort_entities_.size(), 0.0);
}

auto GazeboGravityCompensationSystem::on_init(const hardware_interface::HardwareInfo& system_info)
    -> CallbackReturn {
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Both initializations are intentional: the base SystemInterface::on_init stores
  // the HardwareInfo this decorator needs, while wrapped_system_->on_init runs the
  // stock gz_ros2_control setup that actually drives the simulated joints.
  return wrapped_system_->on_init(system_info);
}

auto GazeboGravityCompensationSystem::on_configure(const rclcpp_lifecycle::State& previous_state)
    -> CallbackReturn {
  return wrapped_system_->on_configure(previous_state);
}

auto GazeboGravityCompensationSystem::on_activate(const rclcpp_lifecycle::State& previous_state)
    -> CallbackReturn {
  return wrapped_system_->on_activate(previous_state);
}

auto GazeboGravityCompensationSystem::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    -> CallbackReturn {
  return wrapped_system_->on_deactivate(previous_state);
}

auto GazeboGravityCompensationSystem::export_state_interfaces()
    -> std::vector<hardware_interface::StateInterface> {
  return wrapped_system_->export_state_interfaces();
}

auto GazeboGravityCompensationSystem::export_command_interfaces()
    -> std::vector<hardware_interface::CommandInterface> {
  return wrapped_system_->export_command_interfaces();
}

auto GazeboGravityCompensationSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) -> hardware_interface::return_type {
  return wrapped_system_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

auto GazeboGravityCompensationSystem::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    -> hardware_interface::return_type {
  return wrapped_system_->read(time, period);
}

auto GazeboGravityCompensationSystem::write(const rclcpp::Time& time,
                                            const rclcpp::Duration& period)
    -> hardware_interface::return_type {
  const auto result = wrapped_system_->write(time, period);
  if (result != hardware_interface::return_type::OK || effort_entities_.empty()) {
    return result;
  }

  for (std::size_t i = 0; i < configuration_entities_.size(); ++i) {
    const auto* position =
        ecm_->Component<sim::components::JointPosition>(configuration_entities_[i]);
    joint_positions_[i] =
        (position != nullptr && !position->Data().empty()) ? position->Data()[0] : 0.0;
  }

  // The gravity computation indexes fixed-size buffers and calls into pinocchio.
  // A single bad cycle must degrade to "no compensation added this step" rather
  // than throwing into the Gazebo system-update loop and tearing down the server.
  try {
    gravity_model_.computeGravityTorque(joint_positions_, gravity_torques_);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR_THROTTLE(model_node_->get_logger(), *model_node_->get_clock(), 1000,
                          "Gravity compensation skipped this cycle: %s", exception.what());
    return result;
  }

  for (std::size_t i = 0; i < effort_entities_.size(); ++i) {
    auto* effort_command = ecm_->Component<sim::components::JointForceCmd>(effort_entities_[i]);
    // JointForceCmd exists only once an effort controller has claimed the interface; a
    // nullptr/empty here means no claimant this cycle. In that case gz_ros2_control's own
    // velocity-zero hold keeps the joint up, so dropping the gravity torque is safe.
    if (effort_command != nullptr && !effort_command->Data().empty()) {
      effort_command->Data()[0] += gravity_torques_[i];
    }
  }

  return result;
}

}  // namespace franka_gazebo_hardware

PLUGINLIB_EXPORT_CLASS(franka_gazebo_hardware::GazeboGravityCompensationSystem,
                       gz_ros2_control::GazeboSimSystemInterface)
