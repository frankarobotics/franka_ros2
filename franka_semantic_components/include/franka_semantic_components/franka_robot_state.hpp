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

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "urdf/model.h"

#include <realtime_tools/realtime_thread_safe_box.hpp>
#include "franka/robot_state.h"
#include "franka_msgs/msg/franka_robot_state.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace franka_semantic_components {
class FrankaRobotState
    : public semantic_components::SemanticComponentInterface<franka_msgs::msg::FrankaRobotState> {
 public:
  explicit FrankaRobotState(const std::string& name, const std::string& robot_description);

  virtual ~FrankaRobotState() = default;

  /**
   * Assign loaned state interfaces and resolve the robot state box the hardware
   * exports through them.
   *
   * \return true when the interfaces were claimed and the box was resolved.
   */
  auto assign_loaned_state_interfaces(
      std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) -> bool;

  /**
   * Drop the cached box pointer and release loaned interfaces.
   */
  auto release_interfaces() -> void;

  /**
   * @param[in/out] message Initializes this message to contain the respective frame_id information
   */
  virtual auto initialize_robot_state_msg(franka_msgs::msg::FrankaRobotState& message) -> void;

  /**
   * Constructs and return a FrankaRobotState message from the current values.
   * Lazily resolves the state box if it was not resolved during
   * assign_loaned_state_interfaces() (e.g. when interfaces were claimed via the base API).
   * \return true if the message was populated; false if the state box could not be resolved.
   */
  virtual auto get_values_as_message(franka_msgs::msg::FrankaRobotState& message) -> bool;

 protected:
  /**
   * @brief Get the robot state object
   *
   * @return franka::RobotState
   */
  auto get_robot_state() -> franka::RobotState*;

 private:
  using Base = semantic_components::SemanticComponentInterface<franka_msgs::msg::FrankaRobotState>;

  /**
   * Resolves and caches the robot state box from the claimed state interface.
   * \return true if the box was resolved.
   */
  auto initialize_state_buffer() -> bool;

  franka::RobotState robot_state_;
  bool cached_robot_state_valid_{false};
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState>* robot_state_box_{nullptr};

  std::string robot_description_;
  std::string robot_name_;
  const std::string state_interface_name_{"robot_state"};
  std::string full_robot_state_interface_name_;
  bool gripper_loaded_{false};
  size_t kEndEffectorLinkIndex{8};
  // TODO(yazi_ba) update stiffness frame with the user defined transformation
  size_t kStiffnessLinkIndex{8};
  std::shared_ptr<urdf::Model> model_;
  std::vector<std::string> joint_names, link_names;

  /**
   * @brief Populate the link_name std::vector with the links from urdf object in order.
   *       The root link is the first element and tcp is the last element.
   *
   */
  auto set_links_from_urdf() -> void;

  /**
   * @brief Populate the joint_name std::vector with this arm's seven joints, in joint order.
   *
   * The robot_description may describe more than one arm; only the joints belonging to this
   * component's arm are published. Throws std::runtime_error if one of them is missing.
   */
  auto set_joints_from_urdf() -> void;

  /**
   * @brief Set all child links from a link and assign them to the link_name
   *
   * @param link root link
   */
  auto set_child_links(const std::shared_ptr<const urdf::Link>& link) -> void;

  /**
   * @brief Check if gripper is loaded and robot_name + "_hand_tcp" frame exists
   *
   * @return true if gripper is loaded
   * @return false if gripper is not loaded
   */
  auto is_gripper_loaded() -> bool;
  /**
   * @brief Get the robot name from urdf object

   * @return std::string
   */
  auto get_robot_name_from_urdf() -> std::string;

  /**
   * @brief Get the link index from link name
   *
   * @param link_name
   * @return size_t
   */
  auto get_link_index(const std::string& link_name) -> size_t;
};

}  // namespace franka_semantic_components
