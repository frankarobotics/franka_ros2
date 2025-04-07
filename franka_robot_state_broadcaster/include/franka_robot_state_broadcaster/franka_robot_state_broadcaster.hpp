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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
// TODO: Remove this define when the realtime_publisher is updated in ROS 2
// realtime_publisher has been almost completely re-written in the Rolling release
// Those changes have been partially backported to Humble, but still require this define
#define NON_POLLING 1  // NOLINT
#include <realtime_tools/realtime_publisher.hpp>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster_parameters.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

namespace franka_robot_state_broadcaster {
class FrankaRobotStateBroadcaster : public controller_interface::ControllerInterface {
 public:
  // NOLINTBEGIN
  explicit FrankaRobotStateBroadcaster(
      std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state = nullptr)
      : franka_robot_state_(std::move(franka_robot_state)){};
  // NOLINTEND

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  // override RealtimePublisher to customize the trylock behavior
  class FrankaRobotStateRealtimePublisher
      : public realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaRobotState> {
    using PublisherSharedPtr = rclcpp::Publisher<franka_msgs::msg::FrankaRobotState>::SharedPtr;

   public:
    // Constructor for the nested class
    // NOLINTBEGIN
    explicit FrankaRobotStateRealtimePublisher(PublisherSharedPtr publisher)
        : realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaRobotState>(
              std::move(publisher)) {}
    // NOLINTEND
    // we only need to hide the trylock() method
    bool trylock();
    [[nodiscard]] int try_count() const { return try_count_; }

   private:
    const int try_count_ = 10;
  };
  // shared_ptr to object of override class
  std::shared_ptr<FrankaRobotStateBroadcaster::FrankaRobotStateRealtimePublisher>
      realtime_franka_state_publisher;

  std::shared_ptr<ParamListener> param_listener;
  Params params;

  std::string state_interface_name{"robot_state"};
  std::shared_ptr<rclcpp::Publisher<franka_msgs::msg::FrankaRobotState>> franka_state_publisher;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>
      current_pose_stamped_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>
      last_desired_pose_stamped_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>>
      desired_end_effector_twist_stamped_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>
      external_wrench_in_base_frame_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>
      external_wrench_in_stiffness_frame_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
      external_joint_torques_publisher_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> measured_joint_states_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> desired_joint_states_publisher_;

  const std::string kCurrentPoseTopic = "~/current_pose";
  const std::string kLastDesiredPoseTopic = "~/last_desired_pose";
  const std::string kDesiredEETwist = "~/desired_end_effector_twist";
  const std::string kMeasuredJointStates = "~/measured_joint_states";
  const std::string kExternalWrenchInStiffnessFrame = "~/external_wrench_in_stiffness_frame";
  const std::string kExternalWrenchInBaseFrame = "~/external_wrench_in_base_frame";
  const std::string kExternalJointTorques = "~/external_joint_torques";
  const std::string kDesiredJointStates = "~/desired_joint_states";

  franka_msgs::msg::FrankaRobotState franka_robot_state_msg_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;
};
}  // namespace franka_robot_state_broadcaster
