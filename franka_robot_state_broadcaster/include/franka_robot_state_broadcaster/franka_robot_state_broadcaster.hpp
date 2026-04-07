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

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "franka_robot_state_broadcaster/async_buffer.hpp"
#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster_parameters.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

namespace franka_robot_state_broadcaster {
class FrankaRobotStateBroadcaster : public controller_interface::ControllerInterface {
 public:
  explicit FrankaRobotStateBroadcaster(
      std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state = nullptr);

  ~FrankaRobotStateBroadcaster() override;

  // Controller update rate in Hz (must match controller_manager update_rate).
  static constexpr int kUpdateRate = 1000;

  // SCHED_FIFO priority for the publish thread.
  // Must be below the ros2_control RT control loop (typically 70-80) so the RT thread
  // is never preempted by publishing, and above 0 so the OS schedules it promptly.
  static constexpr int kPublishThreadPriority = 50;

  // How long the publish thread sleeps when no new data is ready.
  // At 1kHz the update() loop fires every 1ms; sleeping 200µs means the thread
  // wakes at most 200µs after data_ready_ is set, keeping latency well below 1ms.
  static constexpr int kPublishThreadSleepUs = 200;

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

  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  AsyncBuffer<franka_msgs::msg::FrankaRobotState> state_buffer_;

  std::thread publish_thread_;
  std::atomic<bool> is_publish_thread_running_{false};

  std::atomic<bool> data_ready_{false};

  // Publish rate in Hz for convenience topics, loaded from the convenience_publish_rate parameter.
  // Written in on_configure() before the publish thread starts; read in publishRunner().
  int convenience_publish_rate_{1000};

  void startPublishThread();
  void stopPublishThread();
  void publishRunner();
};
}  // namespace franka_robot_state_broadcaster
