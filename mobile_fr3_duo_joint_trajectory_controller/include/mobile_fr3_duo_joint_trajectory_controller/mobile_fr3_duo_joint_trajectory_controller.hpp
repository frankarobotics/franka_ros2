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

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "mobile_fr3_duo_joint_trajectory_controller/mobile_fr3_duo_joint_trajectory_controller_parameters.hpp"
#include "mobile_fr3_duo_joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using namespace std::chrono_literals;  // NOLINT

namespace mobile_fr3_duo_joint_trajectory_controller {

/**
 * The mobile FR3 duo joint trajectory controller combines:
 * - Dual arm joint impedance control (like fr3_duo)
 * - Mobile base cartesian velocity control
 */
using Vector7d = Eigen::Matrix<double, 7, 1>;
static constexpr size_t kArms = 2;

class MobileFR3DuoJointTrajectoryController : public controller_interface::ControllerInterface {
 public:
  CallbackReturn on_init() override;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 protected:
  trajectory_msgs::msg::JointTrajectoryPoint state_current_;
  trajectory_msgs::msg::JointTrajectoryPoint command_next_;
  //   trajectory_msgs::msg::JointTrajectoryPoint state_desired_;
  //   trajectory_msgs::msg::JointTrajectoryPoint state_error_;

  std::shared_ptr<mobile_fr3_duo_joint_trajectory_controller::ParamListener> param_listener_;
  mobile_fr3_duo_joint_trajectory_controller::Params params_;

  rclcpp::Time traj_time_;

  interpolation_methods::InterpolationMethod interpolation_method_{
      interpolation_methods::DEFAULT_INTERPOLATION};

  rclcpp::Duration update_period_{0, 0};

  using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  RealtimeGoalHandleBuffer rt_active_goal_;
  rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;
  std::atomic<bool> rt_has_pending_goal_{false};
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms);

  // callbacks for action_server_
  rclcpp_action::GoalResponse goal_received_callback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const FollowJTrajAction::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);
  void goal_accepted_callback(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  void add_new_trajectory_msg(
      const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& traj_msg);

  bool has_active_trajectory() const;

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> set_hold_position();

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> set_success_trajectory_point();

  std::atomic<bool> rt_is_holding_{false};

  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
      new_trajectory_msg_;

  std::shared_ptr<Trajectory> current_trajectory_ = nullptr;
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> hold_position_msg_ptr_ = nullptr;

 private:
  std::vector<std::string> joint_names_;
  // Dual arm parameters
  std::vector<std::string> robot_types_;
  std::vector<std::string> arm_prefixes_;
  std::vector<std::string> robot_prefixes_;
  std::string robot_description_;

  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface>
      franka_cartesian_velocity_;
  std::array<Vector7d, kArms> q_;
  std::array<Vector7d, kArms> initial_q_;
  std::array<Vector7d, kArms> dq_;
  std::array<Vector7d, kArms> dq_filtered_;

  Vector7d k_gains_;
  Vector7d d_gains_;
  double elapsed_time_{0.0};

  void updateState(trajectory_msgs::msg::JointTrajectoryPoint& state);
  void commandArmPosition(const Vector7d& q_goal, size_t arm_index);
  void commandMobileBaseVelocity(const std::array<double, 3>& planar_base_velocities);
  size_t get_first_joint_index(const std::vector<std::string>& joint_names,
                               const std::string& prefix) const;
  Vector7d get_slice_of_trajectory_positions_arm(
      const trajectory_msgs::msg::JointTrajectoryPoint& command,
      size_t start_index) const;
  std::array<double, 3> get_slice_of_trajectory_velocities_base(
      const trajectory_msgs::msg::JointTrajectoryPoint& command,
      size_t start_index) const;

  bool has_printed_ = false;
};

}  // namespace mobile_fr3_duo_joint_trajectory_controller
