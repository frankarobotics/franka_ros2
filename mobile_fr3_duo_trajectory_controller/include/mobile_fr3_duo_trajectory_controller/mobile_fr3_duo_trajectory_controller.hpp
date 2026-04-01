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

/// @file mobile_fr3_duo_trajectory_controller.hpp
/// @brief ros2_control controller that executes joint trajectories for the
///        mobile FR3 duo platform (dual-arm impedance + mobile base velocity).

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "mobile_fr3_duo_trajectory_controller/mobile_fr3_duo_trajectory_controller_parameters.hpp"
#include "mobile_fr3_duo_trajectory_controller/trajectory.hpp"
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

namespace mobile_fr3_duo_trajectory_controller {

/// @brief Trajectory controller for a mobile FR3 duo platform.
///
/// Combines joint impedance control with forwarding mobile base cartesian
/// velocity commands. Accepts FollowJointTrajectory action goals and
/// interpolates between trajectory points.
using Vector7d = Eigen::Matrix<double, 7, 1>;
static constexpr size_t kArms = 2;

class MobileFR3DuoTrajectoryController : public controller_interface::ControllerInterface {
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

 private:
  trajectory_msgs::msg::JointTrajectoryPoint state_current_;
  trajectory_msgs::msg::JointTrajectoryPoint command_next_;

  std::shared_ptr<mobile_fr3_duo_trajectory_controller::ParamListener> param_listener_;
  mobile_fr3_duo_trajectory_controller::Params params_;

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

  /// @brief Callback invoked when a new FollowJointTrajectory goal is received.
  rclcpp_action::GoalResponse goal_received_callback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const FollowJTrajAction::Goal> goal);

  /// @brief Callback invoked when an active goal is requested to be cancelled.
  rclcpp_action::CancelResponse goal_cancelled_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  /// @brief Callback invoked after a goal has been accepted by the action server.
  void goal_accepted_callback(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  /// @brief Store a new trajectory message for the real-time loop to pick up.
  void add_new_trajectory_msg(
      const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& traj_msg);

  /// @brief Return true if the current trajectory has not finished.
  bool has_active_trajectory() const;

  /// @brief Validate waypoint dimensions, ordering, and joint name consistency.
  bool validate_trajectory_msg(const trajectory_msgs::msg::JointTrajectory& trajectory) const;

  /// @brief Cancel and abort any in-progress goal.
  void preempt_active_goal();

  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
      new_trajectory_msg_;

  std::shared_ptr<Trajectory> current_trajectory_ = nullptr;
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> hold_position_msg_ptr_ = nullptr;

  std::vector<std::string> joint_names_;
  std::array<size_t, 7> left_arm_joint_map_;
  std::array<size_t, 7> right_arm_joint_map_;
  std::array<size_t, 3> mobile_base_joint_map_;
  std::map<std::pair<size_t, size_t>, size_t> joint_state_map_;

  // Dual arm parameters
  std::vector<std::string> robot_types_;
  std::vector<std::string> arm_prefixes_;
  std::vector<std::string> robot_prefixes_;
  std::string robot_description_;

  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface>
      franka_cartesian_velocity_;
  std::array<Vector7d, kArms> q_;
  std::array<Vector7d, kArms> dq_;
  std::array<Vector7d, kArms> dq_filtered_;

  Vector7d k_gains_;
  Vector7d d_gains_;

  /// @brief Extract left-arm, right-arm, mobile-base positions and velocities
  ///        from a single JointTrajectoryPoint.
  std::tuple<std::array<double, 7>,
             std::array<double, 7>,
             std::array<double, 3>,
             std::array<double, 3>>
  getCommandsFromJointTrajectoryPoint(
      const trajectory_msgs::msg::JointTrajectoryPoint& point) const;

  /// @brief Extract the 7-DoF positions for each arm from the position vector for all of the
  /// joints.
  std::tuple<std::array<double, 7>, std::array<double, 7>> getArmJointPositionsFromPoint(
      const std::vector<double>& point) const;

  /// @brief Extract mobile base velocities from a JointTrajectoryPoint velocity vector for all of
  /// the joints.
  std::array<double, 3> getMobileBaseVelocitiesFromPoint(const std::vector<double>& point) const;

  /// @brief Extract mobile base positions from a JointTrajectoryPoint position vector for all of
  /// the joints.
  std::array<double, 3> getMobileBasePositionsFromPoint(const std::vector<double>& point) const;

  /// @brief Read current joint states from hardware interfaces into @p state.
  void updateState(trajectory_msgs::msg::JointTrajectoryPoint& state);

  /// @brief Impedance controller for desired joint position via torque commands to the specified
  /// arm.
  void commandArmPosition(const std::array<double, 7>& position, size_t arm_index);

  /// @brief Send cartesian velocity and position commands to the mobile base.
  void commandMobileBaseVelocity(const std::array<double, 3>& mobile_base_velocities,
                                 const std::array<double, 3>& mobile_base_positions);

  /// @brief Fill @p state with zero-valued entries for each joint name.
  void initializeState(trajectory_msgs::msg::JointTrajectoryPoint& state,
                       const std::vector<std::string>& joint_names);
};

}  // namespace mobile_fr3_duo_trajectory_controller
