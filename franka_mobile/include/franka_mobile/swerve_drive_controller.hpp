// Copyright (c) 2025 Franka Robotics GmbH
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

#include <string>
#include <queue>


#include <controller_interface/chainable_controller_interface.hpp>
#include <diff_drive_controller/speed_limiter.hpp>
#include <franka_mobile/swerve_drive_controller_parameters.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#if RCLCPP_VERSION_MAJOR > 16
#include <realtime_tools/realtime_thread_safe_box.hpp>
#else
#include <realtime_tools/realtime_box.hpp>
#include <franka_mobile/swerve_kinematics.hpp>
#endif
#include <tf2_msgs/msg/tf_message.hpp>

#include "odometry.hpp"

namespace franka_mobile {

class SwerveDriveController : public controller_interface::ChainableControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  // when chained
  controller_interface::return_type update_and_write_commands(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

  // when not chained
  controller_interface::return_type update_reference_from_subscribers() override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  bool on_set_chained_mode(bool chained) override;
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

 private:
  // params
  std::string command_interface_prefix_ = "";
  std::string state_interface_prefix_ = "";
  std::string odom_frame_id_ = "";
  std::string tf_frame_id_ = "";
  std::string base_link_frame_id_ = "";
  bool odom_open_loop_ = false;
  bool publish_limited_velocity_ = true;  // on cmd_vel_out
  bool enable_odom_tf_msg_ = true;
  bool enable_odom_nav_msg_ = true;
  double publish_rate_ = 50.0;
  double cmd_vel_timeout_ = 0.5;
  double last_cmd_time_ = 0.0;

  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  std::queue<Eigen::Vector3d> previous_two_commands_;

  // franka interface
  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface>
      franka_cartesian_velocity_;
#if RCLCPP_VERSION_MAJOR > 16
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
#endif

  // pub/sub
  geometry_msgs::msg::TwistStamped limited_velocity_message_;
  tf2_msgs::msg::TFMessage odom_tf_message_;
  nav_msgs::msg::Odometry odom_nav_message_;

  realtime_tools::RealtimeBox<geometry_msgs::msg::TwistStamped::SharedPtr> received_velocity_msg_;
  realtime_tools::RealtimePublisherSharedPtr<nav_msgs::msg::Odometry> realtime_odom_nav_publisher_;
  realtime_tools::RealtimePublisherSharedPtr<tf2_msgs::msg::TFMessage> realtime_odom_tf_publisher_;
  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::TwistStamped>
      realtime_cmd_vel_out_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_out_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_nav_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odom_tf_pub_;

  // pose integration in open loop
  Odometry odometry_;
  // twist differentiation in closed loop
  Eigen::Vector3d p_;
  Eigen::Quaterniond q_;

  // rate limiting
  std::unique_ptr<diff_drive_controller::SpeedLimiter> linear_x_limiter_;
  std::unique_ptr<diff_drive_controller::SpeedLimiter> linear_y_limiter_;
  std::unique_ptr<diff_drive_controller::SpeedLimiter> angular_z_limiter_;
  
  // for humble
  std::unique_ptr<SwerveKinematics> swerve_kinematics_ = nullptr;

  std::shared_ptr<swerve_drive_controller::ParamListener> param_listener_;
  swerve_drive_controller::Params params_;
};

}  // namespace franka_mobile