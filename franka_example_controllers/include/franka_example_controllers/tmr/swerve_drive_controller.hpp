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

#include <controller_interface/controller_interface.hpp>
#include <diff_drive_controller/speed_limiter.hpp>
#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "odometry.hpp"
#include "swerve_kinematics.hpp"

#include <franka_example_controllers/swerve_drive_controller_parameters.hpp>

namespace franka_example_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fsc = franka_semantic_components;
/**
 * The cartesian velocity example controller
 */
class SwerveDriveController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

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
  double wheel_radius_ = 0.05;

  std::array<double, 6> odom_pose_covariance_diagonal_;
  std::array<double, 6> odom_twist_covariance_diagonal_;

  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  std::queue<std::array<double, 3>> previous_two_commands_;

  // franka interface
  std::unique_ptr<fsc::FrankaCartesianVelocityInterface> franka_cartesian_velocity_;

  // pub/sub
  geometry_msgs::msg::TwistStamped::SharedPtr last_cmd_vel_;
  geometry_msgs::msg::TwistStamped limited_velocity_message_;
  tf2_msgs::msg::TFMessage odom_tf_message_;
  nav_msgs::msg::Odometry odom_nav_message_;

  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::TwistStamped> received_velocity_msg_;
  realtime_tools::RealtimePublisherSharedPtr<nav_msgs::msg::Odometry> realtime_odom_nav_publisher_;
  realtime_tools::RealtimePublisherSharedPtr<tf2_msgs::msg::TFMessage> realtime_odom_tf_publisher_;
  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::TwistStamped>
      realtime_cmd_vel_out_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_out_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_nav_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odom_tf_pub_;

  // twist integration
  Odometry odometry_;
  std::unique_ptr<SwerveKinematics> swerve_kinematics_;

  // rate limiting
  using SpeedLimiter = ::diff_drive_controller::SpeedLimiter;
  std::unique_ptr<SpeedLimiter> linear_x_limiter_;
  std::unique_ptr<SpeedLimiter> linear_y_limiter_;
  std::unique_ptr<SpeedLimiter> angular_z_limiter_;

  std::shared_ptr<swerve_drive_controller::ParamListener> param_listener_;
  swerve_drive_controller::Params params_;
};

}  // namespace franka_example_controllers