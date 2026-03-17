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

#include <franka_example_controllers/tmr/swerve_drive_controller.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Dense>
#include "utils.hpp"

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
SwerveDriveController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
      state_interface_prefix_ + "joint_0/position", state_interface_prefix_ + "joint_1/velocity",
      state_interface_prefix_ + "joint_2/position", state_interface_prefix_ + "joint_3/velocity"};
  return config;
}

controller_interface::return_type SwerveDriveController::update(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period) {
  auto logger = get_node()->get_logger();

  const auto age_of_last_command = (time - last_cmd_vel_->header.stamp).seconds();

  double command_linear_x = 0.0, command_linear_y = 0.0, command_angular_z = 0.0;
  if (age_of_last_command < cmd_vel_timeout_) {
    command_linear_x = last_cmd_vel_->twist.linear.x;
    command_linear_y = last_cmd_vel_->twist.linear.y;
    command_angular_z = last_cmd_vel_->twist.angular.z;
  }

  if (!std::isfinite(command_linear_x) || !std::isfinite(command_linear_y) ||
      !std::isfinite(command_angular_z)) {
    // NaNs occur on initialization when the reference interfaces are not yet set
    return controller_interface::return_type::OK;
  }

  if (odom_open_loop_) {  // update odometry from command interface
    odometry_.update(command_linear_x, command_linear_y, command_angular_z, time);
  } else {  // update odometry from state interface
    const double estimate_steering_position_wheel_1 = state_interfaces_[0].get_optional().value();
    const double estimate_drive_velocity_wheel_1 = state_interfaces_[1].get_optional().value();

    const double estimate_steering_position_wheel_2 = state_interfaces_[2].get_optional().value();
    const double estimate_drive_velocity_wheel_2 = state_interfaces_[3].get_optional().value();

    const std::array<double, 2> steerings{estimate_steering_position_wheel_1,
                                          estimate_steering_position_wheel_2};
    const std::array<double, 2> velocities{estimate_drive_velocity_wheel_1,
                                           estimate_drive_velocity_wheel_2};

    double vx = 0, vy = 0, wz = 0;
    swerve_kinematics_->forward(steerings, velocities, vx, vy, wz);
    odometry_.update(vx, vy, wz, time);
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  bool should_publish = false;
  const rclcpp::Duration publish_period = rclcpp::Duration::from_seconds(1 / publish_rate_);

  try {
    if (previous_publish_timestamp_ + publish_period < time) {
      previous_publish_timestamp_ += publish_period;
      should_publish = true;
    }
  } catch (const std::runtime_error&) {
    // Handle exceptions when the time source changes and initialize publish timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  if (should_publish) {
    if (enable_odom_nav_msg_ && realtime_odom_nav_publisher_) {
      odom_nav_message_.header.stamp = time;
      odom_nav_message_.header.frame_id = odom_frame_id_;
      odom_nav_message_.pose.pose.position.x = odometry_.getX();
      odom_nav_message_.pose.pose.position.y = odometry_.getY();
      odom_nav_message_.pose.pose.orientation.x = orientation.x();
      odom_nav_message_.pose.pose.orientation.y = orientation.y();
      odom_nav_message_.pose.pose.orientation.z = orientation.z();
      odom_nav_message_.pose.pose.orientation.w = orientation.w();
      odom_nav_message_.twist.twist.linear.x = odometry_.getLinearX();
      odom_nav_message_.twist.twist.linear.y = odometry_.getLinearY();
      odom_nav_message_.twist.twist.angular.z = odometry_.getAngular();

      realtime_odom_nav_publisher_->try_publish(odom_nav_message_);
    }

    if (enable_odom_tf_msg_ && realtime_odom_tf_publisher_) {
      auto& transform = odom_tf_message_.transforms.front();
      transform.header.stamp = time;
      transform.header.frame_id = tf_frame_id_;
      transform.child_frame_id = base_link_frame_id_;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odom_tf_publisher_->try_publish(odom_tf_message_);
    }
  }

  double last_linear_x = previous_two_commands_.back().x();
  double second_to_last_linear_x = previous_two_commands_.front().x();

  double last_linear_y = previous_two_commands_.back().y();
  double second_to_last_linear_y = previous_two_commands_.front().y();

  double last_angular = previous_two_commands_.back().z();
  double second_to_last_angular = previous_two_commands_.front().z();

  // rate limiting
  linear_x_limiter_->limit(command_linear_x, last_linear_x, second_to_last_linear_x,
                           period.seconds());
  linear_y_limiter_->limit(command_linear_y, last_linear_y, second_to_last_linear_y,
                           period.seconds());
  angular_z_limiter_->limit(command_angular_z, last_angular, second_to_last_angular,
                            period.seconds());
  previous_two_commands_.pop();
  previous_two_commands_.push({command_linear_x, command_linear_y, command_angular_z});

  if (publish_limited_velocity_ && realtime_cmd_vel_out_publisher_) {
    limited_velocity_message_.header.stamp = time;
    // limited vel is in the same frame as the original one
    limited_velocity_message_.header.frame_id = last_cmd_vel_->header.frame_id;
    limited_velocity_message_.twist.linear.x = command_linear_x;
    limited_velocity_message_.twist.linear.y = command_linear_y;
    limited_velocity_message_.twist.linear.z = 0.0;
    limited_velocity_message_.twist.angular.x = 0.0;
    limited_velocity_message_.twist.angular.y = 0.0;
    limited_velocity_message_.twist.angular.z = command_angular_z;
    realtime_cmd_vel_out_publisher_->try_publish(limited_velocity_message_);
  }

  last_cmd_time_ += period.seconds();

  const Eigen::Vector3d cartesian_linear_velocity(command_linear_x, command_linear_y, 0.0);
  const Eigen::Vector3d cartesian_angular_velocity(0.0, 0.0, command_angular_z);

  if (franka_cartesian_velocity_->setCommand(cartesian_linear_velocity,
                                             cartesian_angular_velocity)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

controller_interface::CallbackReturn SwerveDriveController::on_init() {
  param_listener_ = std::make_shared<swerve_drive_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();

  previous_two_commands_.push({0, 0, 0});
  previous_two_commands_.push({0, 0, 0});

  command_interface_prefix_ = auto_declare<std::string>("command_interface_prefix", "");
  state_interface_prefix_ = auto_declare<std::string>("state_interface_prefix", "");
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          command_interface_prefix_, false);

  tf_frame_id_ = auto_declare("tf_frame_id", "world");
  odom_frame_id_ = auto_declare("odom_frame_id", "base_link");
  odom_open_loop_ = auto_declare("odom_open_loop", true);
  enable_odom_tf_msg_ = auto_declare("enable_odom_tf", true);
  enable_odom_nav_msg_ = auto_declare("enable_odom_nav", true);
  publish_limited_velocity_ = auto_declare("publish_limited_velocity", true);
  publish_rate_ = auto_declare("publish_rate", 50);
  cmd_vel_timeout_ = auto_declare("cmd_vel_timeout", 0.5);

  const std::string argo_drive_front_link_name =
      auto_declare("wheel_1_link_name", "argo_drive_front_link");
  const std::string argo_drive_rear_link_name =
      auto_declare("wheel_2_link_name", "argo_drive_rear_link");
  base_link_frame_id_ = auto_declare("base_link_frame_id", "base_link");

  // get fixed params from robot descriptions
  const std::string robot_description = get_robot_description();
  SE3 front_wheel =
      get_se3_from_description(robot_description, base_link_frame_id_, argo_drive_front_link_name);
  SE3 back_wheel =
      get_se3_from_description(robot_description, base_link_frame_id_, argo_drive_rear_link_name);

  std::array<Eigen::Vector2d, 2> wheel_positions{front_wheel.p.head<2>(), back_wheel.p.head<2>()};
  double wheel_radius =
      get_wheel_radius_from_description(robot_description, argo_drive_front_link_name);

  auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Wheel radius: %f", wheel_radius);
  RCLCPP_INFO(logger, "Wheel 1 x: %f, y: %f", wheel_positions[0].x(), wheel_positions[0].y());
  RCLCPP_INFO(logger, "Wheel 2 x: %f, y: %f", wheel_positions[1].x(), wheel_positions[1].y());

  swerve_kinematics_ = std::make_unique<SwerveKinematics>(wheel_positions, wheel_radius);
  odom_tf_message_.transforms.resize(1);

  odometry_.init(get_node()->now());
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  const auto odom_velocity_rolling_window_size =
      auto_declare<int>("velocity_rolling_window_size", 10);

  auto logger = get_node()->get_logger();

  odometry_.setVelocityRollingWindowSize(odom_velocity_rolling_window_size);

  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  // measurement covariances
  const std::vector<double> odom_pose_covariance_diagonal =
      auto_declare<std::vector<double>>("odom_pose_covariance_diagonal", {0, 0, 0, 0, 0, 0});
  const std::vector<double> odom_twist_covariance_diagonal =
      auto_declare<std::vector<double>>("odom_twist_covariance_diagonal", {0, 0, 0, 0, 0, 0});
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < NUM_DIMENSIONS; ++index) {
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odom_nav_message_.pose.covariance[diagonal_index] = odom_pose_covariance_diagonal[index];
    odom_nav_message_.twist.covariance[diagonal_index] = odom_twist_covariance_diagonal[index];
  }

  // rate limiting
  linear_x_limiter_ = std::make_unique<SpeedLimiter>(
      params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
      params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity,
      params_.linear.x.max_velocity, params_.linear.x.min_acceleration,
      params_.linear.x.max_acceleration, params_.linear.x.min_jerk, params_.linear.x.max_jerk);
  linear_y_limiter_ = std::make_unique<SpeedLimiter>(
      params_.linear.y.has_velocity_limits, params_.linear.y.has_acceleration_limits,
      params_.linear.y.has_jerk_limits, params_.linear.y.min_velocity,
      params_.linear.y.max_velocity, params_.linear.y.min_acceleration,
      params_.linear.y.max_acceleration, params_.linear.y.min_jerk, params_.linear.y.max_jerk);
  angular_z_limiter_ = std::make_unique<SpeedLimiter>(
      params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
      params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
      params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
      params_.angular.z.max_acceleration, params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  // cmd_vel pub/sub
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel", 100, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_cmd_vel_ = msg;
        last_cmd_time_ = 0;
      });
  cmd_vel_out_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel_out", rclcpp::SystemDefaultsQoS());
  realtime_cmd_vel_out_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
          cmd_vel_out_pub_);

  // tf2
  odom_tf_pub_ =
      get_node()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
  realtime_odom_tf_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odom_tf_pub_);

  // nav
  odom_nav_pub_ =
      get_node()->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS());
  realtime_odom_nav_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_nav_pub_);

  odom_tf_message_.transforms.resize(1);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  last_cmd_vel_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::SwerveDriveController,
                       controller_interface::ControllerInterface)