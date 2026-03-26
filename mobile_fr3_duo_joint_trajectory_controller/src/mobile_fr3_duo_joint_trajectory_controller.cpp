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

#include "mobile_fr3_duo_joint_trajectory_controller/mobile_fr3_duo_joint_trajectory_controller.hpp"
#include "mobile_fr3_duo_joint_trajectory_controller/mapping.hpp"

#include <cmath>
#include <string>

#include "rclcpp_action/create_server.hpp"

namespace mobile_fr3_duo_joint_trajectory_controller {

static constexpr size_t kBaseJoints = 3;
static constexpr size_t kArmStateInterfaces = 7 * kArms;  // arm joints: pos + vel
static constexpr size_t kBaseCommandInterfaces = 6;       // vx vy vz wx wy wz
static constexpr size_t kBaseStateInterfaces = 0;         // base joints: pos + vel
static constexpr size_t kArmCommandInterfaces = 7;

CallbackReturn MobileFR3DuoJointTrajectoryController::on_init() {
  try {
    // Create the parameter listener and get the parameters
    param_listener_ =
        std::make_shared<mobile_fr3_duo_joint_trajectory_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  try {
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<std::string>>("robot_prefixes", {});
    auto_declare<std::vector<std::string>>("robot_types", {});
    auto_declare<std::string>("cartesian_velocity_interface_prefix", "");
  } catch (...) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MobileFR3DuoJointTrajectoryController::command_interface_configuration() const {
  auto logger = get_node()->get_logger();

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names = franka_cartesian_velocity_->get_command_interface_names();

  for (size_t index = 0; index < arm_prefixes_.size(); ++index) {
    for (size_t i = 1; i <= kArmCommandInterfaces; ++i) {
      config.names.push_back(arm_prefixes_[index] + "_" + robot_types_[index + 1] + "_joint" +
                             std::to_string(i) + "/effort");
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
MobileFR3DuoJointTrajectoryController::state_interface_configuration() const {
  auto logger = get_node()->get_logger();
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // TODO (wink_ma ): don't use kArmCommandInterfaces here
  for (size_t arm_index = 0; arm_index < kArms; ++arm_index) {
    for (size_t i = 1; i <= kArmCommandInterfaces; ++i) {
      std::string prefix = arm_prefixes_[arm_index] + "_" + robot_types_[arm_index + 1] + "_joint" +
                           std::to_string(i);
      config.names.push_back(prefix + "/position");
      config.names.push_back(prefix + "/velocity");
    }
  }
  return config;
}

controller_interface::return_type MobileFR3DuoJointTrajectoryController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  auto logger = this->get_node()->get_logger();
  auto clock = *this->get_node()->get_clock();

  const auto active_goal = *rt_active_goal_.readFromRT();

  const auto current_trajectory_msg = current_trajectory_->get_trajectory_msg();
  auto new_external_msg = new_trajectory_msg_.readFromRT();

  if (current_trajectory_msg != *new_external_msg &&
      (rt_has_pending_goal_ && !active_goal) == false) {
    sort_to_local_joint_order(*new_external_msg, params_.joints);

    current_trajectory_->update(*new_external_msg);
  }

  updateState(state_current_);

  if (!has_active_trajectory()) {
    return controller_interface::return_type::OK;
  }

  if (!current_trajectory_->is_sampled_already()) {
    current_trajectory_->set_point_before_trajectory_msg(time, state_current_);
    traj_time_ = time;
  } else {
    traj_time_ += period;
  }

  TrajectoryPointConstIter start_segment_itr, end_segment_itr;

  const bool is_valid_point =
      current_trajectory_->sample(traj_time_ + update_period_, interpolation_method_, command_next_,
                                  start_segment_itr, end_segment_itr, false);
  if (!is_valid_point) {
    RCLCPP_ERROR(logger, "Sampling current trajectory returned invalid point!");
    return controller_interface::return_type::ERROR;
  }

  state_current_.time_from_start = time - current_trajectory_->time_from_start();

  const rclcpp::Time traj_start = current_trajectory_->time_from_start();
  const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;

  auto [left_arm_joint_position, right_arm_joint_position, mobile_base_positions,
        mobile_base_velocities] = getCommandsFromJointTrajectoryPoint(command_next_);
  commandArmPosition(left_arm_joint_position, 0);
  commandArmPosition(right_arm_joint_position, 1);
  // TODO (wink_ma): get theta from robot state rather than planned trajectory
  commandMobileBaseVelocity(mobile_base_velocities, mobile_base_positions);

  const bool before_last_point = end_segment_itr != current_trajectory_->end();
  const bool goal_reached_successfully = active_goal && !before_last_point;

  if (goal_reached_successfully) {
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
    result->set__error_string("Goal successfully reached!");
    active_goal->setSucceeded(result);
    // TODO(matthew-reynolds): Need a lock-free write here
    // See https://github.com/ros-controls/ros2_controllers/issues/168
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    rt_has_pending_goal_ = false;

    RCLCPP_INFO(logger, "Goal reached, success!");
  }

  return controller_interface::return_type::OK;
}

CallbackReturn MobileFR3DuoJointTrajectoryController::on_configure(const rclcpp_lifecycle::State&) {
  auto k = get_node()->get_parameter("k_gains").as_double_array();
  auto d = get_node()->get_parameter("d_gains").as_double_array();
  // TODO (wink_ma): throw errror if prefixes and types do not makes sense (e.g. not matching
  // command / state interface number)
  robot_prefixes_ = get_node()->get_parameter("robot_prefixes").as_string_array();
  robot_types_ = get_node()->get_parameter("robot_types").as_string_array();
  const std::string cartesian_velocity_interface_prefix =
      get_node()->get_parameter("cartesian_velocity_interface_prefix").as_string();
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          cartesian_velocity_interface_prefix, false);

  auto arm_prefixes_begin = robot_prefixes_.begin() + 1;
  arm_prefixes_ = std::vector<std::string>(arm_prefixes_begin, arm_prefixes_begin + 2);

  if (k.size() != kArmCommandInterfaces || d.size() != kArmCommandInterfaces) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains and d_gains must be size %zu",
                 kArmCommandInterfaces);
    return CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i < kArmCommandInterfaces; ++i) {
    k_gains_(i) = k[i];
    d_gains_(i) = d[i];
  }

  initializeState(state_current_, params_.joints);

  left_arm_joint_map_ = get_arm_joint_map(params_.joints, "left");
  right_arm_joint_map_ = get_arm_joint_map(params_.joints, "right");
  mobile_base_joint_map_ =
      get_mobile_base_joint_map(params_.joints, {"planar_x", "planar_y", "planar_theta"});
  joint_state_map_ = get_joint_state_map(params_.joints, arm_prefixes_);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
      get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
      get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
      std::string(get_node()->get_name()) + "/follow_joint_trajectory",
      std::bind(&MobileFR3DuoJointTrajectoryController::goal_received_callback, this, _1, _2),
      std::bind(&MobileFR3DuoJointTrajectoryController::goal_cancelled_callback, this, _1),
      std::bind(&MobileFR3DuoJointTrajectoryController::goal_accepted_callback, this, _1));

  update_period_ =
      rclcpp::Duration(0.0, static_cast<uint32_t>(1.0e9 / static_cast<double>(get_update_rate())));

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFR3DuoJointTrajectoryController::on_activate(const rclcpp_lifecycle::State&) {
  auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Trying to activate MobileFR3DuoJointTrajectoryController.");

  current_trajectory_ = std::make_shared<Trajectory>();
  new_trajectory_msg_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);

  updateState(state_current_);

  RCLCPP_INFO(logger, "Successfully activated MobileFR3DuoJointTrajectoryController.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFR3DuoJointTrajectoryController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  current_trajectory_.reset();

  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse MobileFR3DuoJointTrajectoryController::goal_received_callback(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const FollowJTrajAction::Goal> goal) {
  if (get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!validate_trajectory_msg(goal->trajectory)) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MobileFR3DuoJointTrajectoryController::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle) {
  const auto active_goal = *rt_active_goal_.readFromNonRT();

  const bool cancel_request_refers_to_active_goal = active_goal && active_goal->gh_ == goal_handle;
  if (cancel_request_refers_to_active_goal) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Canceling active action goal because cancel callback received.");

    rt_has_pending_goal_ = false;
    active_goal->setCanceled(std::make_shared<FollowJTrajAction::Result>());
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  RCLCPP_INFO(get_node()->get_logger(), "Accepting request to cancel goal");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MobileFR3DuoJointTrajectoryController::goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle) {
  auto logger = this->get_node()->get_logger();

  rt_has_pending_goal_ = true;

  preempt_active_goal();
  auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

  add_new_trajectory_msg(traj_msg);

  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = params_.joints;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ =
      get_node()->create_wall_timer(action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

bool MobileFR3DuoJointTrajectoryController::validate_trajectory_msg(
    const trajectory_msgs::msg::JointTrajectory& trajectory) const {
  // TODO (wink_ma): check for missing joints, empty trajectory points, non-zero end velocity
  return true;
}

bool MobileFR3DuoJointTrajectoryController::has_active_trajectory() const {
  return current_trajectory_ != nullptr && current_trajectory_->has_trajectory_msg();
}

void MobileFR3DuoJointTrajectoryController::add_new_trajectory_msg(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& traj_msg) {
  new_trajectory_msg_.writeFromNonRT(traj_msg);
}
void MobileFR3DuoJointTrajectoryController::preempt_active_goal() {
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal) {
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    result->set__error_string("Current goal cancelled due to new incoming action.");
    active_goal->setCanceled(result);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

void MobileFR3DuoJointTrajectoryController::initializeState(
    trajectory_msgs::msg::JointTrajectoryPoint& state,
    const std::vector<std::string>& joint_names) {
  size_t number_of_joints = joint_names.size();

  q_.fill(Vector7d::Zero());
  dq_.fill(Vector7d::Zero());
  dq_filtered_.fill(Vector7d::Zero());
  state.positions.resize(number_of_joints, 0.0);
  state.velocities.resize(number_of_joints, 0.0);

  for (size_t i = 0; i < number_of_joints; ++i) {
    state.positions[i] = 0.0;
    state.velocities[i] = 0.0;
  }
}

void MobileFR3DuoJointTrajectoryController::updateState(
    trajectory_msgs::msg::JointTrajectoryPoint& state) {
  auto logger = get_node()->get_logger();
  auto clock = *get_node()->get_clock();

  state.time_from_start.sec = 0;
  state.time_from_start.nanosec = 0;

  for (size_t arm_index = 0; arm_index < kArms; ++arm_index) {
    for (size_t i = 0; i < kArmCommandInterfaces; ++i) {
      size_t position_index = kBaseStateInterfaces + arm_index * kArmStateInterfaces + i * kArms;
      size_t velocity_index = position_index + 1;

      auto position = state_interfaces_[position_index].get_optional();
      auto velocity = state_interfaces_[velocity_index].get_optional();

      if (position && velocity) {
        q_[arm_index](i) = *position;
        dq_[arm_index](i) = *velocity;
        dq_filtered_[arm_index](i) = *velocity;
      } else {
        RCLCPP_WARN_THROTTLE(logger, clock, 1000, "Missing state for arm_index %zu joint %zu",
                             arm_index, i);
      }
    }
  }

  for (size_t joint_index = 0; joint_index < kArmCommandInterfaces; ++joint_index) {
    for (size_t arm_index = 0; arm_index < kArms; ++arm_index) {
      state.positions[joint_state_map_.at({arm_index, joint_index})] = q_[arm_index](joint_index);
      state.velocities[joint_state_map_.at({arm_index, joint_index})] = dq_[arm_index](joint_index);
    }
  }

  // TODO (wink_ma): use state_interface  for cartesian pose and cartesian velocity of mobile base
  // to update position and velocity of (x,y,theta)
  // Now, both are left at the default of 0.0
}

void MobileFR3DuoJointTrajectoryController::commandArmPosition(
    const std::array<double, 7>& position,
    size_t arm_index) {
  const Vector7d q_goal = Eigen::Map<const Vector7d>(position.data());

  constexpr double kAlpha = 0.99;
  dq_filtered_[arm_index] = (1.0 - kAlpha) * dq_filtered_[arm_index] + kAlpha * dq_[arm_index];

  Vector7d tau = k_gains_.cwiseProduct(q_goal - q_[arm_index]) +
                 d_gains_.cwiseProduct(-dq_filtered_[arm_index]);

  size_t cmd_offset = kBaseCommandInterfaces + arm_index * kArmCommandInterfaces;
  for (size_t j = 0; j < kArmCommandInterfaces; ++j) {
    if (!command_interfaces_[cmd_offset + j].set_value(tau(j))) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Failed to set torque for arm_index %zu joint %zu", arm_index, j);
    }
  }
}

std::tuple<std::array<double, 7>,
           std::array<double, 7>,
           std::array<double, 3>,
           std::array<double, 3>>
MobileFR3DuoJointTrajectoryController::getCommandsFromJointTrajectoryPoint(
    const trajectory_msgs::msg::JointTrajectoryPoint& point) const {
  auto [left_arm_joint_positions, right_arm_joint_positions] =
      getArmJointPositionsFromPoint(point.positions);
  std::array<double, 3> mobile_base_positions = getMobileBasePositionsFromPoint(point.positions);
  std::array<double, 3> mobile_base_velocities = getMobileBaseVelocitiesFromPoint(point.velocities);

  return std::make_tuple(left_arm_joint_positions, right_arm_joint_positions, mobile_base_positions,
                         mobile_base_velocities);
}

std::tuple<std::array<double, 7>, std::array<double, 7>>
MobileFR3DuoJointTrajectoryController::getArmJointPositionsFromPoint(
    const std::vector<double>& point) const {
  std::array<double, 7> left_arm_joint_positions;
  std::array<double, 7> right_arm_joint_positions;

  for (size_t i = 0; i < 7; ++i) {
    left_arm_joint_positions[i] = point[left_arm_joint_map_[i]];
  }
  for (size_t i = 0; i < 7; ++i) {
    right_arm_joint_positions[i] = point[right_arm_joint_map_[i]];
  }
  return std::make_tuple(left_arm_joint_positions, right_arm_joint_positions);
}

std::array<double, 3> MobileFR3DuoJointTrajectoryController::getMobileBasePositionsFromPoint(
    const std::vector<double>& point) const {
  std::array<double, 3> mobile_base_positions;

  for (size_t i = 0; i < mobile_base_positions.size(); ++i) {
    mobile_base_positions[i] = point[mobile_base_joint_map_[i]];
  }
  return mobile_base_positions;
}

std::array<double, 3> MobileFR3DuoJointTrajectoryController::getMobileBaseVelocitiesFromPoint(
    const std::vector<double>& point) const {
  std::array<double, 3> mobile_base_velocities;

  for (size_t i = 0; i < mobile_base_velocities.size(); ++i) {
    mobile_base_velocities[i] = point[mobile_base_joint_map_[i]];
  }
  return mobile_base_velocities;
}

void MobileFR3DuoJointTrajectoryController::commandMobileBaseVelocity(
    const std::array<double, 3>& mobile_base_velocities,
    const std::array<double, 3>& mobile_base_positions) {
  double vx_world = mobile_base_velocities[0];
  double vy_world = mobile_base_velocities[1];

  // TODO (wink_ma): make eigen matrix multiplication for this
  double theta = mobile_base_positions[2];
  double c = std::cos(theta);
  double s = std::sin(theta);
  double vx_local = c * vx_world + s * vy_world;
  double vy_local = -s * vx_world + c * vy_world;

  const Eigen::Vector3d linear{vx_local, vy_local, 0.0};
  const Eigen::Vector3d angular{0.0, 0.0, mobile_base_velocities[2]};

  if (!franka_cartesian_velocity_->setCommand(linear, angular)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Failed to set tmr velocity");
  }
}

}  // namespace mobile_fr3_duo_joint_trajectory_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mobile_fr3_duo_joint_trajectory_controller::MobileFR3DuoJointTrajectoryController,
    controller_interface::ControllerInterface)
