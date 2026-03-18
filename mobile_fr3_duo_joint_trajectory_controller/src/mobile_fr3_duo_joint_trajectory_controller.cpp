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

#include <mobile_fr3_duo_joint_trajectory_controller/mobile_fr3_duo_joint_trajectory_controller.hpp>

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

  // config.names = {"vx/cartesian_velocity", "vy/cartesian_velocity", "vz/cartesian_velocity",
  //                 "wx/cartesian_velocity", "wy/cartesian_velocity", "wz/cartesian_velocity"};

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
    // fill_partial_goal(*new_external_msg);
    // sort_to_local_joint_order(*new_external_msg);

    current_trajectory_->update(*new_external_msg);
    joint_names_ = current_trajectory_->get_trajectory_msg()->joint_names;

    RCLCPP_DEBUG(logger, "Updated current_trajectory_msg with new_external_msg.");
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      RCLCPP_DEBUG(logger, "joint_names_[%zu] = %s", i, joint_names_[i].c_str());
    }
  }

  updateState(state_current_);
  state_current_.time_from_start.sec = 0;
  state_current_.time_from_start.nanosec = 0;
  // read_state_from_state_interfaces(state_current_);

  if (has_active_trajectory()) {
    bool first_sample = false;
    if (!current_trajectory_->is_sampled_already()) {
      first_sample = true;
      // TODO (wink_ma): remove this joints_angle_wraparound
      std::vector<bool> joints_angle_wraparound(17, false);
      current_trajectory_->set_point_before_trajectory_msg(time, state_current_,
                                                           joints_angle_wraparound);
      traj_time_ = time;
    } else {
      traj_time_ += period;
    }

    TrajectoryPointConstIter start_segment_itr, end_segment_itr;

    const bool valid_point =
        current_trajectory_->sample(traj_time_ + update_period_, interpolation_method_,
                                    command_next_, start_segment_itr, end_segment_itr, false);

    state_current_.time_from_start = time - current_trajectory_->time_from_start();

    if (valid_point) {
      const rclcpp::Time traj_start = current_trajectory_->time_from_start();
      // this is the time instance
      // - started with the first segment: when the first point will be reached (in the future)
      // - later: when the point of the current segment was reached
      const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;
      // time_difference is
      // - negative until first point is reached
      // - counting from zero to time_from_start of next point
      double time_difference = traj_time_.seconds() - segment_time_from_start.seconds();
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      const bool before_last_point = end_segment_itr != current_trajectory_->end();

      // TODO (wink_ma): check these in separate functions
      // bool should_abort_due_to_timeout = !before_last_point && !rt_is_holding_ &&
      //                                    cmd_timeout_ > 0.0 && time_difference > cmd_timeout_;
      tolerance_violated_while_moving = false;
      within_goal_time = true;

      bool should_pass_commands_to_command_interfaces =
          !tolerance_violated_while_moving && within_goal_time;

      // This could be much simpler if our own command_interfaces_ already had the same
      // structure as the next_command
      if (should_pass_commands_to_command_interfaces) {
        std::array<std::string, kArms> arm_prefixes = {"left", "right"};
        std::array<size_t, kArms> first_joint_indices = {0, 0};

        for (size_t arm_index = 0; arm_index < kArms; ++arm_index) {
          first_joint_indices[arm_index] =
              get_first_joint_index(joint_names_, arm_prefixes[arm_index]);
          Vector7d q =
              get_slice_of_trajectory_positions_arm(command_next_, first_joint_indices[arm_index]);
          commandArmPosition(q, arm_index);
        }
        size_t first_joint_index = get_first_joint_index(joint_names_, "planar");
        std::array<double, 3> planar_base_velocities =
            get_slice_of_trajectory_velocities_base(command_next_, first_joint_index);

        commandMobileBaseVelocity(planar_base_velocities);
        RCLCPP_INFO_THROTTLE(logger, clock, 1000, "Commanded planar_base_velocities = [%f,%f,%f]",
                             planar_base_velocities[0], planar_base_velocities[1],
                             planar_base_velocities[2]);
      }
      if (active_goal) {
        // check goal tolerance
        if (!before_last_point) {
          if (!outside_goal_tolerance) {
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            result->set__error_string("Goal successfully reached!");
            active_goal->setSucceeded(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_ = false;

            RCLCPP_INFO(logger, "Goal reached, success!");

            // new_trajectory_msg_.reset();
            // new_trajectory_msg_.initRT(set_success_trajectory_point());
          }
        }
      }
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn MobileFR3DuoJointTrajectoryController::on_configure(const rclcpp_lifecycle::State&) {
  auto k = get_node()->get_parameter("k_gains").as_double_array();
  auto d = get_node()->get_parameter("d_gains").as_double_array();
  robot_prefixes_ = get_node()->get_parameter("robot_prefixes").as_string_array();
  robot_types_ = get_node()->get_parameter("robot_types").as_string_array();
  const std::string cartesian_velocity_interface_prefix =
      get_node()->get_parameter("cartesian_velocity_interface_prefix").as_string();
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          cartesian_velocity_interface_prefix, false);

  auto arm_prefixes_begin = robot_prefixes_.begin() + 1;
  arm_prefixes_ = std::vector<std::string>(arm_prefixes_begin, arm_prefixes_begin + 2);

  if (k.size() != 7 || d.size() != 7) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains and d_gains must be size 7");
    return CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i < kArmCommandInterfaces; ++i) {
    k_gains_(i) = k[i];
    d_gains_(i) = d[i];
  }

  auto params = param_listener_->get_params();
  joint_names_ = params.joints;

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

  q_.fill(Vector7d::Zero());
  dq_.fill(Vector7d::Zero());
  initial_q_.fill(Vector7d::Zero());
  dq_filtered_.fill(Vector7d::Zero());

  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);

  updateState(state_current_);
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  // add_new_trajectory_msg(set_hold_position());
  // rt_is_holding_ = true;

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
  // Precondition: Running controller
  if (get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // if (!validate_trajectory_msg(goal->trajectory)) {
  //   return rclcpp_action::GoalResponse::REJECT;
  // }

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MobileFR3DuoJointTrajectoryController::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle) {
  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    rt_has_pending_goal_ = false;
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

    // TODO (wink_ma):Enter hold current position mode
    // add_new_trajectory_msg(set_hold_position());
  }
  RCLCPP_INFO(get_node()->get_logger(), "Accepting request to cancel goal");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MobileFR3DuoJointTrajectoryController::goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle) {
  // Update tolerances if specified in the goal
  auto logger = this->get_node()->get_logger();

  // mark a pending goal
  rt_has_pending_goal_ = true;

  // Update new trajectory

  // TODO (wink_ma):
  // preempt_active_goal();
  auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

  add_new_trajectory_msg(traj_msg);
  rt_is_holding_ = false;

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = params_.joints;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // TODO (wink_ma):
  // active_tolerances_.writeFromNonRT(get_segment_tolerances(
  //     logger, default_tolerances_, *(goal_handle->get_goal()), params_.joints));

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ =
      get_node()->create_wall_timer(action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
  RCLCPP_INFO(logger, "Accepted new action goal");
}

bool MobileFR3DuoJointTrajectoryController::has_active_trajectory() const {
  return current_trajectory_ != nullptr && current_trajectory_->has_trajectory_msg();
}

void MobileFR3DuoJointTrajectoryController::add_new_trajectory_msg(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& traj_msg) {
  new_trajectory_msg_.writeFromNonRT(traj_msg);
}

std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
MobileFR3DuoJointTrajectoryController::set_hold_position() {
  // Command to stay at current position

  hold_position_msg_ptr_->points[0].positions = state_current_.positions;

  // set flag, otherwise tolerances will be checked with holding position too
  rt_is_holding_ = true;

  return hold_position_msg_ptr_;
}

std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
MobileFR3DuoJointTrajectoryController::set_success_trajectory_point() {
  // set last command to be repeated at success, no matter if it has nonzero velocity or
  // acceleration
  hold_position_msg_ptr_->points[0] = current_trajectory_->get_trajectory_msg()->points.back();
  hold_position_msg_ptr_->points[0].time_from_start = rclcpp::Duration(0, 0);

  // set flag, otherwise tolerances will be checked with success_trajectory_point too
  rt_is_holding_ = true;

  return hold_position_msg_ptr_;
}

void MobileFR3DuoJointTrajectoryController::updateState(
    trajectory_msgs::msg::JointTrajectoryPoint& state) {
  auto logger = get_node()->get_logger();
  auto clock = *get_node()->get_clock();

  // auto velocity_x = state_interfaces_[0].get_optional();
  // auto velocity_y = state_interfaces_[1].get_optional();
  // auto angular_velocity_z = state_interfaces_[5].get_optional();

  std::array<double, 3> planar_velocity = {0.0, 0.0, 0.0};
  // if (velocity_x && velocity_y && angular_velocity_z) {
  //   planar_velocity = {velocity_x.value(), velocity_y.value(), angular_velocity_z.value()};
  // } else {
  //   RCLCPP_WARN_THROTTLE(logger, clock, 1000, "Missing state for mobile base velocities");
  // }

  for (size_t arm_index = 0; arm_index < kArms; ++arm_index) {
    for (size_t i = 0; i < kArmCommandInterfaces; ++i) {
      size_t position_index = kBaseStateInterfaces + arm_index * kArmStateInterfaces + i * kArms;
      size_t velocity_index = position_index + 1;

      auto position = state_interfaces_[position_index].get_optional();
      auto velocity = state_interfaces_[velocity_index].get_optional();

      if (position && velocity) {
        q_[arm_index](i) = *position;
        dq_[arm_index](i) = *velocity;
      } else {
        RCLCPP_WARN_THROTTLE(logger, clock, 1000, "Missing state for arm_index %zu joint %zu",
                             arm_index, i);
      }
    }
  }

  size_t first_planar_joint_index = get_first_joint_index(joint_names_, "planar");
  size_t first_left_arm_joint_index = get_first_joint_index(joint_names_, "left");
  size_t first_right_arm_joint_index = get_first_joint_index(joint_names_, "right");

  state.positions.resize(joint_names_.size(), 0.0);
  state.velocities.resize(joint_names_.size(), 0.0);
  for (size_t i = 0; i < planar_velocity.size(); ++i) {
    state.velocities[i + first_planar_joint_index] = planar_velocity[i];
  }
  for (size_t i = 0; i < kArmCommandInterfaces; ++i) {
    state.positions[i + first_left_arm_joint_index] = q_[0](i);
    state.velocities[i + first_left_arm_joint_index] = dq_[0](i);
    state.positions[i + first_right_arm_joint_index] = q_[1](i);
    state.velocities[i + first_right_arm_joint_index] = dq_[1](i);
  }
}

void MobileFR3DuoJointTrajectoryController::commandArmPosition(const Vector7d& q_goal,
                                                               size_t arm_index) {
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

void MobileFR3DuoJointTrajectoryController::commandMobileBaseVelocity(
    const std::array<double, 3>& planar_base_velocities) {
  const Eigen::Vector3d linear{planar_base_velocities[0], planar_base_velocities[1], 0.0};
  const Eigen::Vector3d angular{0.0, 0.0, planar_base_velocities[2]};

  if (!franka_cartesian_velocity_->setCommand(linear, angular)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Failed to set tmr velocity");
  }
}

size_t MobileFR3DuoJointTrajectoryController::get_first_joint_index(
    const std::vector<std::string>& joint_names,
    const std::string& prefix) const {
  for (size_t i = 0; i < joint_names.size(); ++i) {
    if (joint_names[i].find(prefix) != std::string::npos) {
      return i;
    }
  }
  RCLCPP_WARN(get_node()->get_logger(),
              "Could not find joint index for prefix '%s' in trajectory joint names. Returning "
              "0 instead.",
              prefix.c_str());
  return 0;  // Default to 0 if not found
}

Vector7d MobileFR3DuoJointTrajectoryController::get_slice_of_trajectory_positions_arm(
    const trajectory_msgs::msg::JointTrajectoryPoint& command,
    size_t start_index) const {
  Vector7d slice = Vector7d::Zero();
  for (size_t i = 0; i < kArmCommandInterfaces; ++i) {
    if (start_index + i < command.positions.size()) {
      slice(i) = command.positions[start_index + i];
    } else {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Requested slice exceeds command positions size. Filling remaining with zeros.");
      break;
    }
  }
  return slice;
}

std::array<double, 3>
MobileFR3DuoJointTrajectoryController::get_slice_of_trajectory_velocities_base(
    const trajectory_msgs::msg::JointTrajectoryPoint& command,
    size_t start_index) const {
  std::array<double, 3> slice = {0.0, 0.0, 0.0};
  double w_z = command.velocities[start_index];
  double v_x = command.velocities[start_index + 1];
  double v_y = command.velocities[start_index + 2];
  slice[0] = v_x;
  slice[1] = v_y;
  slice[2] = w_z;
  return slice;
}

}  // namespace mobile_fr3_duo_joint_trajectory_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mobile_fr3_duo_joint_trajectory_controller::MobileFR3DuoJointTrajectoryController,
    controller_interface::ControllerInterface)
