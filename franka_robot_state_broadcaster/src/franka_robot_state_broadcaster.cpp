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

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <memory>
#include <string>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp>

namespace franka_robot_state_broadcaster {

FrankaRobotStateBroadcaster::FrankaRobotStateBroadcaster(
    std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state)
    : franka_robot_state_(std::move(franka_robot_state)) {}

FrankaRobotStateBroadcaster::~FrankaRobotStateBroadcaster() {
  stopPublishThread();
}

void FrankaRobotStateBroadcaster::startPublishThread() {
  if (is_publish_thread_running_) {
    return;
  }

  // Drain any stale data from the mailbox so the publish thread does not
  // immediately publish outdated state from a previous activation.
  bool had_stale_data = false;
  state_buffer_.get_active_buffer(had_stale_data);

  is_publish_thread_running_ = true;
  data_ready_.store(false, std::memory_order_relaxed);
  publish_thread_ = std::thread(&FrankaRobotStateBroadcaster::publishRunner, this);

  // Apply SCHED_FIFO so the publish thread is woken promptly after update()
  // signals data_ready_. Priority stays below the CM RT loop and NIC IRQs.
  sched_param sch{};
  sch.sched_priority = kPublishThreadPriority;
  if (pthread_setschedparam(publish_thread_.native_handle(), SCHED_FIFO, &sch) != 0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Could not set SCHED_FIFO priority %d on publish thread: %s. "
                "Publishing will run at normal priority — expect coalesced frames. "
                "Grant CAP_SYS_NICE or run as root to enable RT scheduling.",
                kPublishThreadPriority, strerror(errno));
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Publish thread started with SCHED_FIFO priority %d.",
                kPublishThreadPriority);
  }
}

void FrankaRobotStateBroadcaster::stopPublishThread() {
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    is_publish_thread_running_ = false;
  }
  publish_cv_.notify_all();
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = franka_robot_state_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();
  auto this_node = get_node();
  auto robot_description = get_robot_description();
  if (robot_description.empty()) {
    RCLCPP_ERROR(this_node->get_logger(), "Failed to get robot_description parameter");
    return CallbackReturn::ERROR;
  }

  if (!franka_robot_state_) {
    try {
      const std::string full_prefix = params_.arm_prefix.empty() ? "" : params_.arm_prefix + "_";
      const std::string hw_interface_name =
          full_prefix + params_.robot_type + "/" + state_interface_name_;

      franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
          hw_interface_name, robot_description);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this_node->get_logger(),
                   "Failed to resolve this arm's joints and frames from robot_description: %s",
                   e.what());
      return CallbackReturn::ERROR;
    }
  }

  const auto convenience_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  current_pose_stamped_publisher_ = this_node->create_publisher<geometry_msgs::msg::PoseStamped>(
      kCurrentPoseTopic, convenience_qos);
  last_desired_pose_stamped_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::PoseStamped>(kLastDesiredPoseTopic,
                                                                   convenience_qos);
  desired_end_effector_twist_stamped_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::TwistStamped>(kDesiredEETwist,
                                                                    convenience_qos);
  measured_joint_states_publisher_ = this_node->create_publisher<sensor_msgs::msg::JointState>(
      kMeasuredJointStates, convenience_qos);
  external_wrench_in_stiffness_frame_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::WrenchStamped>(
          kExternalWrenchInStiffnessFrame, convenience_qos);
  external_wrench_in_base_frame_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::WrenchStamped>(kExternalWrenchInBaseFrame,
                                                                     convenience_qos);
  external_joint_torques_publisher_ = this_node->create_publisher<sensor_msgs::msg::JointState>(
      kExternalJointTorques, convenience_qos);
  desired_joint_states_publisher_ = this_node->create_publisher<sensor_msgs::msg::JointState>(
      kDesiredJointStates, convenience_qos);

  try {
    franka_state_publisher_ = this_node->create_publisher<franka_msgs::msg::FrankaRobotState>(
        "~/" + state_interface_name_, rclcpp::SystemDefaultsQoS());

    // Initialize all three triple-buffer slots so get_values_as_message() never
    // writes into a default-constructed message with empty vectors.
    for (size_t i = 0; i < AsyncBuffer<franka_msgs::msg::FrankaRobotState>::kSize; ++i) {
      auto& msg = state_buffer_.get_free_buffer();
      franka_robot_state_->initialize_robot_state_msg(msg);
      state_buffer_.commit_free_buffer();
      bool consumed = false;
      state_buffer_.get_active_buffer(consumed);
    }
  } catch (const std::exception& e) {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage with message : %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }

  const int update_rate = static_cast<int>(get_update_rate());
  if (update_rate <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Update rate is %d Hz, so no publish rate can be derived from it.", update_rate);
    return CallbackReturn::ERROR;
  }

  const int requested_rate =
      std::min(static_cast<int>(params_.convenience_publish_rate), update_rate);
  convenience_publish_skip_ = std::max(1, update_rate / requested_rate);
  const int effective_rate = update_rate / convenience_publish_skip_;
  if (effective_rate != requested_rate) {
    RCLCPP_WARN(get_node()->get_logger(),
                "convenience_publish_rate %d Hz does not evenly divide update rate %d Hz. "
                "Effective rate: %d Hz.",
                requested_rate, update_rate, effective_rate);
  }
  RCLCPP_INFO(get_node()->get_logger(), "Convenience topics at %d Hz, full state at %d Hz",
              effective_rate, update_rate);

  if (is_async()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Running as an async controller. Prefer is_async: false so this broadcaster "
                "shares the controller manager thread with other readers of the robot state "
                "box; DDS publishes already run on a dedicated publish thread.");
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Could not claim or resolve the robot state interface. Check that 'robot_state' is "
        "listed among this controller's state interfaces and that the hardware exports it.");
    return CallbackReturn::ERROR;
  }
  startPublishThread();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  stopPublishThread();
  franka_robot_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrankaRobotStateBroadcaster::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  auto& free_state = state_buffer_.get_free_buffer();
  free_state.header.stamp = time;

  if (!franka_robot_state_->get_values_as_message(free_state)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get franka state via franka state interface.");
    return controller_interface::return_type::ERROR;
  }

  state_buffer_.commit_free_buffer();
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    data_ready_.store(true, std::memory_order_release);
  }
  publish_cv_.notify_one();

  return controller_interface::return_type::OK;
}

void FrankaRobotStateBroadcaster::publishRunner() {
  int convenience_counter = 0;

  while (true) {
    {
      std::unique_lock<std::mutex> lock(publish_mutex_);
      publish_cv_.wait(lock, [this] {
        return !is_publish_thread_running_ || data_ready_.load(std::memory_order_acquire);
      });
      if (!is_publish_thread_running_) {
        break;
      }
      data_ready_.store(false, std::memory_order_relaxed);
    }

    bool has_new_data = false;
    auto& state = state_buffer_.get_active_buffer(has_new_data);

    if (!has_new_data) {
      continue;
    }

    franka_state_publisher_->publish(state);

    if (++convenience_counter >= convenience_publish_skip_) {
      convenience_counter = 0;
      current_pose_stamped_publisher_->publish(state.o_t_ee);
      last_desired_pose_stamped_publisher_->publish(state.o_t_ee_d);
      desired_end_effector_twist_stamped_publisher_->publish(state.o_dp_ee_d);
      external_wrench_in_base_frame_publisher_->publish(state.o_f_ext_hat_k);
      external_wrench_in_stiffness_frame_publisher_->publish(state.k_f_ext_hat_k);
      measured_joint_states_publisher_->publish(state.measured_joint_state);
      external_joint_torques_publisher_->publish(state.tau_ext_hat_filtered);
      desired_joint_states_publisher_->publish(state.desired_joint_state);
    }
  }
}

}  // namespace franka_robot_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_robot_state_broadcaster::FrankaRobotStateBroadcaster,
                       controller_interface::ControllerInterface)
