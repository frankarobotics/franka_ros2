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

#include <rcutils/logging_macros.h>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcpputils/split.hpp>

#include <franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp>

namespace franka_robot_state_broadcaster {

FrankaRobotStateBroadcaster::FrankaRobotStateBroadcaster(
    std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state)
    : franka_robot_state_(std::move(franka_robot_state)) {}

FrankaRobotStateBroadcaster::~FrankaRobotStateBroadcaster() {
  stopPublishThread();
}

void FrankaRobotStateBroadcaster::startPublishThread() {
  if (!is_publish_thread_running_) {
    // Drain any stale data from the mailbox so the publish thread doesn't
    // immediately publish outdated state from a previous activation.
    bool had_stale_data = false;
    state_buffer_.get_active_buffer(had_stale_data);

    is_publish_thread_running_ = true;
    data_ready_.store(false, std::memory_order_relaxed);
    publish_thread_ = std::thread(&FrankaRobotStateBroadcaster::publishRunner, this);

    // Apply SCHED_FIFO so the publish thread is woken within microseconds of
    // update() signalling data_ready_. Priority is intentionally below the
    // ros2_control RT control loop (typically SCHED_FIFO 70-80) so the RT
    // thread is never preempted by publishing.
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
}

void FrankaRobotStateBroadcaster::stopPublishThread() {
  is_publish_thread_running_ = false;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_init() {
  try {
    param_listener = std::make_shared<ParamListener>(get_node());
    params = param_listener->get_params();
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
  params = param_listener->get_params();
  auto this_node = get_node();
  std::string robot_description;
  if (!this_node->get_parameter("robot_description", robot_description)) {
    RCLCPP_ERROR(this_node->get_logger(), "Failed to get robot_description parameter");
    return CallbackReturn::ERROR;
  }

  if (!franka_robot_state_) {
    std::string full_prefix = params.arm_prefix.empty() ? "" : params.arm_prefix + "_";
    std::string hw_interface_name = full_prefix + params.robot_type + "/" + state_interface_name;
    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
        franka_semantic_components::FrankaRobotState(hw_interface_name, robot_description));
  }

  auto convenience_qos = rclcpp::QoS(1).best_effort();

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
    franka_state_publisher = this_node->create_publisher<franka_msgs::msg::FrankaRobotState>(
        "~/" + state_interface_name, rclcpp::SystemDefaultsQoS());

    // Initialize all three triple-buffer slots so that get_values_as_message()
    // never writes into a default-constructed message with empty vectors.
    for (int i = 0; i < 3; ++i) {
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

  convenience_publish_rate_ =
      std::min(static_cast<int>(params.convenience_publish_rate), kUpdateRate);
  int skip = std::max(1, kUpdateRate / convenience_publish_rate_);
  int effective_rate = kUpdateRate / skip;
  if (effective_rate != convenience_publish_rate_) {
    RCLCPP_WARN(get_node()->get_logger(),
                "convenience_publish_rate %d Hz does not evenly divide update rate %d Hz. "
                "Effective rate: %d Hz.",
                convenience_publish_rate_, kUpdateRate, effective_rate);
  }
  RCLCPP_INFO(get_node()->get_logger(), "Convenience topics at %d Hz, full state at %d Hz",
              effective_rate, kUpdateRate);

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
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

  data_ready_.store(true, std::memory_order_release);

  return controller_interface::return_type::OK;
}

void FrankaRobotStateBroadcaster::publishRunner() {
  // Publish convenience topics every N-th cycle where N = kUpdateRate / convenience_publish_rate_.
  const int skip = std::max(1, kUpdateRate / convenience_publish_rate_);
  int convenience_counter = 0;

  while (is_publish_thread_running_) {
    if (!data_ready_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::microseconds(kPublishThreadSleepUs));
      continue;
    }
    data_ready_.store(false, std::memory_order_relaxed);

    bool has_new_data = false;
    auto& state = state_buffer_.get_active_buffer(has_new_data);

    // Publishers are guaranteed to be valid here: the thread only runs between
    // on_activate (after on_configure creates publishers) and on_deactivate.
    if (has_new_data) {
      // Full state always publishes at the update rate (1kHz).
      franka_state_publisher->publish(state);

      // Convenience topics publish at convenience_publish_rate_ Hz.
      if (++convenience_counter >= skip) {
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
}

}  // namespace franka_robot_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_robot_state_broadcaster::FrankaRobotStateBroadcaster,
                       controller_interface::ControllerInterface)
