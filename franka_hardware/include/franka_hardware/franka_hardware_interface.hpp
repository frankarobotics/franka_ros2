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

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include <hardware_interface/visibility_control.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "franka_action_server.hpp"
#include "franka_hardware/franka_executor.hpp"
#include "franka_hardware/franka_param_service_server.hpp"
#include "franka_hardware/robot.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_hardware {

enum class ControlInterface {
  None,
  Effort,
  JointVelocity,
  JointPosition,
  CartesianVelocity,
  CartesianVelocityWithElbow,
  CartesianPose,
  CartesianPoseWithElbow
};

class FrankaHardwareInterface : public hardware_interface::SystemInterface {
 public:
  explicit FrankaHardwareInterface(const std::shared_ptr<Robot>& robot,
                                   const std::string& robot_type);
  FrankaHardwareInterface();
  FrankaHardwareInterface(const FrankaHardwareInterface&) = delete;
  FrankaHardwareInterface& operator=(const FrankaHardwareInterface& other) = delete;
  FrankaHardwareInterface& operator=(FrankaHardwareInterface&& other) = delete;
  FrankaHardwareInterface(FrankaHardwareInterface&& other) = delete;
  ~FrankaHardwareInterface() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  static const size_t kNumberOfJoints = 7;

 private:
  struct InterfaceInfo {
    std::string interface_type;
    size_t size;
    bool& claim_flag;
  };

  void initializePositionCommands(const franka::RobotState& robot_state);

  // Support Franka ros2 control interface version
  const int kSupportedControlInterfaceMajor = 1;

  ControlInterface active_mode_{ControlInterface::None};
  bool needs_initial_command_{true};
  double robot_time_state_{0.0};

  std::shared_ptr<Robot> robot_;
  std::shared_ptr<FrankaParamServiceServer> service_node_;
  std::shared_ptr<ActionServer> action_node_;
  std::shared_ptr<FrankaExecutor> executor_;

  // Torque joint commands for the effort command interface
  std::vector<double> hw_effort_commands_{0, 0, 0, 0, 0, 0, 0};
  // Position joint commands for the position command interface
  std::vector<double> hw_position_commands_{0, 0, 0, 0, 0, 0, 0};
  // Velocity joint commands for the position command interface
  std::vector<double> hw_velocity_commands_{0, 0, 0, 0, 0, 0, 0};

  // Robot joint states
  std::array<double, kNumberOfJoints> hw_positions_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, kNumberOfJoints> hw_velocities_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, kNumberOfJoints> hw_efforts_{0, 0, 0, 0, 0, 0, 0};

  // Cartesian States
  std::array<double, 16> cartesian_pose_state_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::array<double, 2> elbow_state_{0, 0};

  // External wrench estimated in stiffness frame (K_F_ext_hat_K)
  std::array<double, 6> force_torque_sensor_state_{0, 0, 0, 0, 0, 0};

  /**
   * Desired Cartesian velocity with respect to the o-frame
   * "base frame O" with (vx, vy, vz)\ in [m/s] and
   * (wx, wy, wz) in [rad/s].
   */
  const std::string k_HW_IF_CARTESIAN_VELOCITY = "cartesian_velocity";
  std::vector<double> hw_cartesian_velocities_{0, 0, 0, 0, 0, 0};

  // Pose is represented as a column-major homogeneous transformation matrix.
  const std::string k_HW_IF_CARTESIAN_POSE_COMMAND = "cartesian_pose_command";
  std::vector<double> hw_cartesian_pose_commands_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  /**
   * Elbow configuration.
   *
   * The values of the array are:
   *  - elbow[0]: Position of the 3rd joint in \f$[rad]\f$.
   *  - elbow[1]: Flip direction of the elbow (4th joint):
   *    - +1 if \f$q_4 > \alpha\f$
   *    - 0 if \f$q_4 == \alpha \f$
   *    - -1 if \f$q_4 < \alpha \f$
   *    .
   *    with \f$\alpha = -0.467002423653011\f$ \f$rad\f$
   */
  const std::string k_HW_IF_ELBOW_COMMAND = "elbow_command";
  std::vector<double> hw_elbow_command_{0, 0};

  std::map<std::string, std::vector<double>&> command_interface_map_{
      {hardware_interface::HW_IF_EFFORT, hw_effort_commands_},
      {hardware_interface::HW_IF_POSITION, hw_position_commands_},
      {hardware_interface::HW_IF_VELOCITY, hw_velocity_commands_},
      {k_HW_IF_CARTESIAN_VELOCITY, hw_cartesian_velocities_},
      {k_HW_IF_CARTESIAN_POSE_COMMAND, hw_cartesian_pose_commands_},
      {k_HW_IF_ELBOW_COMMAND, hw_elbow_command_}};

  std::array<std::string, 2> hw_elbow_command_names_{"joint_3_position", "joint_4_sign"};

  std::array<std::string, 2> elbow_state_names_{"joint_3_position", "joint_4_sign"};

  const std::string k_HW_IF_ELBOW_STATE = "elbow_state";
  const std::string k_HW_IF_CARTESIAN_POSE_STATE = "cartesian_pose_state";

  const std::vector<InterfaceInfo> command_interfaces_info_;

  franka::RobotState hw_franka_robot_state_;
  franka::RobotState* hw_franka_robot_state_addr_ = &hw_franka_robot_state_;
  Model* hw_franka_model_ptr_ = nullptr;
  franka::Duration robot_time_;

  bool effort_interface_claimed_ = false;
  bool velocity_joint_interface_claimed_ = false;
  bool position_joint_interface_claimed_ = false;
  bool velocity_cartesian_interface_claimed_ = false;
  bool pose_cartesian_interface_claimed_ = false;
  bool elbow_command_interface_claimed_ = false;

  static rclcpp::Logger getLogger();

  std::string robot_ip_;
  std::string robot_type_{"fr3"};
  std::string prefix_;
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
  const size_t max_number_start_interfaces = 45;

  std::unordered_set<std::string> exported_command_interfaces_;
};
}  // namespace franka_hardware
