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

#include <franka_example_controllers/mobile_fr3_duo/mobile_fr3_duo_joint_impedance_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
MobileFr3DuoJointImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names = {"vx/cartesian_velocity", "vy/cartesian_velocity", "vz/cartesian_velocity",
                  "wx/cartesian_velocity", "wy/cartesian_velocity", "wz/cartesian_velocity"};

  for (size_t index = 0; index < arm_prefixes_.size(); ++index) {
    for (int i = 1; i <= num_arm_joints; ++i) {
      config.names.push_back(arm_prefixes_[index] + "_" + robot_types_[index + 1] + "_joint" +
                             std::to_string(i) + "/effort");
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
MobileFr3DuoJointImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 0; i < num_base_joints; ++i) {
      config.names.push_back(robot_types_[0] + "_joint_" + std::to_string(i) + "/position");
      config.names.push_back(robot_types_[0] + "_joint_" + std::to_string(i) + "/velocity");
    }

  for (int arm = 0; arm < 2; ++arm) {
    for (int i = 1; i <= num_arm_joints; ++i) {
      std::string prefix =
          arm_prefixes_[arm] + "_" + robot_types_[arm + 1] + "_joint" + std::to_string(i);
      config.names.push_back(prefix + "/position");
      config.names.push_back(prefix + "/velocity");
    }
  }

  return config;
}

controller_interface::return_type MobileFr3DuoJointImpedanceExampleController::update(
    const rclcpp::Time&,
    const rclcpp::Duration& period) {
  updateJointStates();
  elapsed_time_ += period.seconds();

  for (size_t arm = 0; arm < 2; ++arm) {
    Vector7d q_goal = initial_q_[arm];

    double delta = M_PI / 8.0 * (1.0 - std::cos(M_PI / 2.5 * elapsed_time_));
    q_goal(3) += delta;
    q_goal(4) += delta;

    constexpr double kAlpha = 0.99;
    dq_filtered_[arm] = (1.0 - kAlpha) * dq_filtered_[arm] + kAlpha * dq_[arm];

    Vector7d tau =
        k_gains_.cwiseProduct(q_goal - q_[arm]) + d_gains_.cwiseProduct(-dq_filtered_[arm]);

    size_t cmd_offset = kBaseCommandInterfaces_ + arm * kArmCommandInterfaces;
    for (size_t j = 0; j < 7; ++j) {
      if (!command_interfaces_[cmd_offset + j].set_value(tau(j))) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Failed to set torque for arm %zu joint %zu", arm, j);
      }
    }
  }

  updateMobileBaseCommand(period);

  return controller_interface::return_type::OK;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_init() {
  try {
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<std::string>>("arm_prefixes", {});
    auto_declare<std::vector<std::string>>("robot_prefixes", {});
    auto_declare<std::vector<std::string>>("robot_types", {});
    auto_declare<double>("wheel_radius", 0.1);
  } catch (...) {
    return CallbackReturn::ERROR;
  }


  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State&) {
  auto k = get_node()->get_parameter("k_gains").as_double_array();
  auto d = get_node()->get_parameter("d_gains").as_double_array();
  robot_prefixes_ = get_node()->get_parameter("robot_prefixes").as_string_array();
  robot_types_ = get_node()->get_parameter("robot_types").as_string_array();
  // wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();

  kBaseStateInterfaces_ = kBaseStateInterfacesHardware;
  kBaseCommandInterfaces_ = kBaseCommandInterfacesHardware;

  auto arm_prefixes_begin = robot_prefixes_.begin() + 1;
  arm_prefixes_ = std::vector<std::string>(arm_prefixes_begin, arm_prefixes_begin + 2);

  if (k.size() != 7 || d.size() != 7) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains and d_gains must be size 7");
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_arm_joints; ++i) {
    k_gains_(i) = k[i];
    d_gains_(i) = d[i];
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State&) {
  q_.resize(2, Vector7d::Zero());
  dq_.resize(2, Vector7d::Zero());
  dq_filtered_.resize(2, Vector7d::Zero());
  initial_q_.resize(2, Vector7d::Zero());

  updateJointStates();
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  return CallbackReturn::SUCCESS;
}

void MobileFr3DuoJointImpedanceExampleController::updateJointStates() {
  for (int arm = 0; arm < 2; ++arm) {
    for (auto i = 0; i < num_arm_joints; ++i) {
      int pos = kBaseStateInterfaces_ + arm * kArmStateInterfaces + i * 2;
      int vel = pos + 1;

      auto p = state_interfaces_[pos].get_optional();
      auto v = state_interfaces_[vel].get_optional();

      if (p && v) {
        q_[arm](i) = *p;
        dq_[arm](i) = *v;
      } else {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Missing state for arm %d joint %d", static_cast<int>(arm), i);
      }
    }
  }
}

void MobileFr3DuoJointImpedanceExampleController::updateMobileBaseCommand(const rclcpp::Duration&) {
  double cycle = std::floor(std::pow(
      -1.0, (elapsed_time_ - std::fmod(elapsed_time_, k_mobile_time_max_)) / k_mobile_time_max_));

  double v = cycle * k_mobile_v_max_ / 2.0 *
             (1.0 - std::cos(2.0 * M_PI / k_mobile_time_max_ * elapsed_time_));

  double v_x = std::cos(k_mobile_angle_) * v;
  double v_y = std::sin(k_mobile_angle_) * v;
  const double wz = 0.0;

  std::array<double, 6> values = {v_x, v_y, 0.0, 0.0, 0.0, wz};
  std::array<std::string, 6> labels = {"vx", "vy", "vz", "wx", "wy", "wz"};
  for (int i = 0; i < kBaseCommandInterfacesHardware; ++i) {
    if (!command_interfaces_[i].set_value(values[i])) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "Failed to set %s velocity", labels[i].c_str());
    }
  }
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MobileFr3DuoJointImpedanceExampleController,
                       controller_interface::ControllerInterface)
