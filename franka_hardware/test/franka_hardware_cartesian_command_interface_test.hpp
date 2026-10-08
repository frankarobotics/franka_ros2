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

#include <gmock/gmock.h>
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

#include <franka_hardware_mocks/franka_hardware_robot_mock.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "franka/exception.h"
#include "test_utils.hpp"

class FrankaCartesianCommandInterfaceTest
    : public ::testing::TestWithParam<std::tuple<std::vector<std::string>, std::string>> {
 public:
  auto SetUp() -> void override {
    auto filename = TEST_CASE_DIRECTORY + robot_type + ".urdf";
    auto urdf_string = readFileToString(filename);
    auto parsed_hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf_string);
    auto number_of_expected_hardware_components = 1;

    ASSERT_EQ(parsed_hardware_infos.size(), number_of_expected_hardware_components);

    default_hardware_info = parsed_hardware_infos[0];

    // Wrap the driver in hardware_interface::System, as the real resource_manager does, and
    // export interfaces exactly once. This is required before read()/write() are called: they
    // now look values up by name (set_state()/get_command()) instead of writing through an
    // aliased raw pointer, and that lookup only works once on_export_*_interfaces() has run - and
    // must only be exported once, since each export call builds fresh interface objects.
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = default_hardware_info;
    params.clock = std::make_shared<rclcpp::Clock>();
    params.logger = rclcpp::get_logger("franka_hardware_cartesian_command_interface_test");

    hw_ = std::make_unique<hardware_interface::System>(std::move(franka_driver_));
    const auto state = hw_->initialize(params);
    ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    hw_->export_state_interfaces();
    hw_->export_command_interfaces();
  }

 protected:
  std::string robot_type{"fr3"};
  std::shared_ptr<MockRobot> default_mock_robot = std::make_shared<MockRobot>();
  hardware_interface::HardwareInfo default_hardware_info;

  // franka_driver_ is moved into hw_ in SetUp(). default_franka_hardware_interface stays a valid
  // reference to the same object afterwards (now owned by hw_), so every existing call site below
  // that calls a method directly on it (bypassing the wrapper's own lifecycle-state gating, same
  // as before this migration) keeps working unchanged.
  std::unique_ptr<franka_hardware::FrankaHardwareInterface> franka_driver_ =
      std::make_unique<franka_hardware::FrankaHardwareInterface>(default_mock_robot, robot_type);
  franka_hardware::FrankaHardwareInterface& default_franka_hardware_interface = *franka_driver_;
  std::unique_ptr<hardware_interface::System> hw_;

  const std::vector<std::string> k_hw_cartesian_pose_names{
      "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"};
  const std::vector<std::string> k_hw_cartesian_velocities_names{"vx", "vy", "vz",
                                                                 "wx", "wy", "wz"};
  const std::vector<std::string> k_hw_elbow_command_names{"joint_3_position", "joint_4_sign"};

  const std::string k_joint_name{"joint"};
  const size_t k_number_of_joints{7};

  const std::string k_cartesian_velocity_command_interface_name{"cartesian_velocity"};
  const std::string k_cartesian_pose_command_interface_name{"cartesian_pose_command"};
  const std::string k_elbow_command_interface_name{"elbow_command"};
};

INSTANTIATE_TEST_SUITE_P(
    FrankaCartesianCommandTest,
    FrankaCartesianCommandInterfaceTest,
    ::testing::Values(std::make_tuple(std::vector<std::string>{"vx", "vy", "vz", "wx", "wy", "wz"},
                                      "cartesian_velocity"),
                      std::make_tuple(std::vector<std::string>{"0", "1", "2", "3", "4", "5", "6",
                                                               "7", "8", "9", "10", "11", "12",
                                                               "13", "14", "15"},
                                      "cartesian_pose_command"),
                      std::make_tuple(std::vector<std::string>{"joint_3_position", "joint_4_sign"},
                                      "elbow_command")));
