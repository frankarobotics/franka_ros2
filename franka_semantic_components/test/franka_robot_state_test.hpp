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

#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <sstream>
#include "franka/robot_state.h"
#include "franka_semantic_components/franka_robot_state.hpp"
#include "gmock/gmock.h"

inline std::string read_robot_description(const std::string& package_name) {
  std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
  std::string file_path = package_path + "/robot_description_test.txt";

  std::ifstream file(file_path);
  if (!file) {
    throw std::runtime_error("Failed to open file: " + file_path);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();

  return buffer.str();
}

// Splices a second seven-joint arm into a single-arm description, so that one robot_description
// holds more than seven revolute joints - the shape a duo bringup hands to the controller
// manager. The extra arm needs no accelerometer frames; only the arm under test is read out.
inline std::string add_second_arm(const std::string& robot_description,
                                  const std::string& arm_prefix,
                                  const std::string& mount_link) {
  std::stringstream arm;
  arm << "<link name=\"" << arm_prefix << "_link0\"/>";
  arm << "<joint name=\"" << arm_prefix << "_mount_joint\" type=\"fixed\">"
      << "<parent link=\"" << mount_link << "\"/>"
      << "<child link=\"" << arm_prefix << "_link0\"/></joint>";
  for (size_t joint = 1; joint <= 7; ++joint) {
    arm << "<link name=\"" << arm_prefix << "_link" << joint << "\"/>";
    arm << "<joint name=\"" << arm_prefix << "_joint" << joint << "\" type=\"revolute\">"
        << "<parent link=\"" << arm_prefix << "_link" << joint - 1 << "\"/>"
        << "<child link=\"" << arm_prefix << "_link" << joint << "\"/>"
        << "<axis xyz=\"0 0 1\"/>"
        << "<limit lower=\"-2.0\" upper=\"2.0\" effort=\"87.0\" velocity=\"2.0\"/></joint>";
  }

  const auto closing_tag = robot_description.rfind("</robot>");
  if (closing_tag == std::string::npos) {
    throw std::runtime_error("robot_description has no closing </robot> tag.");
  }
  return robot_description.substr(0, closing_tag) + arm.str() +
         robot_description.substr(closing_tag);
}

// implementing and friending so we can access member variables
class FrankaRobotStateTestFriend : public franka_semantic_components::FrankaRobotState {
  FRIEND_TEST(FrankaRobotStateTest, validate_state_names_and_size);
  FRIEND_TEST(FrankaRobotStateTest,
              givenFrankaSemanticStateInitialized_whenMessageReturnedExpectsCorrectValues);
  FRIEND_TEST(FrankaRobotStateTest, robot_state_ptr_uncasted_correctly);
  FRIEND_TEST(FrankaRobotStateTest, a_null_state_box_fails_assignment);

 public:
  // Use generation of interface names
  explicit FrankaRobotStateTestFriend(const std::string& name)
      : franka_semantic_components::FrankaRobotState(
            name,
            read_robot_description("franka_semantic_components")) {}

  virtual ~FrankaRobotStateTestFriend() = default;
};

// Same component, but built from a robot_description that describes two arms.
class FrankaRobotStateMultiArmTestFriend : public franka_semantic_components::FrankaRobotState {
 public:
  explicit FrankaRobotStateMultiArmTestFriend(const std::string& name)
      : franka_semantic_components::FrankaRobotState(
            name,
            add_second_arm(read_robot_description("franka_semantic_components"),
                           "second_fr3",
                           "fr3_link0")) {}

  virtual ~FrankaRobotStateMultiArmTestFriend() = default;
};

class FrankaRobotStateTest : public ::testing::Test {
 public:
  void SetUp();

  void TearDown();

 protected:
  const size_t size = 1;
  const std::string robot_name = "fr3";
  const std::string franka_state_interface_name = "robot_state";
  franka::RobotState robot_state;
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState> robot_state_box;
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState>* robot_state_box_ptr = &robot_state_box;

  std::array<double, 7> joint_angles = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 7> joint_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 16> end_effector_pose = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,   0.0,
                                              0.0, 0.0, 1.0, 0.0, 2.2, 3.8, 93.23, 1.0};
  franka::RobotMode robot_mode = franka::RobotMode::kUserStopped;
  franka_msgs::msg::FrankaRobotState franka_robot_state_msg;

  std::unique_ptr<FrankaRobotStateTestFriend> franka_state_friend;

  std::vector<std::string> full_interface_names;
};

class FrankaRobotStateMultiArmTest : public ::testing::Test {
 public:
  void SetUp();

  void TearDown();

 protected:
  const size_t size = 1;
  const std::string robot_name = "fr3";
  const std::string franka_state_interface_name = "robot_state";
  franka::RobotState robot_state;
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState> robot_state_box;
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState>* robot_state_box_ptr = &robot_state_box;

  // Distinct per joint, so a value that came from the wrong libfranka array is visible.
  std::array<double, 7> joint_angles = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  std::array<double, 7> desired_joint_angles = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7};
  franka_msgs::msg::FrankaRobotState franka_robot_state_msg;

  std::unique_ptr<FrankaRobotStateMultiArmTestFriend> franka_state_friend;
};
