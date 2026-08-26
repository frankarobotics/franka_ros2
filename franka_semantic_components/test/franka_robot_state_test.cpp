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

#include "franka_robot_state_test.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <research_interface/robot/error.h>
#include <research_interface/robot/rbk_types.h>

void FrankaRobotStateTest::TearDown() {
  franka_state_friend.reset(nullptr);
}

void FrankaRobotStateTest::SetUp() {
  full_interface_names.reserve(size);
  full_interface_names.emplace_back(robot_name + "/" + franka_state_interface_name);
  franka_state_friend =
      std::make_unique<FrankaRobotStateTestFriend>(robot_name + "/" + franka_state_interface_name);

  std::vector<std::string> interface_names = franka_state_friend->get_state_interface_names();

  robot_state.q = joint_angles;
  robot_state.q_d = joint_velocities;
  robot_state.O_T_EE = end_effector_pose;
  robot_state.robot_mode = robot_mode;
  robot_state_box.set(robot_state);

  hardware_interface::StateInterface franka_hw_state{
      robot_name, franka_state_interface_name, reinterpret_cast<double*>(&robot_state_box_ptr)};
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;

  temp_state_interfaces.reserve(size);

  temp_state_interfaces.emplace_back(franka_hw_state);
  ASSERT_TRUE(franka_state_friend->assign_loaned_state_interfaces(temp_state_interfaces));
  franka_state_friend->initialize_robot_state_msg(franka_robot_state_msg);
  ASSERT_TRUE(franka_state_friend->get_values_as_message(franka_robot_state_msg));
}

TEST_F(FrankaRobotStateTest, validate_state_names_and_size) {
  ASSERT_EQ(franka_state_friend->name_, robot_name + "/" + franka_state_interface_name);

  ASSERT_EQ(franka_state_friend->interface_names_.size(), size);
  ASSERT_EQ(franka_state_friend->state_interfaces_.capacity(), size);

  ASSERT_TRUE(std::equal(franka_state_friend->interface_names_.begin(),
                         franka_state_friend->interface_names_.end(), full_interface_names.begin(),
                         full_interface_names.end()));
  ASSERT_EQ(franka_state_friend->state_interfaces_.size(), size);
  franka_state_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_state_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotStateTest, robot_state_ptr_uncasted_correctly) {
  auto robot_state_ptr = franka_state_friend->get_robot_state();
  ASSERT_NE(robot_state_ptr, nullptr);
  ASSERT_EQ(robot_state_ptr->q, robot_state.q);
  franka_state_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_state_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotStateTest,
       givenFrankaSemanticStateInitialized_whenMessageReturnedExpectsCorrectValues) {
  ASSERT_THAT(joint_angles,
              ::testing::ElementsAreArray(franka_robot_state_msg.measured_joint_state.position));
  ASSERT_THAT(joint_velocities,
              ::testing::ElementsAreArray(franka_robot_state_msg.desired_joint_state.position));

  ASSERT_EQ(end_effector_pose[12], franka_robot_state_msg.o_t_ee.pose.position.x);
  ASSERT_EQ(end_effector_pose[13], franka_robot_state_msg.o_t_ee.pose.position.y);
  ASSERT_EQ(end_effector_pose[14], franka_robot_state_msg.o_t_ee.pose.position.z);

  ASSERT_EQ(franka_msgs::msg::FrankaRobotState::ROBOT_MODE_USER_STOPPED,
            franka_robot_state_msg.robot_mode);
  franka_state_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_state_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotStateTest, a_released_buffer_stops_the_message_from_being_built) {
  franka_state_friend->release_interfaces();

  ASSERT_FALSE(franka_state_friend->get_values_as_message(franka_robot_state_msg));
}

TEST_F(FrankaRobotStateTest, a_null_state_box_fails_assignment) {
  franka_state_friend->release_interfaces();

  realtime_tools::RealtimeThreadSafeBox<franka::RobotState>* null_box_ptr = nullptr;
  hardware_interface::StateInterface franka_hw_state{robot_name, franka_state_interface_name,
                                                     reinterpret_cast<double*>(&null_box_ptr)};
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.emplace_back(franka_hw_state);

  ASSERT_FALSE(franka_state_friend->assign_loaned_state_interfaces(temp_state_interfaces));
  ASSERT_EQ(franka_state_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotStateTest, the_cached_box_keeps_serving_fresh_state) {
  // The interface is only consulted at assignment, so later writes have to reach the
  // message through the cached box rather than through another lookup.
  std::array<double, 7> moved_angles = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  robot_state.q = moved_angles;
  robot_state_box.set(robot_state);

  ASSERT_TRUE(franka_state_friend->get_values_as_message(franka_robot_state_msg));
  ASSERT_THAT(moved_angles,
              ::testing::ElementsAreArray(franka_robot_state_msg.measured_joint_state.position));
}

TEST_F(FrankaRobotStateTest,
       givenReflexRobotStateWithErrors_whenMessageReturned_thenReflexModeAndErrorsArePublished) {
  auto current_error_flags =
      std::array<bool, sizeof(research_interface::robot::RobotState::errors)>{};
  current_error_flags[static_cast<size_t>(research_interface::robot::Error::kJointReflex)] = true;
  current_error_flags[static_cast<size_t>(
      research_interface::robot::Error::kForceControlSafetyViolation)] = true;

  auto last_motion_error_flags =
      std::array<bool, sizeof(research_interface::robot::RobotState::errors)>{};
  last_motion_error_flags[static_cast<size_t>(research_interface::robot::Error::kCartesianReflex)] =
      true;
  last_motion_error_flags[static_cast<size_t>(
      research_interface::robot::Error::kControllerTorqueDiscontinuity)] = true;

  robot_state.robot_mode = franka::RobotMode::kReflex;
  robot_state.current_errors = franka::Errors(current_error_flags);
  robot_state.last_motion_errors = franka::Errors(last_motion_error_flags);
  robot_state_box.set(robot_state);

  ASSERT_TRUE(franka_state_friend->get_values_as_message(franka_robot_state_msg));

  EXPECT_EQ(franka_msgs::msg::FrankaRobotState::ROBOT_MODE_REFLEX,
            franka_robot_state_msg.robot_mode);
  EXPECT_TRUE(franka_robot_state_msg.current_errors.joint_reflex);
  EXPECT_TRUE(franka_robot_state_msg.current_errors.force_control_safety_violation);
  EXPECT_TRUE(franka_robot_state_msg.last_motion_errors.cartesian_reflex);
  EXPECT_TRUE(franka_robot_state_msg.last_motion_errors.controller_torque_discontinuity);

  franka_state_friend->release_interfaces();
}

TEST_F(FrankaRobotStateTest, givenInitializedRobotStateMsg_thenCorrectFrameIDs) {
  franka_state_friend->initialize_robot_state_msg(franka_robot_state_msg);

  ASSERT_EQ(franka_robot_state_msg.o_t_ee.header.frame_id, "fr3_link0");
  ASSERT_EQ(franka_robot_state_msg.ee_t_k.header.frame_id, "fr3_hand_tcp");
  ASSERT_EQ(franka_robot_state_msg.measured_joint_state.name[1], "fr3_joint2");
  ASSERT_EQ(franka_robot_state_msg.k_f_ext_hat_k.header.frame_id, "fr3_hand_tcp");
  ASSERT_EQ(franka_robot_state_msg.o_f_ext_hat_k.header.frame_id, "fr3_link0");
  ASSERT_EQ(franka_robot_state_msg.o_dp_ee_c.header.frame_id, "fr3_link0");
  ASSERT_EQ(franka_robot_state_msg.o_ddp_ee_c.header.frame_id, "fr3_link0");
}

TEST_F(FrankaRobotStateTest, givenInitializedRobotStateMsg_thenCorrectlySizedDynamicVectors) {
  franka_state_friend->initialize_robot_state_msg(franka_robot_state_msg);
  auto expected_size = franka_robot_state_msg.measured_joint_state.name.size();

  ASSERT_EQ(franka_robot_state_msg.desired_joint_state.position.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.desired_joint_state.velocity.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.desired_joint_state.effort.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_state.position.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_state.velocity.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_state.effort.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_motor_state.position.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_motor_state.velocity.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_motor_state.effort.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.tau_ext_hat_filtered.position.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.tau_ext_hat_filtered.velocity.size(), expected_size);
  ASSERT_EQ(franka_robot_state_msg.tau_ext_hat_filtered.effort.size(), expected_size);
}

TEST_F(FrankaRobotStateTest,
       givenAccelerometerRobotState_whenMessageReturned_thenOrderedValuesFramesAndStamps) {
  constexpr size_t kSensorCount = 6;
  constexpr size_t kAxisCount = 3;
  std::array<std::array<float, kAxisCount>, kSensorCount> top_readings;
  std::array<std::array<float, kAxisCount>, kSensorCount> bottom_readings;
  for (size_t sensor = 0; sensor < kSensorCount; ++sensor) {
    for (size_t axis = 0; axis < kAxisCount; ++axis) {
      // Exact binary fractions keep the values distinct per sensor and axis while
      // staying safe for exact-equality assertions.
      top_readings[sensor][axis] = kAxisCount * sensor + axis + 0.125;
      bottom_readings[sensor][axis] = -(kAxisCount * sensor + axis + 0.25);
    }
  }
  robot_state.accelerometer_top = top_readings;
  robot_state.accelerometer_bottom = bottom_readings;
  robot_state_box.set(robot_state);

  builtin_interfaces::msg::Time requested_stamp;
  requested_stamp.sec = 123;
  requested_stamp.nanosec = 456789;
  franka_robot_state_msg.header.stamp = requested_stamp;

  ASSERT_TRUE(franka_state_friend->get_values_as_message(franka_robot_state_msg));

  ASSERT_EQ(franka_robot_state_msg.accelerometer_top.size(), kSensorCount);
  ASSERT_EQ(franka_robot_state_msg.accelerometer_bottom.size(), kSensorCount);
  for (size_t sensor = 0; sensor < kSensorCount; ++sensor) {
    const auto expected_frame = robot_name + "_link" + std::to_string(sensor) + "_accelerometer_";
    ASSERT_EQ(franka_robot_state_msg.accelerometer_top[sensor].header.frame_id,
              expected_frame + "top");
    ASSERT_EQ(franka_robot_state_msg.accelerometer_bottom[sensor].header.frame_id,
              expected_frame + "bottom");

    ASSERT_EQ(franka_robot_state_msg.accelerometer_top[sensor].header.stamp.sec,
              requested_stamp.sec);
    ASSERT_EQ(franka_robot_state_msg.accelerometer_top[sensor].header.stamp.nanosec,
              requested_stamp.nanosec);
    ASSERT_EQ(franka_robot_state_msg.accelerometer_bottom[sensor].header.stamp.sec,
              requested_stamp.sec);
    ASSERT_EQ(franka_robot_state_msg.accelerometer_bottom[sensor].header.stamp.nanosec,
              requested_stamp.nanosec);

    ASSERT_EQ(franka_robot_state_msg.accelerometer_top[sensor].vector.x, top_readings[sensor][0]);
    ASSERT_EQ(franka_robot_state_msg.accelerometer_top[sensor].vector.y, top_readings[sensor][1]);
    ASSERT_EQ(franka_robot_state_msg.accelerometer_top[sensor].vector.z, top_readings[sensor][2]);
    ASSERT_EQ(franka_robot_state_msg.accelerometer_bottom[sensor].vector.x,
              bottom_readings[sensor][0]);
    ASSERT_EQ(franka_robot_state_msg.accelerometer_bottom[sensor].vector.y,
              bottom_readings[sensor][1]);
    ASSERT_EQ(franka_robot_state_msg.accelerometer_bottom[sensor].vector.z,
              bottom_readings[sensor][2]);
  }
}

void FrankaRobotStateMultiArmTest::TearDown() {
  franka_state_friend.reset(nullptr);
}

void FrankaRobotStateMultiArmTest::SetUp() {
  franka_state_friend = std::make_unique<FrankaRobotStateMultiArmTestFriend>(
      robot_name + "/" + franka_state_interface_name);

  robot_state.q = joint_angles;
  robot_state.q_d = desired_joint_angles;
  robot_state_box.set(robot_state);

  hardware_interface::StateInterface franka_hw_state{
      robot_name, franka_state_interface_name, reinterpret_cast<double*>(&robot_state_box_ptr)};
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;

  temp_state_interfaces.reserve(size);
  temp_state_interfaces.emplace_back(franka_hw_state);

  ASSERT_TRUE(franka_state_friend->assign_loaned_state_interfaces(temp_state_interfaces));
  franka_state_friend->initialize_robot_state_msg(franka_robot_state_msg);
  ASSERT_TRUE(franka_state_friend->get_values_as_message(franka_robot_state_msg));
}

// The robot_description holds fourteen revolute joints, but libfranka reports seven values for
// the one arm this component is bound to. Sizing the arrays from the description made them
// fourteen long and filled the extra entries by reading past the end of franka::RobotState's
// seven-element arrays.
TEST_F(FrankaRobotStateMultiArmTest, givenMultiArmRobotDescription_thenJointArraysAreArmLocal) {
  const std::vector<std::string> expected_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3",
                                                   "fr3_joint4", "fr3_joint5", "fr3_joint6",
                                                   "fr3_joint7"};

  ASSERT_EQ(franka_robot_state_msg.measured_joint_state.name, expected_names);
  ASSERT_EQ(franka_robot_state_msg.desired_joint_state.name, expected_names);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_motor_state.name, expected_names);
  ASSERT_EQ(franka_robot_state_msg.tau_ext_hat_filtered.name, expected_names);

  ASSERT_EQ(franka_robot_state_msg.measured_joint_state.position.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_state.velocity.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_state.effort.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.desired_joint_state.position.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.desired_joint_state.velocity.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.desired_joint_state.effort.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_motor_state.position.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_motor_state.velocity.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.measured_joint_motor_state.effort.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.tau_ext_hat_filtered.position.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.tau_ext_hat_filtered.velocity.size(), 7u);
  ASSERT_EQ(franka_robot_state_msg.tau_ext_hat_filtered.effort.size(), 7u);
}

// Each entry has to carry the quantity its own field promises. Before the fix, entry seven of
// measured_joint_state.position held q_d[0] rather than a measured position.
TEST_F(FrankaRobotStateMultiArmTest, givenMultiArmRobotDescription_thenJointValuesAreNotMixed) {
  ASSERT_THAT(joint_angles,
              ::testing::ElementsAreArray(franka_robot_state_msg.measured_joint_state.position));
  ASSERT_THAT(desired_joint_angles,
              ::testing::ElementsAreArray(franka_robot_state_msg.desired_joint_state.position));
}

// A robot_description that does not describe this arm is a configuration error, not something to
// publish around. initialize_robot_state_msg() already rejects a missing accelerometer frame the
// same way.
TEST(FrankaRobotStateStandaloneTest, givenDescriptionWithoutThisArm_thenConstructionThrows) {
  EXPECT_THROW(franka_semantic_components::FrankaRobotState(
                   "no_such_arm/robot_state", read_robot_description("franka_semantic_components")),
               std::runtime_error);
}
