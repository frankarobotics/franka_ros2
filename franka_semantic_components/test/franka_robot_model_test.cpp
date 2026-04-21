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

#include "franka_robot_model_test.hpp"
#include <exception>

#include <memory>
#include <string>
#include <vector>

void FrankaRobotModelTest::TearDown() {
  franka_robot_model_friend.reset(nullptr);
}

void FrankaRobotModelTest::SetUp() {
  full_interface_names.reserve(size);
  full_interface_names.emplace_back(robot_name + "/" + franka_model_interface_name);
  full_interface_names.emplace_back(robot_name + "/" + franka_state_interface_name);
  franka_robot_model_friend =
      std::make_unique<FrankaRobotModelTestFriend>(robot_name + "/" + franka_model_interface_name,
                                                   robot_name + "/" + franka_state_interface_name);
}

TEST_F(FrankaRobotModelTest, given_franka_semantic_model_initialized_when_get_coriolis_expect_one) {
  std::vector<std::string> interface_names = franka_robot_model_friend->get_state_interface_names();
  hardware_interface::StateInterface franka_hw_model{robot_name, franka_model_interface_name,
                                                     reinterpret_cast<double*>(&model_address)};
  hardware_interface::StateInterface franka_hw_state{
      robot_name, franka_state_interface_name, reinterpret_cast<double*>(&robot_state_buffer_ptr)};
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(size);
  temp_state_interfaces.emplace_back(franka_hw_state);
  temp_state_interfaces.emplace_back(franka_hw_model);

  franka_robot_model_friend->assign_loaned_state_interfaces(temp_state_interfaces);

  std::array<double, 7> expected_coriolis{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  EXPECT_CALL(mock_model, coriolis(testing::_)).WillOnce(testing::Return(expected_coriolis));
  ASSERT_EQ(franka_robot_model_friend->getCoriolisForceVector(), expected_coriolis);
  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotModelTest, validate_state_names_and_size) {
  std::vector<std::string> interface_names = franka_robot_model_friend->get_state_interface_names();
  hardware_interface::StateInterface franka_hw_model{robot_name, franka_model_interface_name,
                                                     reinterpret_cast<double*>(&model_address)};
  hardware_interface::StateInterface franka_hw_state{
      robot_name, franka_state_interface_name, reinterpret_cast<double*>(&robot_state_buffer_ptr)};
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(size);
  temp_state_interfaces.emplace_back(franka_hw_state);
  temp_state_interfaces.emplace_back(franka_hw_model);

  franka_robot_model_friend->assign_loaned_state_interfaces(temp_state_interfaces);

  franka_robot_model_friend->initialize();
  ASSERT_EQ(franka_robot_model_friend->interface_names_.size(), size);
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.capacity(), size);

  ASSERT_TRUE(std::equal(franka_robot_model_friend->interface_names_.begin(),
                         franka_robot_model_friend->interface_names_.end(),
                         full_interface_names.begin(), full_interface_names.end()));
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), size);
  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotModelTest, given_franka_semantic_model_initialized_when_get_gravity_expect_one) {
  std::vector<std::string> interface_names = franka_robot_model_friend->get_state_interface_names();
  hardware_interface::StateInterface franka_hw_model{robot_name, franka_model_interface_name,
                                                     reinterpret_cast<double*>(&model_address)};
  hardware_interface::StateInterface franka_hw_state{
      robot_name, franka_state_interface_name, reinterpret_cast<double*>(&robot_state_buffer_ptr)};

  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(size);
  temp_state_interfaces.emplace_back(franka_hw_state);
  temp_state_interfaces.emplace_back(franka_hw_model);

  franka_robot_model_friend->assign_loaned_state_interfaces(temp_state_interfaces);

  std::array<double, 7> expected_gravity{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  EXPECT_CALL(mock_model, gravity(testing::_)).WillOnce(testing::Return(expected_gravity));

  ASSERT_EQ(franka_robot_model_friend->getGravityForceVector(), expected_gravity);

  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotModelTest, given_franka_semantic_model_initialized_when_get_pose_expect_one) {
  std::vector<std::string> interface_names = franka_robot_model_friend->get_state_interface_names();
  hardware_interface::StateInterface franka_hw_model{robot_name, franka_model_interface_name,
                                                     reinterpret_cast<double*>(&model_address)};
  hardware_interface::StateInterface franka_hw_state{
      robot_name, franka_state_interface_name, reinterpret_cast<double*>(&robot_state_buffer_ptr)};

  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(size);
  temp_state_interfaces.emplace_back(franka_hw_state);
  temp_state_interfaces.emplace_back(franka_hw_model);

  franka_robot_model_friend->assign_loaned_state_interfaces(temp_state_interfaces);

  std::array<double, 16> expected_pose{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                       1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  EXPECT_CALL(mock_model, pose(testing::_, testing::_)).WillOnce(testing::Return(expected_pose));
  ASSERT_EQ(franka_robot_model_friend->getPoseMatrix(franka::Frame::kEndEffector), expected_pose);

  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotModelTest, given_franka_semantic_model_initialized_when_get_mass_expect_correct) {
  std::vector<std::string> interface_names = franka_robot_model_friend->get_state_interface_names();
  hardware_interface::StateInterface franka_hw_model{robot_name, franka_model_interface_name,
                                                     reinterpret_cast<double*>(&model_address)};
  hardware_interface::StateInterface franka_hw_state{
      robot_name, franka_state_interface_name, reinterpret_cast<double*>(&robot_state_buffer_ptr)};

  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(size);
  temp_state_interfaces.emplace_back(franka_hw_state);
  temp_state_interfaces.emplace_back(franka_hw_model);
  franka_robot_model_friend->assign_loaned_state_interfaces(temp_state_interfaces);

  std::array<double, 49> expected_mass;
  expected_mass.fill(1.0);

  EXPECT_CALL(mock_model, mass(testing::_)).WillOnce(testing::Return(expected_mass));
  ASSERT_EQ(franka_robot_model_friend->getMassMatrix(), expected_mass);
  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotModelTest, given_franka_semantic_model_not_initialized_expect_exception) {
  EXPECT_THROW(franka_robot_model_friend->getMassMatrix(), std::runtime_error);
  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotModelTest,
       given_franka_semantic_model_not_initialized_when_get_gravity_called_expect_exception) {
  EXPECT_THROW(franka_robot_model_friend->getGravityForceVector(), std::runtime_error);
  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotModelTest,
       given_franka_semantic_model_not_initialized_when_get_pose_called_expect_exception) {
  EXPECT_THROW(franka_robot_model_friend->getPoseMatrix(franka::Frame::kEndEffector),
               std::runtime_error);
  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotModelTest,
       given_franka_semantic_model_not_initialized_when_get_body_jacobian_called_expect_exception) {
  EXPECT_THROW(franka_robot_model_friend->getBodyJacobian(franka::Frame::kEndEffector),
               std::runtime_error);
  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotModelTest,
       given_franka_semantic_model_not_initialized_when_get_zero_jacobian_called_expect_exception) {
  EXPECT_THROW(franka_robot_model_friend->getZeroJacobian(franka::Frame::kEndEffector),
               std::runtime_error);
  franka_robot_model_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_robot_model_friend->state_interfaces_.size(), 0u);
}
