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

#include "franka_cartesian_pose_interface_test.hpp"

#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <vector>

void FrankaCartesianPoseTest::TearDown() {
  franka_cartesian_command_friend.reset(nullptr);
}

void FrankaCartesianPoseTest::constructFrankaCartesianPoseInterface(bool elbow_activate) {
  franka_cartesian_command_friend = std::make_unique<FrankaCartesianPoseTestFriend>(elbow_activate);
  setUpHWCommandInterfaces(elbow_activate);
  setUpHWStateInterfaces(elbow_activate);
}

void FrankaCartesianPoseTest::setUpHWStateInterfaces(bool elbow_activate) {
  temp_state_interfaces.clear();

  for (auto i = 0U; i < hw_cartesian_pose_command_.size(); i++) {
    pose_state_interfaces_container.push_back(std::make_shared<hardware_interface::StateInterface>(
        hardware_interface::StateInterface{std::to_string(i), cartesian_pose_state_interface_name_,
                                           &cartesian_pose_state_.at(i)}));
  }
  for (auto& pose_state_interface : pose_state_interfaces_container) {
    temp_state_interfaces.emplace_back(*pose_state_interface.get());
  }
  if (elbow_activate) {
    for (auto i = 0U; i < hw_elbow_command_names_.size(); i++) {
      elbow_state_interfaces_container.push_back(
          std::make_shared<hardware_interface::StateInterface>(hardware_interface::StateInterface{
              hw_elbow_command_names_[i], elbow_state_interface_name_, &elbow_state_.at(i)}));
    }
    for (auto& elbow_state_interface : elbow_state_interfaces_container) {
      temp_state_interfaces.emplace_back(*elbow_state_interface.get());
    }
  }

  franka_cartesian_command_friend->assign_loaned_state_interfaces(temp_state_interfaces);
}

void FrankaCartesianPoseTest::setUpHWCommandInterfaces(bool elbow_activate) {
  temp_command_interfaces.clear();

  for (auto i = 0U; i < hw_cartesian_pose_command_.size(); i++) {
    pose_command_interfaces_container.push_back(
        std::make_shared<hardware_interface::CommandInterface>(hardware_interface::CommandInterface{
            std::to_string(i), cartesian_pose_command_interface_name_,
            &hw_cartesian_pose_command_.at(i)}));
  }
  for (auto& pose_command_interface : pose_command_interfaces_container) {
    temp_command_interfaces.emplace_back(*pose_command_interface.get());
  }
  if (elbow_activate) {
    for (auto i = 0U; i < hw_elbow_command_names_.size(); i++) {
      elbow_command_interfaces_container.push_back(
          std::make_shared<hardware_interface::CommandInterface>(
              hardware_interface::CommandInterface{hw_elbow_command_names_[i],
                                                   elbow_command_interface_name_,
                                                   &hw_elbow_command_.at(i)}));
    }
    for (auto& elbow_command_interface : elbow_command_interfaces_container) {
      temp_command_interfaces.emplace_back(*elbow_command_interface.get());
    }
  }
  franka_cartesian_command_friend->assign_loaned_command_interfaces(temp_command_interfaces);
}

TEST_F(FrankaCartesianPoseTest,
       given_correct_interfaces_set_carte_with_elbow_command_expect_successful) {
  constructFrankaCartesianPoseInterface(true);
  std::array<double, 16> new_hw_cartesian_pose_command{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                                       0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0};
  std::array<double, 2> new_elbow_command{0.0, 2.0};

  auto success =
      franka_cartesian_command_friend->setCommand(new_hw_cartesian_pose_command, new_elbow_command);

  ASSERT_TRUE(success);

  ASSERT_EQ(hw_cartesian_pose_command_, new_hw_cartesian_pose_command);
  ASSERT_EQ(hw_elbow_command_, new_elbow_command);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest,
       given_correct_interfaces_when_get_elbow_command_values_called_expect_successful) {
  constructFrankaCartesianPoseInterface(true);
  std::array<double, 16> new_hw_cartesian_pose_command{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                                       0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0};
  std::array<double, 2> new_elbow_command{0.0, 2.0};

  auto success_set_command =
      franka_cartesian_command_friend->setCommand(new_hw_cartesian_pose_command, new_elbow_command);

  ASSERT_TRUE(success_set_command);

  ASSERT_EQ(hw_cartesian_pose_command_, new_hw_cartesian_pose_command);
  ASSERT_EQ(hw_elbow_command_, new_elbow_command);

  std::array<double, 2> elbow_configuration;
  elbow_configuration = franka_cartesian_command_friend->getCommandedElbowConfiguration();

  ASSERT_EQ(new_elbow_command, elbow_configuration);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest,
       given_elbow_is_not_activated_when_elbow_command_get_value_is_called_expect_throw) {
  constructFrankaCartesianPoseInterface(false);
  std::array<double, 16> new_hw_cartesian_pose_command{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                                       0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0};

  auto success_set_command =
      franka_cartesian_command_friend->setCommand(new_hw_cartesian_pose_command);

  ASSERT_TRUE(success_set_command);

  ASSERT_EQ(hw_cartesian_pose_command_, new_hw_cartesian_pose_command);

  ASSERT_THROW(franka_cartesian_command_friend->getCommandedElbowConfiguration(),
               std::runtime_error);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest,
       given_elbow_and_cartesian_pose_claimed_when_set_pose_command_without_elbow_expect_failure) {
  constructFrankaCartesianPoseInterface(true);
  std::array<double, 16> new_hw_cartesian_pose_command{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                                       0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0};

  auto success = franka_cartesian_command_friend->setCommand(new_hw_cartesian_pose_command);

  ASSERT_FALSE(success);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(
    FrankaCartesianPoseTest,
    given_only_cartesian_velocity_claimed_without_elbow_when_set_velocity_command_cartesian_vel_expect_successful) {
  constructFrankaCartesianPoseInterface(false);
  std::array<double, 16> new_hw_cartesian_pose_command{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                                       0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0};
  std::array<double, 2> default_zero_elbow_command{0.0, 0.0};

  auto success = franka_cartesian_command_friend->setCommand(new_hw_cartesian_pose_command,
                                                             default_zero_elbow_command);

  ASSERT_FALSE(success);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest,
       given_incorrect_command_interfaces_set_velocity_expect_unsuccesful) {
  franka_cartesian_command_friend = std::make_unique<FrankaCartesianPoseTestFriend>(true);
  hardware_interface::CommandInterface dummy_command_interface{"dummy", "dummy_cartesian_pose",
                                                               &hw_elbow_command_.at(0)};
  std::array<double, 16> new_hw_cartesian_pose_command{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                                       0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0};

  temp_command_interfaces.emplace_back(dummy_command_interface);
  franka_cartesian_command_friend->assign_loaned_command_interfaces(temp_command_interfaces);

  auto success = franka_cartesian_command_friend->setCommand(new_hw_cartesian_pose_command);
  ASSERT_FALSE(success);

  franka_cartesian_command_friend->release_interfaces();
  temp_command_interfaces.clear();
}

TEST_F(
    FrankaCartesianPoseTest,
    given_correct_interfaces_when_set_command_called_with_quaternion_and_translation_expect_the_correct_pose) {
  constructFrankaCartesianPoseInterface(false);
  Eigen::Quaterniond quaternion(0.0, 1.0, 2.0, 3.0);
  Eigen::Vector3d translation(4.0, 5.0, 6.0);
  std::array<double, 16> expected_command{-25, 4, 6, 0, 4, -19, 12, 0, 6, 12, -9, 0, 4, 5, 6, 1};

  auto success = franka_cartesian_command_friend->setCommand(quaternion, translation);

  ASSERT_TRUE(success);

  for (auto i = 0U; i < expected_command.size(); i++) {
    ASSERT_NEAR(expected_command.at(i), hw_cartesian_pose_command_.at(i), 1e-6);
  }
  ASSERT_EQ(hw_cartesian_pose_command_, expected_command);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(
    FrankaCartesianPoseTest,
    given_correct_interfaces_when_set_command_called_with_quaternion_and_translation_and_elbow_expect_the_correct_pose) {
  constructFrankaCartesianPoseInterface(true);
  Eigen::Quaterniond quaternion(0.0, 1.0, 2.0, 3.0);
  Eigen::Vector3d translation(4.0, 5.0, 6.0);
  std::array<double, 2> elbow{0.0, 1.0};

  std::array<double, 16> expected_command{-25, 4, 6, 0, 4, -19, 12, 0, 6, 12, -9, 0, 4, 5, 6, 1};

  auto success = franka_cartesian_command_friend->setCommand(quaternion, translation, elbow);

  ASSERT_TRUE(success);

  for (auto i = 0U; i < expected_command.size(); i++) {
    ASSERT_NEAR(expected_command.at(i), hw_cartesian_pose_command_.at(i), 1e-6);
  }

  ASSERT_EQ(hw_elbow_command_, elbow);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest,
       given_correct_interfaces_when_get_commanded_pose_called_expect_the_correct_pose) {
  constructFrankaCartesianPoseInterface(false);
  std::array<double, 16> received_pose;

  received_pose = franka_cartesian_command_friend->getCommandedPoseMatrix();
  ASSERT_EQ(hw_cartesian_pose_command_, received_pose);

  auto [received_quaternion, received_translation] =
      franka_cartesian_command_friend->getCommandedOrientationAndTranslation();

  Eigen::Matrix4d pose =
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(hw_cartesian_pose_command_.data());

  Eigen::Quaterniond quaternion = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
  Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

  ASSERT_EQ(received_quaternion, quaternion);
  ASSERT_EQ(received_translation, translation);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest,
       given_correct_interfaces_when_get_elbow_commanded_called_expect_the_correct_elbow) {
  constructFrankaCartesianPoseInterface(true);
  std::array<double, 2> elbow_configuration;

  elbow_configuration = franka_cartesian_command_friend->getCommandedElbowConfiguration();

  ASSERT_EQ(hw_elbow_command_, elbow_configuration);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest, given_correct_interfaces_when_elbow_request_expect_correct_state) {
  constructFrankaCartesianPoseInterface(true);
  std::array<double, 2> elbow_configuration;

  elbow_configuration = franka_cartesian_command_friend->getCurrentElbowConfiguration();

  ASSERT_EQ(elbow_state_, elbow_configuration);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest, given_elbow_not_activated_when_elbow_requested_expect_throw) {
  constructFrankaCartesianPoseInterface(false);

  ASSERT_THROW(franka_cartesian_command_friend->getCurrentElbowConfiguration(), std::runtime_error);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest,
       given_correct_interfaces_when_current_orientation_request_expect_correct_pose) {
  constructFrankaCartesianPoseInterface(true);
  auto [received_orientation, received_translation] =
      franka_cartesian_command_friend->getCurrentOrientationAndTranslation();

  Eigen::Matrix4d pose =
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(cartesian_pose_state_.data());

  Eigen::Quaterniond expected_orientation = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
  Eigen::Vector3d expected_translation = pose.block<3, 1>(0, 3);

  ASSERT_EQ(received_orientation, expected_orientation);
  ASSERT_EQ(received_translation, expected_translation);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest, given_correct_interfaces_when_pose_requested_expect_correct_pose) {
  constructFrankaCartesianPoseInterface(true);
  auto received_pose = franka_cartesian_command_friend->getCurrentPoseMatrix();

  ASSERT_EQ(received_pose, cartesian_pose_state_);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(
    FrankaCartesianPoseTest,
    given_valid_state_interfaces_when_reading_pose_realtime_safe_then_orientation_and_translation_match) {
  constructFrankaCartesianPoseInterface(false);
  Eigen::Quaterniond received_orientation(0.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d received_translation(99.0, 99.0, 99.0);

  const bool success = franka_cartesian_command_friend->getCurrentOrientationAndTranslation(
      received_orientation, received_translation);

  ASSERT_TRUE(success);

  const Eigen::Matrix4d pose =
      Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(cartesian_pose_state_.data());
  const Eigen::Quaterniond expected_orientation(pose.block<3, 3>(0, 0));
  const Eigen::Vector3d expected_translation = pose.block<3, 1>(0, 3);
  ASSERT_EQ(received_orientation, expected_orientation);
  ASSERT_EQ(received_translation, expected_translation);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(
    FrankaCartesianPoseTest,
    given_non_finite_state_value_when_reading_pose_realtime_safe_then_read_fails_and_outputs_untouched) {
  constructFrankaCartesianPoseInterface(false);
  cartesian_pose_state_[3] = std::numeric_limits<double>::quiet_NaN();
  Eigen::Quaterniond received_orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d received_translation(1.0, 2.0, 3.0);

  const bool success = franka_cartesian_command_friend->getCurrentOrientationAndTranslation(
      received_orientation, received_translation);

  ASSERT_FALSE(success);
  ASSERT_EQ(received_orientation, Eigen::Quaterniond::Identity());
  ASSERT_EQ(received_translation, Eigen::Vector3d(1.0, 2.0, 3.0));

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianPoseTest,
       given_unassigned_state_interfaces_when_reading_pose_realtime_safe_then_read_fails) {
  franka_cartesian_command_friend = std::make_unique<FrankaCartesianPoseTestFriend>(false);
  Eigen::Quaterniond received_orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d received_translation = Eigen::Vector3d::Zero();

  ASSERT_FALSE(franka_cartesian_command_friend->getCurrentOrientationAndTranslation(
      received_orientation, received_translation));

  franka_cartesian_command_friend->release_interfaces();
}
