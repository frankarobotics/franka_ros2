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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <realtime_tools/realtime_buffer.hpp>
#include "franka/model.h"
#include "franka/robot_state.h"
#include "franka_semantic_components/franka_robot_model.hpp"
#include "gmock/gmock.h"

class FrankaRobotModelTest;

class MockModel : public franka_hardware::Model {
 public:
  MOCK_METHOD((std::array<double, 7>), gravity, (const franka::RobotState&), (const, override));
  MOCK_METHOD((std::array<double, 7>), coriolis, (const franka::RobotState&), (const, override));
  MOCK_METHOD((std::array<double, 16>),
              pose,
              (franka::Frame, const franka::RobotState&),
              (const, override));
  MOCK_METHOD((std::array<double, 42>),
              bodyJacobian,
              (franka::Frame, const franka::RobotState&),
              (const, override));
  MOCK_METHOD((std::array<double, 42>),
              zeroJacobian,
              (franka::Frame, const franka::RobotState&),
              (const, override));
  MOCK_METHOD((std::array<double, 49>), mass, (const franka::RobotState&), (const, override));
};

class FrankaRobotModelTestFriend : public franka_semantic_components::FrankaRobotModel {
  FRIEND_TEST(FrankaRobotModelTest, validate_state_names_and_size);
  FRIEND_TEST(FrankaRobotModelTest,
              given_franka_semantic_model_initialized_when_get_coriolis_expect_one);
  FRIEND_TEST(FrankaRobotModelTest,
              given_franka_semantic_model_initialized_when_get_gravity_expect_one);
  FRIEND_TEST(FrankaRobotModelTest,
              given_franka_semantic_model_initialized_when_get_pose_expect_one);
  FRIEND_TEST(FrankaRobotModelTest,
              given_franka_semantic_model_initialized_when_get_mass_expect_correct);
  FRIEND_TEST(FrankaRobotModelTest, given_franka_semantic_model_not_initialized_expect_exception);
  FRIEND_TEST(FrankaRobotModelTest,
              given_franka_semantic_model_not_initialized_when_get_gravity_called_expect_exception);
  FRIEND_TEST(FrankaRobotModelTest,
              given_franka_semantic_model_not_initialized_when_get_pose_called_expect_exception);
  FRIEND_TEST(FrankaRobotModelTest,
              given_franka_semantic_model_not_initialized_when_get_pose_called_expect_exception);
  FRIEND_TEST(
      FrankaRobotModelTest,
      given_franka_semantic_model_not_initialized_when_get_body_jacobian_called_expect_exception);
  FRIEND_TEST(
      FrankaRobotModelTest,
      given_franka_semantic_model_not_initialized_when_get_zero_jacobian_called_expect_exception);

 public:
  FrankaRobotModelTestFriend(const std::string& model_interface_name,
                             const std::string& model_state_name)
      : franka_semantic_components::FrankaRobotModel(model_interface_name, model_state_name) {}
  FrankaRobotModelTestFriend() = delete;

  virtual ~FrankaRobotModelTestFriend() = default;
};

class FrankaRobotModelTest : public ::testing::Test {
 public:
  void SetUp();
  void TearDown();

 protected:
  const size_t size = 2;
  const std::string robot_name = "fr3";
  const std::string franka_model_interface_name = "robot_model";
  const std::string franka_state_interface_name = "robot_state";

  MockModel mock_model;
  MockModel* model_address = &mock_model;

  franka::RobotState robot_state;
  realtime_tools::RealtimeBuffer<franka::RobotState> robot_state_buffer;
  realtime_tools::RealtimeBuffer<franka::RobotState>* robot_state_buffer_ptr = &robot_state_buffer;

  std::unique_ptr<FrankaRobotModelTestFriend> franka_robot_model_friend;

  std::vector<std::string> full_interface_names;
};
