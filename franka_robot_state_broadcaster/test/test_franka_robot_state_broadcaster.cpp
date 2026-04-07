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

#include <ros2_control_test_assets/descriptions.hpp>
#include "controller_interface/controller_interface.hpp"
#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

class MockFrankaRobotState : public franka_semantic_components::FrankaRobotState {
 public:
  MockFrankaRobotState(const std::string& name, const std::string& robot_description)
      : FrankaRobotState(name, robot_description){};

  MOCK_METHOD(void, initialize_robot_state_msg, (franka_msgs::msg::FrankaRobotState&), (override));
  MOCK_METHOD(bool, get_values_as_message, (franka_msgs::msg::FrankaRobotState&), (override));
};

using namespace franka_robot_state_broadcaster;
class TestFrankaRobotStateBroadcaster : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<MockFrankaRobotState> franka_robot_state =
        std::make_unique<MockFrankaRobotState>("mock_franka_robot_state",
                                               ros2_control_test_assets::minimal_robot_urdf);
    franka_robot_state_raw_ = franka_robot_state.get();  // Save raw pointer for mocking

    broadcaster_ = std::make_unique<FrankaRobotStateBroadcaster>(std::move(franka_robot_state));
    broadcaster_->init("test_broadcaster", "",
                       rclcpp::NodeOptions()
                           .allow_undeclared_parameters(true)
                           .automatically_declare_parameters_from_overrides(true));
    broadcaster_->get_node()->set_parameter(
        {"robot_description", ros2_control_test_assets::minimal_robot_urdf});
  }
  std::unique_ptr<FrankaRobotStateBroadcaster> broadcaster_;
  MockFrankaRobotState* franka_robot_state_raw_;
};

TEST_F(TestFrankaRobotStateBroadcaster, test_init_return_success) {
  EXPECT_EQ(broadcaster_->on_init(), controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_configure_return_success) {
  // Avoid the "Uninteresting mock function call - returning default value"
  EXPECT_CALL(*franka_robot_state_raw_, initialize_robot_state_msg(::testing::_)).Times(3);
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_activate_return_success) {
  // Avoid the "Uninteresting mock function call - returning default value"
  EXPECT_CALL(*franka_robot_state_raw_, initialize_robot_state_msg(::testing::_)).Times(3);
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_deactivate_return_success) {
  // Avoid the "Uninteresting mock function call - returning default value"
  EXPECT_CALL(*franka_robot_state_raw_, initialize_robot_state_msg(::testing::_)).Times(3);
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_update_without_franka_state_interface_returns_error) {
  EXPECT_CALL(*franka_robot_state_raw_, initialize_robot_state_msg(::testing::_)).Times(3);

  // Simulate failure: no valid state interface
  EXPECT_CALL(*franka_robot_state_raw_, get_values_as_message(::testing::_))
      .Times(1)
      .WillOnce(::testing::Return(false));

  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  rclcpp::Time time(0.0, 0.0);
  rclcpp::Duration period(0, 0);

  EXPECT_EQ(broadcaster_->update(time, period), controller_interface::return_type::ERROR);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_update_with_franka_state_returns_success) {
  EXPECT_CALL(*franka_robot_state_raw_, initialize_robot_state_msg(::testing::_)).Times(3);

  // Simulate success: valid state interface
  EXPECT_CALL(*franka_robot_state_raw_, get_values_as_message(::testing::_))
      .Times(1)
      .WillOnce(::testing::Return(true));

  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  rclcpp::Time time(0.0, 0.0);
  rclcpp::Duration period(0, 0);

  EXPECT_EQ(broadcaster_->update(time, period), controller_interface::return_type::OK);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
