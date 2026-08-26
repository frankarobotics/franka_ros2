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

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <ros2_control_test_assets/descriptions.hpp>
#include "controller_interface/controller_interface.hpp"
#include "franka/robot_state.h"
#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
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

// FrankaRobotState resolves its own arm's seven joints out of the description, so the mock needs
// a description that names them. ros2_control_test_assets' minimal robot does not.
inline std::string singleArmUrdf(const std::string& arm_name) {
  std::stringstream urdf;
  urdf << "<?xml version=\"1.0\"?><robot name=\"" << arm_name << "\">";
  urdf << "<link name=\"" << arm_name << "_link0\"/>";
  for (size_t joint = 1; joint <= 7; ++joint) {
    urdf << "<link name=\"" << arm_name << "_link" << joint << "\"/>"
         << "<joint name=\"" << arm_name << "_joint" << joint << "\" type=\"revolute\">"
         << "<parent link=\"" << arm_name << "_link" << joint - 1 << "\"/>"
         << "<child link=\"" << arm_name << "_link" << joint << "\"/>"
         << "<axis xyz=\"0 0 1\"/>"
         << "<limit lower=\"-2.0\" upper=\"2.0\" effort=\"87.0\" velocity=\"2.0\"/></joint>";
  }
  urdf << "</robot>";
  return urdf.str();
}

using namespace franka_robot_state_broadcaster;
class TestFrankaRobotStateBroadcaster : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<MockFrankaRobotState> franka_robot_state =
        std::make_unique<MockFrankaRobotState>("mock_franka_robot_state",
                                               singleArmUrdf("mock_franka_robot_state"));
    franka_robot_state_raw_ = franka_robot_state.get();  // Save raw pointer for mocking

    broadcaster_ = std::make_unique<FrankaRobotStateBroadcaster>(std::move(franka_robot_state));
    broadcaster_->init("test_broadcaster", ros2_control_test_assets::minimal_robot_urdf, 30, "",
                       rclcpp::NodeOptions()
                           .allow_undeclared_parameters(true)
                           .automatically_declare_parameters_from_overrides(true));
    broadcaster_->get_node()->set_parameter(
        {"robot_description", ros2_control_test_assets::minimal_robot_urdf});

    // The broadcaster claims the robot state interface on activation, so offer one that
    // carries the address of a state box the way the hardware does.
    std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
    loaned_state_interfaces.emplace_back(robot_state_interface_);
    broadcaster_->assign_interfaces({}, std::move(loaned_state_interfaces));
  }

  realtime_tools::RealtimeThreadSafeBox<franka::RobotState> robot_state_box_;
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState>* robot_state_box_ptr_ =
      &robot_state_box_;
  hardware_interface::StateInterface robot_state_interface_{
      "mock_franka_robot_state", "robot_state",
      reinterpret_cast<double*>(&robot_state_box_ptr_)};
  // AsyncBuffer has three slots; each is initialized once in on_configure().
  void expectMessageInitialized() {
    EXPECT_CALL(*franka_robot_state_raw_, initialize_robot_state_msg(::testing::_)).Times(3);
  }

  std::unique_ptr<FrankaRobotStateBroadcaster> broadcaster_;
  MockFrankaRobotState* franka_robot_state_raw_;
};

TEST_F(TestFrankaRobotStateBroadcaster, test_init_return_success) {
  EXPECT_EQ(broadcaster_->on_init(), controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_configure_return_success) {
  expectMessageInitialized();
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_activate_return_success) {
  expectMessageInitialized();
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, an_unclaimable_state_interface_fails_activation) {
  expectMessageInitialized();
  broadcaster_->release_interfaces();

  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
}

TEST_F(TestFrankaRobotStateBroadcaster, an_unresolvable_state_buffer_fails_activation) {
  expectMessageInitialized();
  robot_state_box_ptr_ = nullptr;

  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_deactivate_return_success) {
  expectMessageInitialized();
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_update_without_franka_state_interface_returns_error) {
  expectMessageInitialized();

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
  expectMessageInitialized();

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
