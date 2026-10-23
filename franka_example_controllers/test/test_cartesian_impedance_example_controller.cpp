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

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "franka_example_controllers/fr3/cartesian_impedance_example_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CartesianImpedanceExampleControllerTest : public ::testing::Test {
 public:
  static void SetUpTestSuite();
  static void TearDownTestSuite();

  void SetUp();
  void TearDown();

  void SetUpController(const std::string& robot_type, const std::string& arm_prefix);

 protected:
  std::unique_ptr<franka_example_controllers::CartesianImpedanceExampleController> controller_;
};

void CartesianImpedanceExampleControllerTest::SetUpTestSuite() {
  rclcpp::init(0, nullptr);
}

void CartesianImpedanceExampleControllerTest::TearDownTestSuite() {
  rclcpp::shutdown();
}

void CartesianImpedanceExampleControllerTest::SetUp() {
  controller_ = std::make_unique<franka_example_controllers::CartesianImpedanceExampleController>();
}

void CartesianImpedanceExampleControllerTest::TearDown() {
  controller_.reset(nullptr);
}

void CartesianImpedanceExampleControllerTest::SetUpController(const std::string& robot_type,
                                                              const std::string& arm_prefix) {
  const auto result = controller_->init("test_cartesian_impedance_example",
                                        ros2_control_test_assets::minimal_robot_urdf, 30, "",
                                        rclcpp::NodeOptions()
                                            .allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  controller_->get_node()->set_parameter({"robot_type", robot_type});
  controller_->get_node()->set_parameter({"arm_prefix", arm_prefix});
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(CartesianImpedanceExampleControllerTest, interface_names_follow_robot_type) {
  SetUpController("fr3v2", "");

  std::vector<std::string> expected_command_interfaces;
  std::vector<std::string> expected_state_interfaces;
  for (int i = 1; i <= 7; ++i) {
    expected_command_interfaces.push_back("fr3v2_joint" + std::to_string(i) + "/effort");
    expected_state_interfaces.push_back("fr3v2_joint" + std::to_string(i) + "/position");
    expected_state_interfaces.push_back("fr3v2_joint" + std::to_string(i) + "/velocity");
  }

  const auto command_config = controller_->command_interface_configuration();
  EXPECT_EQ(command_config.names, expected_command_interfaces);

  const auto state_config = controller_->state_interface_configuration();
  ASSERT_GE(state_config.names.size(), expected_state_interfaces.size());
  for (size_t i = 0; i < expected_state_interfaces.size(); ++i) {
    EXPECT_EQ(state_config.names[i], expected_state_interfaces[i]);
  }
}

TEST_F(CartesianImpedanceExampleControllerTest, interface_names_follow_arm_prefix) {
  SetUpController("fr3v2", "left");

  const auto command_config = controller_->command_interface_configuration();
  ASSERT_EQ(command_config.names.size(), 7U);
  for (int i = 1; i <= 7; ++i) {
    EXPECT_EQ(command_config.names[i - 1], "left_fr3v2_joint" + std::to_string(i) + "/effort");
  }
}

TEST_F(CartesianImpedanceExampleControllerTest, stale_arm_id_warns_and_keeps_robot_type) {
  // controller_manager NodeOptions: no auto-declare; arm_id arrives as an override, so
  // only the on_init declaration makes it visible to on_configure. Deliberately not via
  // SetUpController(), whose auto-declare options would declare it behind the back of
  // the code under test. No set_parameter for arm_id: the override is the whole point.
  const auto result = controller_->init(
      "test_cartesian_impedance_example", ros2_control_test_assets::minimal_robot_urdf, 30, "",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("arm_id", "fr3_legacy")}));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  controller_->get_node()->set_parameter({"robot_type", "fr3"});
  controller_->get_node()->set_parameter({"arm_prefix", ""});
  testing::internal::CaptureStderr();
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  const std::string log_output = testing::internal::GetCapturedStderr();
  EXPECT_NE(log_output.find("set 'robot_type' instead"), std::string::npos);

  // No fallback: the interface names still follow robot_type, not arm_id.
  const auto command_config = controller_->command_interface_configuration();
  ASSERT_EQ(command_config.names.size(), 7U);
  for (int i = 1; i <= 7; ++i) {
    EXPECT_EQ(command_config.names[i - 1], "fr3_joint" + std::to_string(i) + "/effort");
  }
}
