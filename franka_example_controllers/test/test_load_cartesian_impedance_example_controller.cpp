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

#include <gtest/gtest.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadCartesianImpedanceExampleController, load_controller) {
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  rclcpp::Logger logger = rclcpp::get_logger("load_controller");

  controller_manager::ControllerManager cm(std::make_unique<hardware_interface::ResourceManager>(
                                               ros2_control_test_assets::minimal_robot_urdf,
                                               std::make_shared<rclcpp::Clock>(), logger),
                                           executor, "test_controller_manager");

  auto response =
      cm.load_controller("test_cartesian_impedance_example_controller",
                         "franka_example_controllers/CartesianImpedanceExampleController");

  ASSERT_NE(response, nullptr);

  rclcpp::shutdown();
}
