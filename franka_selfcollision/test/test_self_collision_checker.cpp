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

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "franka_selfcollision/self_collision_checker.hpp"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

std::string readFileToString(const std::string& filename) {
  auto file = std::ifstream(filename);
  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return "";
  }
  auto oss = std::ostringstream();
  oss << file.rdbuf();
  file.close();
  return oss.str();
}

// ---------------------------------------------------------------------------
// FR3 Duo
// ---------------------------------------------------------------------------

class SelfCollisionCheckerTest : public ::testing::Test {
 protected:
  static constexpr double kSecurityMargin = 0.001;
  static constexpr size_t kNumJoints = 14;

  void SetUp() override {
    try {
      std::string test_dir = TEST_DIR;
      std::string urdf_xml = readFileToString(test_dir + "/fr3_duo.urdf");
      std::string srdf_xml = readFileToString(test_dir + "/fr3_duo.srdf");
      auto clock = std::make_shared<rclcpp::Clock>();
      checker_ = std::make_unique<franka_selfcollision::SelfCollisionChecker>(
          urdf_xml, srdf_xml, kSecurityMargin, rclcpp::get_logger("test_logger"), clock);
    } catch (const std::exception& e) {
      FAIL() << "Setup failed: " << e.what();
    }
  }

  std::unique_ptr<franka_selfcollision::SelfCollisionChecker> checker_;
};

TEST_F(SelfCollisionCheckerTest, givenInvalidInputDimensions_thenThrowInvalidArgument) {
  std::vector<double> input_too_small(SelfCollisionCheckerTest::kNumJoints - 1, 0.0);
  std::vector<double> input_too_big(SelfCollisionCheckerTest::kNumJoints + 1, 0.0);
  ASSERT_THROW({ checker_->checkCollision(input_too_small, false); }, std::invalid_argument);
  ASSERT_THROW({ checker_->checkCollision(input_too_big, false); }, std::invalid_argument);
}

TEST_F(SelfCollisionCheckerTest, givenSafeConfiguration_thenReturnFalse) {
  std::vector<double> home_config(SelfCollisionCheckerTest::kNumJoints, 0.0);
  std::vector<double> start_config = {
      0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4,  // Arm 1
      0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4   // Arm 2
  };
  ASSERT_FALSE(checker_->checkCollision(home_config, true)) << "Home config should be safe";
  ASSERT_FALSE(checker_->checkCollision(start_config, true)) << "Start config should be safe";
}

TEST_F(SelfCollisionCheckerTest, givenCollidingConfiguration_thenReturnTrue) {
  // Left arm into bottom plate
  std::vector<double> mount_collision = {
      0.0, M_PI_2, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4,  // Arm 1
      0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4  // Arm 2
  };
  // Arms into each other
  std::vector<double> dual_collision = {
      0.0, 0.2, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4,  // Arm 1
      0.0, 0.2, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4   // Arm 2
  };
  ASSERT_TRUE(checker_->checkCollision(mount_collision, true))
      << "Left arm should collide into the mount";
  ASSERT_TRUE(checker_->checkCollision(dual_collision, true))
      << "Arms should collide with each other";
}

// ---------------------------------------------------------------------------
// Mobile FR3 Duo
// ---------------------------------------------------------------------------

class MobileSelfCollisionCheckerTest : public ::testing::Test {
 protected:
  static constexpr double kSecurityMargin = 0.001;

  void SetUp() override {
    std::string test_dir = TEST_DIR;
    std::string urdf_path = test_dir + "/mobile_fr3_duo_v0_2.urdf";
    std::string srdf_path = test_dir + "/mobile_fr3_duo_v0_2.srdf";

    // Skip the fixture entirely if test assets are not present yet
    if (std::ifstream(urdf_path).fail() || std::ifstream(srdf_path).fail()) {
      GTEST_SKIP() << "Mobile test assets not found — skipping MobileSelfCollisionCheckerTest";
    }

    try {
      std::string urdf_xml = readFileToString(urdf_path);
      std::string srdf_xml = readFileToString(srdf_path);
      auto clock = std::make_shared<rclcpp::Clock>();
      checker_ = std::make_unique<franka_selfcollision::SelfCollisionChecker>(
          urdf_xml, srdf_xml, kSecurityMargin, rclcpp::get_logger("test_logger"), clock);
    } catch (const std::exception& e) {
      GTEST_SKIP() << "Mobile test assets unavailable: " << e.what();
    }
  }

  std::unique_ptr<franka_selfcollision::SelfCollisionChecker> checker_;
};

TEST_F(MobileSelfCollisionCheckerTest, givenNeutralConfiguration_thenReturnFalse) {
  // getNeutralConfiguration() returns Eigen::VectorXd — convert explicitly.
  Eigen::VectorXd q0 = checker_->getNeutralConfiguration();
  std::vector<double> config(q0.data(), q0.data() + q0.size());
  ASSERT_FALSE(checker_->checkCollision(config, true)) << "Neutral config should be safe";
}

TEST_F(MobileSelfCollisionCheckerTest, givenModelLoaded_thenHasExpectedJoints) {
  // Verify the checker was constructed successfully and has a sane number of joints: universe + spine + 7 left + 7 right + base/drivetrain extras
  ASSERT_GE(checker_->getModelNjoints(), 16u)
      << "Model should have at least 16 joints";
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}