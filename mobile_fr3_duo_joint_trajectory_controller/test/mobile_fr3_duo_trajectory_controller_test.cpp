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

#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "mobile_fr3_duo_joint_trajectory_controller/mapping.hpp"

class MobileFR3DuoTrajectoryControllerTest : public ::testing::Test {
  void SetUp() override {}
};

TEST_F(MobileFR3DuoTrajectoryControllerTest,
       givenJointNamesWhenSubstringContainedThenCorrectIndicesReturned) {
  const std::vector<std::string> joint_names = {
      "left_joint2", "left_joint1", "bottom_fr3_joint2", "bottom_fr3_joint1", "bottom_fr3_joint5",
  };
  const std::vector<std::string> substrings = {"bottom", "joint1"};

  size_t i = mobile_fr3_duo_joint_trajectory_controller::find_element_with_substrings(joint_names,
                                                                                      substrings);
  size_t expected_index = 3;
  ASSERT_EQ(i, expected_index);
}

TEST_F(MobileFR3DuoTrajectoryControllerTest,
       givenJointNamesWhenNoElementsContainSubstringsThenThrowsError) {
  const std::vector<std::string> joint_names = {
      "left_joint2", "left_joint1", "bottom_fr3_joint2", "bottom_fr3_joint1", "bottom_fr3_joint5",
  };
  const std::vector<std::string> substrings = {"top", "joint1"};

  ASSERT_THROW(
      {
        mobile_fr3_duo_joint_trajectory_controller::find_element_with_substrings(joint_names,
                                                                                 substrings);
      },
      std::runtime_error);
}

TEST_F(MobileFR3DuoTrajectoryControllerTest,
       givenJointNamesWhenStringMatchesThenReturnsCorrectIndex) {
  const std::vector<std::string> joint_names = {
      "left_joint2", "planar_thetario", "planar_theta", "bottom_fr3_joint1", "bottom_fr3_joint5",
  };
  const std::string joint_name = "planar_theta";

  size_t i =
      mobile_fr3_duo_joint_trajectory_controller::find_matching_element(joint_names, joint_name);

  size_t expected_index = 2;
  ASSERT_EQ(i, expected_index);
}

TEST_F(MobileFR3DuoTrajectoryControllerTest,
       givenJointNamesWhenStringMatchesNoElementThenThrowsError) {
  const std::vector<std::string> joint_names = {
      "left_joint2", "planar_yfff", "planar_theta", "bottom_fr3_joint1", "bottom_fr3_joint5",
  };
  const std::string joint_name = "planar_y";

  ASSERT_THROW(
      {
        mobile_fr3_duo_joint_trajectory_controller::find_matching_element(joint_names, joint_name);
      },
      std::runtime_error);
}

TEST_F(MobileFR3DuoTrajectoryControllerTest, givenJointNamesWhenGetStateMapThenAsExpected) {
  const std::vector<std::string> joint_names = {
      "bottom_fr3_joint1", "left_joint1", "bottom_fr3_joint2", "bottom_fr3_joint5", "left_joint2"};
  const std::vector<std::string> arm_prefixes = {"left"};
  std::map<std::pair<size_t, size_t>, size_t> expected_joint_map = {{{0, 0}, 1}, {{0, 1}, 4}};
  std::map<std::pair<size_t, size_t>, size_t> joint_map =
      mobile_fr3_duo_joint_trajectory_controller::get_joint_state_map(joint_names, arm_prefixes);

  ASSERT_EQ(joint_map, expected_joint_map);
};

TEST_F(MobileFR3DuoTrajectoryControllerTest, givenJointNamesWhenGetArmMapThenAsExpected) {
  const std::vector<std::string> joint_names = {
      "bottom_fr3_joint1", "left_joint1",       "left_joint3",
      "bottom_fr3_joint5", "left_fr3_joint2",   "left_joint4",
      "left_joint5",       "left_fr3v2_joint6", "left_joint7"};
  std::array<size_t, 7> expected_arm_map = {1, 4, 2, 5, 6, 7, 8};

  std::array<size_t, 7> arm_map =
      mobile_fr3_duo_joint_trajectory_controller::get_arm_joint_map(joint_names, "left");

  ASSERT_EQ(arm_map, expected_arm_map);
};

TEST_F(MobileFR3DuoTrajectoryControllerTest, givenMissingJoint6WhenGetArmMapThenThrowsError) {
  const std::vector<std::string> joint_names = {
      "bottom_fr3_joint1", "left_joint1", "left_joint3", "bottom_fr3_joint5",
      "left_fr3_joint2",   "left_joint4", "left_joint5", "left_joint7"};

  ASSERT_THROW(
      { mobile_fr3_duo_joint_trajectory_controller::get_arm_joint_map(joint_names, "left"); },
      std::runtime_error);
};

TEST_F(MobileFR3DuoTrajectoryControllerTest, givenJointNamesWhenGetMobileBaseMapThenAsExpected) {
  const std::vector<std::string> all_joint_names = {
      "bottom_fr3_joint1", "left_joint1",  "left_joint3", "bottom_fr3_joint5", "planar_y",
      "left_joint4",       "planar_theta", "left_joint6", "planar_x"};

  const std::array<std::string, 3> base_joint_names = {"planar_x", "planar_theta", "planar_y"};
  std::array<size_t, 3> expected_mobile_base_map = {8, 6, 4};

  std::array<size_t, 3> mobile_base_map =
      mobile_fr3_duo_joint_trajectory_controller::get_mobile_base_joint_map(all_joint_names,
                                                                            base_joint_names);

  ASSERT_EQ(mobile_base_map, expected_mobile_base_map);
};

TEST_F(MobileFR3DuoTrajectoryControllerTest,
       givenMisspelledPlanarThetaWhenGetMobileBaseMapThenThrowsError) {
  const std::vector<std::string> all_joint_names = {
      "bottom_fr3_joint1", "left_joint1",     "left_joint3", "bottom_fr3_joint5", "planar_y",
      "left_joint4",       "planar_thetasss", "left_joint6", "planar_x"};

  const std::array<std::string, 3> base_joint_names = {"planar_x", "planar_theta", "planar_y"};

  ASSERT_THROW(
      {
        mobile_fr3_duo_joint_trajectory_controller::get_mobile_base_joint_map(all_joint_names,
                                                                              base_joint_names);
      },
      std::runtime_error);
};

TEST_F(MobileFR3DuoTrajectoryControllerTest, givenJointNamesWhenSortThenExpectedOrder) {
  std_msgs::msg::Header header;
  auto trajectory = std::make_shared<trajectory_msgs::msg::JointTrajectory>();

  trajectory->header = header;
  trajectory->joint_names = {"joint3", "joint1", "joint4"};

  trajectory_msgs::msg::JointTrajectoryPoint p0;
  p0.positions = {3.0, 1.0, 4.0};
  p0.velocities = {3.0, 1.0, 4.0};
  p0.accelerations = {3.0, 1.0, 4.0};
  p0.effort = {3.0, 1.0, 4.0};
  p0.time_from_start.sec = 0;
  p0.time_from_start.nanosec = 1;

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.positions = {3.1, 1.1, 4.1};
  p1.velocities = {3.1, 1.1, 4.1};
  p1.accelerations = {3.1, 1.1, 4.1};
  p1.effort = {3.1, 1.1, 4.1};
  p1.time_from_start.sec = 1;
  p1.time_from_start.nanosec = 1;

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.positions = {3.2, 1.2, 4.2};
  p2.velocities = {3.2, 1.2, 4.2};
  p2.accelerations = {3.2, 1.2, 4.2};
  p2.effort = {3.2, 1.2, 4.2};
  p2.time_from_start.sec = 2;
  p2.time_from_start.nanosec = 1;

  trajectory->points = {p0, p1, p2};

  const std::vector<std::string> local_joint_names = {"joint1", "joint3", "joint4"};

  mobile_fr3_duo_joint_trajectory_controller::sort_to_local_joint_order(trajectory,
                                                                        local_joint_names);

  auto expected_trajectory = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  expected_trajectory->header = header;
  expected_trajectory->joint_names = local_joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint e0;
  e0.positions = {1.0, 3.0, 4.0};
  e0.velocities = {1.0, 3.0, 4.0};
  e0.accelerations = {1.0, 3.0, 4.0};
  e0.effort = {1.0, 3.0, 4.0};
  e0.time_from_start.sec = 0;
  e0.time_from_start.nanosec = 1;

  trajectory_msgs::msg::JointTrajectoryPoint e1;
  e1.positions = {1.1, 3.1, 4.1};
  e1.velocities = {1.1, 3.1, 4.1};
  e1.accelerations = {1.1, 3.1, 4.1};
  e1.effort = {1.1, 3.1, 4.1};
  e1.time_from_start.sec = 1;
  e1.time_from_start.nanosec = 1;

  trajectory_msgs::msg::JointTrajectoryPoint e2;
  e2.positions = {1.2, 3.2, 4.2};
  e2.velocities = {1.2, 3.2, 4.2};
  e2.accelerations = {1.2, 3.2, 4.2};
  e2.effort = {1.2, 3.2, 4.2};
  e2.time_from_start.sec = 2;
  e2.time_from_start.nanosec = 1;

  expected_trajectory->points = {e0, e1, e2};

  ASSERT_EQ(trajectory->points.size(), expected_trajectory->points.size());

  for (size_t i = 0; i < trajectory->points.size(); ++i) {
    ASSERT_EQ(trajectory->points[i].positions, expected_trajectory->points[i].positions);
    ASSERT_EQ(trajectory->points[i].velocities, expected_trajectory->points[i].velocities);
    ASSERT_EQ(trajectory->points[i].accelerations, expected_trajectory->points[i].accelerations);
    ASSERT_EQ(trajectory->points[i].effort, expected_trajectory->points[i].effort);
    ASSERT_EQ(trajectory->points[i].time_from_start.sec,
              expected_trajectory->points[i].time_from_start.sec);
    ASSERT_EQ(trajectory->points[i].time_from_start.nanosec,
              expected_trajectory->points[i].time_from_start.nanosec);
  }
};
