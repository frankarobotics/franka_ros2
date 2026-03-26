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

TEST_F(MobileFR3DuoTrajectoryControllerTest, givenJointNamesWhenGetStateMapThenAsExpected) {
  const std::vector<std::string> joint_names = {
      "bottom_fr3_joint1", "left_joint1", "bottom_fr3_joint2", "bottom_fr3_joint5", "left_joint2"};
  const std::vector<std::string> arm_prefixes = {"left"};
  std::map<std::pair<size_t, size_t>, size_t> expected_joint_map = {{{0, 0}, 1}, {{0, 1}, 4}};
  std::map<std::pair<size_t, size_t>, size_t> joint_map =
      mobile_fr3_duo_joint_trajectory_controller::get_joint_state_map(joint_names, arm_prefixes);

  ASSERT_EQ(joint_map, expected_joint_map);
};