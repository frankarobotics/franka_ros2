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

#pragma once

/// @file mapping.hpp
/// @brief Utility functions for mapping joint names between the trajectory
///        message ordering and the internal controller ordering.

#include <map>
#include <memory>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace mobile_fr3_duo_trajectory_controller {

/// @brief Return the index of @p match in @p vec.
/// @throws std::runtime_error if no exact match is found.
size_t findMatchingElement(const std::vector<std::string>& vec, const std::string& match);

/// @brief Return the index of the first element in @p vec that contains every
///        substring in @p substrings.
/// @throws std::runtime_error if no element matches.
size_t findElementWithSubstrings(const std::vector<std::string>& vec,
                                 const std::vector<std::string>& substrings);

/// @brief Build a map from (arm_index, local_joint_index) to the flat state
///        interface index, corresponding to the to their @p joint_names @p arm_prefixes.
std::map<std::pair<size_t, size_t>, size_t> getJointStateMap(
    const std::vector<std::string>& joint_names,
    const std::vector<std::string>& arm_prefixes);

/// @brief Return a 7-element index array mapping local joint indices to their
///        positions in @p joint_names for the given @p side ("left" or "right").
std::array<size_t, 7> getArmJointMap(std::vector<std::string> joint_names, std::string side);

/// @brief Return a 3-element index array mapping mobile base joints to their
///        positions in @p joint_names.
std::array<size_t, 3> getMobileBaseJointMap(std::vector<std::string> joint_names,
                                            std::array<std::string, 3> expected_joint_names);

/// @brief Re-order the positions/velocities/accelerations of every waypoint in
///        @p trajectory_msg so they match @p joint_names ordering.
void sortToLocalJointOrder(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg,
                           const std::vector<std::string> joint_names);

}  // namespace mobile_fr3_duo_trajectory_controller