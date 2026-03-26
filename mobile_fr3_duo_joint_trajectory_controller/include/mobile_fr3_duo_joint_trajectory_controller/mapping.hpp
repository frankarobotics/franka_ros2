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

#include <map>
#include <memory>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace mobile_fr3_duo_joint_trajectory_controller {

size_t find_matching_element(const std::vector<std::string>& vec, const std::string& match);

size_t find_element_with_substrings(const std::vector<std::string>& vec,
                                    const std::vector<std::string>& substrings);

std::map<std::pair<size_t, size_t>, size_t> get_joint_state_map(
    const std::vector<std::string>& joint_names,
    const std::vector<std::string>& arm_prefixes);

std::array<size_t, 7> get_arm_joint_map(std::vector<std::string> joint_names, std::string side);

std::array<size_t, 3> get_mobile_base_joint_map(std::vector<std::string> joint_names,
                                                std::array<std::string, 3> expected_joint_names);

void sort_to_local_joint_order(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg,
    const std::vector<std::string> joint_names);

}  // namespace mobile_fr3_duo_joint_trajectory_controller