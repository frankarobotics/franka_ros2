#pragma once

#include <map>
#include <string>
#include <vector>

namespace mobile_fr3_duo_joint_trajectory_controller {

bool contains_substring(const std::string& str, const std::string& substring);

size_t find_element_with_substrings(const std::vector<std::string>& vec,
                                    const std::vector<std::string>& substrings);

std::map<std::pair<size_t, size_t>, size_t> get_joint_state_map(
    const std::vector<std::string>& joint_names,
    const std::vector<std::string>& arm_prefixes);

}  // namespace mobile_fr3_duo_joint_trajectory_controller