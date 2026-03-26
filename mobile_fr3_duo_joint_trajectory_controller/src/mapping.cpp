#include <algorithm>
#include <stdexcept>

#include "mobile_fr3_duo_joint_trajectory_controller/mapping.hpp"

namespace mobile_fr3_duo_joint_trajectory_controller {

bool contains_substring(const std::string& str, const std::string& substring) {
  return str.find(substring) != std::string::npos;
}

size_t find_element_with_substrings(const std::vector<std::string>& vec,
                                    const std::vector<std::string>& substrings) {
  auto it = std::find_if(vec.begin(), vec.end(), [&substrings](const std::string& str) {
    for (const auto& substring : substrings) {
      if (!contains_substring(str, substring)) {
        return false;
      }
    }
    return true;
  });

  if (it != vec.end()) {
    return std::distance(vec.begin(), it);
  }

  throw std::runtime_error("Substrings not found in vector.");
}

std::map<std::pair<size_t, size_t>, size_t> get_joint_state_map(
    const std::vector<std::string>& joint_names,
    const std::vector<std::string>& arm_prefixes) {
  std::map<std::pair<size_t, size_t>, size_t> map;

  // TODO (wink_ma): avoid magic numbers
  for (size_t arm_index = 0; arm_index < arm_prefixes.size(); ++arm_index) {
    for (size_t joint_index = 0; joint_index < 7; ++joint_index) {
      const std::string joint_string = "joint" + std::to_string(joint_index + 1);
      const std::string side = arm_prefixes[arm_index];
      try {
        const size_t state_index = find_element_with_substrings(joint_names, {side, joint_string});
        map[{arm_index, joint_index}] = state_index;
      } catch (const std::runtime_error& e) {
        // it's ok for the map if the substrings were not part of the vector
      }
    }
  }

  return map;
}

}  // namespace mobile_fr3_duo_joint_trajectory_controller