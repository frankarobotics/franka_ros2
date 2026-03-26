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

#include <algorithm>
#include <stdexcept>

#include "mobile_fr3_duo_joint_trajectory_controller/mapping.hpp"

namespace mobile_fr3_duo_joint_trajectory_controller {

bool contains_substring(const std::string& str, const std::string& substring) {
  return str.find(substring) != std::string::npos;
}

size_t find_matching_element(const std::vector<std::string>& vec, const std::string& match) {
  auto it = std::find(vec.begin(), vec.end(), match);
  if (it != vec.end()) {
    return std::distance(vec.begin(), it);
  }

  std::string msg = "No match found for '" + match + "' in vector.";
  throw std::runtime_error(msg);
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

std::array<size_t, 7> get_arm_joint_map(std::vector<std::string> joint_names, std::string side) {
  std::array<size_t, 7> map;

  for (size_t i = 0; i < map.size(); ++i) {
    std::string joint_string = "joint" + std::to_string(i + 1);
    try {
      map[i] = find_element_with_substrings(joint_names, {side, joint_string});
    } catch (const std::runtime_error& e) {
      std::string msg = "Could not find joint name matching {'" + side + "', '" + joint_string +
                        "'}. "
                        " Error : " +
                        e.what();
      throw;
    }
  }
  return map;
}

std::array<size_t, 3> get_mobile_base_joint_map(std::vector<std::string> joint_names,
                                                std::array<std::string, 3> expected_joint_names) {
  std::array<size_t, 3> map;

  for (size_t i = 0; i < 3; ++i) {
    std::string joint_name = expected_joint_names[i];
    map[i] = find_matching_element(joint_names, joint_name);
  }
  return map;
}

/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \p t2
 * indices. If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated
 * mapping vector is <tt>"{2, 1}"</tt>. return empty vector if \p t1 is not a subset of \p t2.
 */
template <class T>
inline std::vector<size_t> mapping(const T& t1, const T& t2) {
  // t1 must be a subset of t2
  if (t1.size() > t2.size()) {
    return std::vector<size_t>();
  }

  std::vector<size_t> mapping_vector(t1.size());  // Return value
  for (auto t1_it = t1.begin(); t1_it != t1.end(); ++t1_it) {
    auto t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {
      return std::vector<size_t>();
    } else {
      const size_t t1_dist = static_cast<size_t>(std::distance(t1.begin(), t1_it));
      const size_t t2_dist = static_cast<size_t>(std::distance(t2.begin(), t2_it));
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

void sort_to_local_joint_order(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg,
    const std::vector<std::string> joint_names) {
  // rearrange all points in the trajectory message based on mapping
  std::vector<size_t> mapping_vector = mapping(trajectory_msg->joint_names, joint_names);
  auto remap = [&joint_names](const std::vector<double>& to_remap,
                              const std::vector<size_t>& mapping) -> std::vector<double> {
    if (to_remap.empty()) {
      return to_remap;
    }
    if (to_remap.size() != mapping.size()) {
      return to_remap;
    }
    static std::vector<double> output(joint_names.size(), 0.0);
    // Only resize if necessary since it's an expensive operation
    if (output.size() != mapping.size()) {
      output.resize(mapping.size(), 0.0);
    }
    for (size_t index = 0; index < mapping.size(); ++index) {
      auto map_index = mapping[index];
      output[map_index] = to_remap[index];
    }
    return output;
  };

  for (size_t index = 0; index < trajectory_msg->points.size(); ++index) {
    trajectory_msg->points[index].positions =
        remap(trajectory_msg->points[index].positions, mapping_vector);

    trajectory_msg->points[index].velocities =
        remap(trajectory_msg->points[index].velocities, mapping_vector);

    // TODO (wink_ma): don't really care about accelerations and effort - delete?
    trajectory_msg->points[index].accelerations =
        remap(trajectory_msg->points[index].accelerations, mapping_vector);

    trajectory_msg->points[index].effort =
        remap(trajectory_msg->points[index].effort, mapping_vector);
  }
}

}  // namespace mobile_fr3_duo_joint_trajectory_controller