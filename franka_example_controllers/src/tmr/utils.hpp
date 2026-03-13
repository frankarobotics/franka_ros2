#pragma once

#include <Eigen/Eigen>
#include <string>

namespace franka_example_controllers {

struct SE3 {
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
};

SE3 get_se3_from_description(const std::string& robot_description,
                             const std::string& reference_frame,
                             const std::string& target_frame);

double get_wheel_radius_from_description(const std::string& robot_description,
                                         const std::string& link_name);

}  // namespace franka_example_controllers