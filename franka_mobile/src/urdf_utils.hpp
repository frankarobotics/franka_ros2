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

#include <Eigen/Eigen>
#include <string>

namespace franka_mobile {

struct SE3 {
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
};

// Retrieves the SE3 of the target frame in the reference frame from the robot description xml
// string
SE3 getSe3FromDescription(const std::string& robot_description,
                          const std::string& reference_frame,
                          const std::string& target_frame);

// Retrieves the radius of the cylinder tag in the selected link_name from
// the robot description xml string
double getWheelRadiusFromDescription(const std::string& robot_description,
                                     const std::string& link_name);

}  // namespace franka_mobile