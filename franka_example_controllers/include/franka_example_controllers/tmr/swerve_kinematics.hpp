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

namespace franka_example_controllers {

class SwerveKinematics {
 public:
  explicit SwerveKinematics(const std::array<Eigen::Vector2d, 2>& wheel_positions, double wheel_radius);

  bool forward(const std::array<double, 2>& steering_angles,
               const std::array<double, 2>& wheel_speeds,
               double& vx,
               double& vy,
               double& wz) const;

  bool inverse(double vx,
               double vy,
               double wz,
               std::array<double, 2>& steering_angles,
               std::array<double, 2>& wheel_speeds);

 private:
  std::array<double, 2> steering_angles_, wheel_speeds_;
  std::array<Eigen::Vector2d, 2> wheel_positions_;
  double wheel_radius_;
};

}  // namespace franka_example_controllers