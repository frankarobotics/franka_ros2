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

#include "swerve_ik.hpp"

namespace franka_example_controllers {

void computeSwerveIK(double vx,
                     double vy,
                     double wz,
                     const Eigen::Vector4d& wheel_positions,
                     double wheel_radius,
                     Eigen::Vector4d& steering_angles,
                     Eigen::Vector4d& wheel_velocities,
                     std::array<WheelCommand, 2>& commands) {
  constexpr int kNumberOfWheels = 2;
  Eigen::Array2d x = wheel_positions.head<2>();
  Eigen::Array2d y = wheel_positions.tail<2>();

  Eigen::Array2d vx_wheel = vx - wz * y;
  Eigen::Array2d vy_wheel = vy + wz * x;

  Eigen::Array2d speed = ((vx_wheel.square() + vy_wheel.square()).sqrt()) / wheel_radius;
  Eigen::Array2d angle;
  for (int i = 0; i < kNumberOfWheels; ++i) {
    angle(i) = std::atan2(vy_wheel(i), vx_wheel(i));
  }

  for (int i = 0; i < kNumberOfWheels; ++i) {
    double prev_angle = steering_angles(i);
    double new_angle = angle(i);
    double new_speed = speed(i);

    double angle_diff = std::fabs(new_angle - prev_angle);
    if (angle_diff > M_PI / 2.0) {
      commands[i].steering_angle = prev_angle;
      commands[i].wheel_velocity = -new_speed;
    } else {
      commands[i].steering_angle = new_angle;
      commands[i].wheel_velocity = new_speed;
    }
    steering_angles(i) = commands[i].steering_angle;
    wheel_velocities(i) = commands[i].wheel_velocity;
  }
}

}  // namespace franka_example_controllers