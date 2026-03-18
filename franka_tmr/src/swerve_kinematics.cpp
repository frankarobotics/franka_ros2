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

#include <Eigen/Dense>
#include <franka_tmr/swerve_kinematics.hpp>

namespace franka_tmr {

SwerveKinematics::SwerveKinematics(const std::array<Eigen::Vector2d, 2>& wheel_positions,
                                   double wheel_radius)
    : wheel_positions_(wheel_positions), wheel_radius_(wheel_radius) {
  if (!std::isfinite(wheel_radius) || fabs(wheel_radius) < 1e-3 || wheel_radius < 0) {
    throw std::invalid_argument("Wheel radius must be positive");
  }

  for (const auto& pos : wheel_positions) {
    if (pos.isZero(1e-3)) {
      throw std::invalid_argument("Wheel position cannot be zero");
    }
  }
}

bool SwerveKinematics::forward(const std::array<double, 2>& steering_angles,
                               const std::array<double, 2>& wheel_speeds,
                               double& vx,
                               double& vy,
                               double& wz) const {
  // Each wheel velocity vector in world frame
  // v_wheel = [v*cos(θ), v*sin(θ)]
  // Body velocity constraint per wheel:
  //   v*cos(θ) = vx - ry*ω
  //   v*sin(θ) = vy + rx*ω
  //
  // Build A (4x3) and b (4x1) for least squares A*[vx,vy,ω]^T = b

  // A rows: [1, 0, -ry_i] and [0, 1, rx_i] per wheel
  // b entries: v_i*cos(θ_i) and v_i*sin(θ_i)

  Eigen::Matrix<double, 4, 3> A;
  Eigen::Vector4d b;

  for (int i = 0; i < 2; ++i) {
    const double wheel_speed = wheel_speeds[i] * wheel_radius_;
    const double cos_s = std::cos(steering_angles[i]);
    const double sin_s = std::sin(steering_angles[i]);

    A.row(2 * i) << 1.0, 0.0, -wheel_positions_[i].y();
    A.row(2 * i + 1) << 0.0, 1.0, wheel_positions_[i].x();

    b(2 * i) = wheel_speed * cos_s;
    b(2 * i + 1) = wheel_speed * sin_s;
  }

  const Eigen::Vector3d result = A.colPivHouseholderQr().solve(b);

  if (!result.allFinite()) {
    return false;
  }

  vx = result(0);
  vy = result(1);
  wz = result(2);

  return true;
}

bool SwerveKinematics::inverse(double vx,
                               double vy,
                               double wz,
                               std::array<double, 2>& steering_angles,
                               std::array<double, 2>& wheel_speeds) {
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
    return false;
  }

  static constexpr double eps = 1e-3;

  for (int i = 0; i < 2; ++i) {
    const double rx = wheel_positions_[i].x();
    const double ry = wheel_positions_[i].y();

    // Wheel velocity vector in base frame
    const double vx_wheel = vx - wz * ry;
    const double vy_wheel = vy + wz * rx;

    double speed = std::sqrt(vx_wheel * vx_wheel + vy_wheel * vy_wheel) / wheel_radius_;
    double angle = speed > eps ? std::atan2(vy_wheel, vx_wheel) : steering_angles_[i];

    const double diff = angle - steering_angles_[i];
    if (std::fabs(diff) > M_PI / 2.0) {
      angle = speed > eps ? std::atan2(-vy_wheel, -vx_wheel) : steering_angles_[i];
      speed = -speed;
    }

    steering_angles_[i] = angle;
    wheel_speeds_[i] = speed;

    steering_angles[i] = angle;
    wheel_speeds[i] = speed;
  }

  return true;
}

}  // namespace franka_tmr