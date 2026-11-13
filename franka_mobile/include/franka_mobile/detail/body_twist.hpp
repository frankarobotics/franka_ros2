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

#include <cmath>

#include <Eigen/Dense>

namespace franka_mobile {
namespace detail {

/**
 * @brief Yaw of a planar orientation [rad], in (-pi, pi].
 */
inline double yawOf(const Eigen::Quaterniond& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/**
 * @brief Body-frame twist (vx, vy, wz) that moves a planar base from pose 0 to pose 1 in dt.
 *
 * nav_msgs/Odometry expresses its twist in child_frame_id (the base), so the odom-frame
 * displacement is rotated into the base's axes at the mid-heading:
 * \f$ v_b = R(\psi_0 + \Delta\psi/2)^\top \, \Delta p / \Delta t \f$, exact up to a factor
 * \f$ \mathrm{sinc}(\Delta\psi/2) \f$ for a constant twist (1 - 2e-5 at 1 rad/s over 20 ms).
 *
 * @param p0, q0  Pose at the start, in the odometry frame.
 * @param p1, q1  Pose at the end, in the odometry frame.
 * @param dt      Elapsed time [s]; must be positive.
 */
inline Eigen::Vector3d bodyTwist(const Eigen::Vector3d& p0,
                                 const Eigen::Quaterniond& q0,
                                 const Eigen::Vector3d& p1,
                                 const Eigen::Quaterniond& q1,
                                 double dt) {
  const double yaw0 = yawOf(q0);
  const double dyaw = std::remainder(yawOf(q1) - yaw0, 2.0 * M_PI);
  const Eigen::Vector2d displacement =
      Eigen::Rotation2Dd(-(yaw0 + 0.5 * dyaw)) * (p1 - p0).head<2>();
  return {displacement.x() / dt, displacement.y() / dt, dyaw / dt};
}

}  // namespace detail
}  // namespace franka_mobile
