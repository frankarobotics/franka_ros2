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

#include <Eigen/Core>

namespace franka_mobile {

/**
 * @brief Kinematics solver for a two-wheel swerve drive platform.
 *
 * A swerve drive (also known as holonomic drive) allows each wheel to independently
 * control both its steering angle and rotational speed, enabling omnidirectional
 * motion — the platform can translate in any direction while simultaneously rotating.
 *
 * This class provides both forward and inverse kinematics for a two-wheel configuration:
 *
 * - **Forward kinematics**: given the current wheel states (steering angles and speeds),
 *   compute the resulting body-frame velocity (vx, vy, wz).
 *
 * - **Inverse kinematics**: given a desired body-frame velocity, compute the required
 *   steering angles and wheel speeds for each wheel.
 *
 * Wheel positions are defined in the robot base frame, with the X axis pointing forward
 * and Y axis pointing left. Velocities are expressed in the robot body frame.
 *
 * @note This class is not thread-safe. The inverse kinematics stores the last computed
 *       steering angles internally to resolve the heading ambiguity (a wheel pointing at
 *       angle theta is equivalent to one pointing at theta+pi with reversed speed).
 *
 * Example usage:
 * @code
 * std::array<Eigen::Vector2d, 2> wheel_positions = {
 *     Eigen::Vector2d{0.2, 0.15},   // front wheel, 20cm forward, 15cm left
 *     Eigen::Vector2d{-0.2, -0.15}  // rear wheel
 * };
 * SwerveKinematics kinematics(wheel_positions, 0.05);  // 5cm wheel radius
 *
 * // Inverse: command 0.5 m/s forward, no lateral, no rotation
 * std::array<double, 2> steering_angles, wheel_speeds;
 * kinematics.inverse(0.5, 0.0, 0.0, steering_angles, wheel_speeds);
 *
 * // Forward: recover body velocity from wheel states
 * double vx, vy, wz;
 * kinematics.forward(steering_angles, wheel_speeds, vx, vy, wz);
 * @endcode
 */
class SwerveKinematics {
 public:
  explicit SwerveKinematics(const std::array<Eigen::Vector2d, 2>& wheel_positions,
                            double wheel_radius);

  /**
   * @brief Compute body-frame velocity from wheel states (forward kinematics).
   *
   * Averages the wheel velocities
   *
   * @param[in]  steering_angles  Current steering angle of each wheel [rad],
   *                              measured from the robot X axis (forward).
   * @param[in]  wheel_speeds     Current rotational speed of each wheel [rad/s].
   *                              Positive is forward along the wheel's heading direction.
   * @param[out] vx               Resulting body-frame linear velocity along X [m/s].
   * @param[out] vy               Resulting body-frame linear velocity along Y [m/s].
   * @param[out] wz               Resulting body-frame angular velocity around Z [rad/s].
   *
   * @return true on success, false if the solve failed
   */
  bool forwardKinematics(const std::array<double, 2>& steering_angles,
                         const std::array<double, 2>& wheel_speeds,
                         double& vx,
                         double& vy,
                         double& wz) const;

  /**
   * @brief Compute body-frame velocity from wheel states (forward kinematics).
   *
   * Solves the overdetermined system of wheel velocity constraints in a least-squares
   * sense to recover the best-fit rigid body velocity.
   *
   * @param[in]  steering_angles  Current steering angle of each wheel [rad],
   *                              measured from the robot X axis (forward).
   * @param[in]  wheel_speeds     Current rotational speed of each wheel [rad/s].
   *                              Positive is forward along the wheel's heading direction.
   * @param[out] vx               Resulting body-frame linear velocity along X [m/s].
   * @param[out] vy               Resulting body-frame linear velocity along Y [m/s].
   * @param[out] wz               Resulting body-frame angular velocity around Z [rad/s].
   *
   * @return true on success, false if the solve failed (e.g. singular configuration).
   */
  bool forwardKinematicsQr(const std::array<double, 2>& steering_angles,
                           const std::array<double, 2>& wheel_speeds,
                           double& vx,
                           double& vy,
                           double& wz) const;

  /**
   * @brief Compute wheel commands from a desired body-frame velocity (inverse kinematics).
   *
   * Each wheel's heading is chosen to minimize steering travel from the previously
   * commanded angle, resolving the π-ambiguity by potentially reversing wheel speed.
   *
   * @param[in]  vx               Desired linear velocity along X in body frame [m/s].
   * @param[in]  vy               Desired linear velocity along Y in body frame [m/s].
   * @param[in]  wz               Desired angular velocity around Z in body frame [rad/s].
   * @param[out] steering_angles  Required steering angle for each wheel [rad].
   * @param[out] wheel_speeds     Required rotational speed for each wheel [rad/s].
   *
   * @return true on success, false if no valid solution exists
   *         (e.g. kinematically infeasible command or zero-radius turn singularity).
   */
  bool inverseKinematics(double vx,
                         double vy,
                         double wz,
                         std::array<double, 2>& steering_angles,
                         std::array<double, 2>& wheel_speeds);

 private:
  std::array<double, 2> steering_angles_,
      wheel_speeds_;  ///< Last commanded steering angles [rad], used for pi-ambiguity resolution.

  // kinematic parameters
  std::array<Eigen::Vector2d, 2> wheel_positions_;
  double wheel_radius_;
};

}  // namespace franka_mobile