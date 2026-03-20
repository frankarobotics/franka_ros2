// Copyright 2026 Franka Robotics Gmbh
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

#include <rcpputils/version.h>
#include <rclcpp/time.hpp>

// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace franka_mobile {

/**
 * @brief Tracks the pose and velocity of a holonomic (omnidirectional) platform.
 *
 * Integrates body-frame velocities (vx, vy, wz) over time to maintain an estimate
 * of the robot's 2D pose (x, y, heading) in the odometry frame. Velocities are
 * smoothed using rolling mean accumulators to reduce noise.
 *
 * Example usage:
 * @code
 * Odometry odom(10);  // 10-sample rolling window for velocity smoothing
 * odom.init(node->now());
 *
 * // In the control loop:
 * odom.update(vx, vy, wz, node->now());
 * geometry_msgs::msg::Pose2D pose;
 * pose.x = odom.getX();
 * pose.y = odom.getY();
 * pose.theta = odom.getHeading();
 * @endcode
 */
class Odometry {
 public:
  /**
   * @brief Construct an Odometry tracker.
   *
   * @param velocity_rolling_window_size  Number of samples used for rolling mean
   *                                      velocity smoothing. Larger values reduce
   *                                      noise but increase latency. Default is 10.
   */
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  /**
   * @brief Initialize the odometry state and timestamp.
   *
   * Resets the pose to the origin, zeroes all velocities and accumulators,
   * and records the given time as the integration start point.
   * Must be called before the first call to update().
   *
   * @param time  Current ROS time.
   */
  void init(const rclcpp::Time& time);

  /**
   * @brief Integrate a new velocity measurement and update pose and velocity estimates.
   *
   * Computes the elapsed time since the last update, integrates the displacement
   * into the pose estimate, and updates the smoothed velocity via rolling mean
   * accumulators. Selects between Runge-Kutta 2 and exact integration based on
   * the magnitude of the angular velocity.
   *
   * @param linear_x  Body-frame linear velocity along X [m/s].
   * @param linear_y  Body-frame linear velocity along Y [m/s].
   * @param angular   Body-frame angular velocity around Z [rad/s].
   * @param time      Current ROS time. Used to compute the integration timestep.
   */
  void update(double linear_x, double linear_y, double angular, const rclcpp::Time& time);

  /**
   * @brief Reset the pose estimate to the origin and zero all velocities.
   *
   * Does not reset the timestamp or rolling window size.
   */
  void resetOdometry();

  [[nodiscard]] double getX() const { return x_; }
  [[nodiscard]] double getY() const { return y_; }
  [[nodiscard]] double getHeading() const { return heading_; }
  [[nodiscard]] double getLinearX() const { return linear_x_; }
  [[nodiscard]] double getLinearY() const { return linear_y_; }
  [[nodiscard]] double getAngular() const { return angular_; }

  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

 private:
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

  void integrate(double linear_x, double linear_y, double angular);
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_{0};

  // Current pose:
  double x_{0.0};        //   [m]
  double y_{0.0};        //   [m]
  double heading_{0.0};  // [rad]

  // Current velocity:
  double linear_x_{0.0};  //   [m/s]
  double linear_y_{0.0};  //   [m/s]
  double angular_{0.0};   // [rad/s]

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_{10};
  RollingMeanAccumulator linear_x_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace franka_mobile
