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

#include <rclcpp/time.hpp>
#include <rcpputils/version.h>

// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace franka_example_controllers {

class Odometry {
 public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time& time);
  bool update(const std::array<double, 2>& steering_positions,
              const std::array<double, 2>& wheel_velocities,
              const rclcpp::Time& time);
  void update(double linear_x, double linear_y, double angular, const rclcpp::Time& time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinearX() const { return linear_x_; }
  double getLinearY() const { return linear_y_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

 private:
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

  void integrateRungeKutta2(double linear_x, double linear_y, double angular);
  void integrateExact(double linear_x, double linear_y, double angular);
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_x_;  //   [m/s]
  double linear_y_;  //   [m/s]
  double angular_;   // [rad/s]

  // Previous wheel position/state [rad]:
  double left_wheel_old_pos_;
  double right_wheel_old_pos_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_x_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace franka_example_controllers
