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

#include <cmath>

#include <franka_mobile/odometry.hpp>

namespace franka_mobile {

Odometry::Odometry(size_t velocity_rolling_window_size)
    : velocity_rolling_window_size_(velocity_rolling_window_size),
      linear_x_accumulator_(velocity_rolling_window_size),
      linear_y_accumulator_(velocity_rolling_window_size),
      angular_accumulator_(velocity_rolling_window_size) {}

void Odometry::init(const rclcpp::Time& time) {
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

void Odometry::update(double linear_x, double linear_y, double angular, const rclcpp::Time& time) {
  linear_x_accumulator_.accumulate(linear_x);
  linear_y_accumulator_.accumulate(linear_y);
  angular_accumulator_.accumulate(angular);

  linear_x_ = linear_x_accumulator_.getRollingMean();
  linear_y_ = linear_y_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrate(linear_x * dt, linear_y * dt, angular * dt);
}

void Odometry::resetOdometry() {
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size) {
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

void Odometry::integrate(double linear_x, double linear_y, double angular) {
  const double direction = heading_ + angular * 0.5;
  const double cos_d = std::cos(direction);
  const double sin_d = std::sin(direction);

  x_ += linear_x * cos_d - linear_y * sin_d;
  y_ += linear_x * sin_d + linear_y * cos_d;
  heading_ += angular;
}

void Odometry::resetAccumulators() {
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace franka_mobile