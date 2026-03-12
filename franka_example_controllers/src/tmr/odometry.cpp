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

/*
 * Author: Andrea Franceschetti
 * Code inspired by ros2_controllers/diff_drive_controller
 */

#include <cmath>

#include <franka_example_controllers/tmr/odometry.hpp>

namespace franka_example_controllers {

Odometry::Odometry(size_t velocity_rolling_window_size)
    : timestamp_(0.0),
      x_(0.0),
      y_(0.0),
      heading_(0.0),
      linear_x_(0.0),
      linear_y_(0.0),
      angular_(0.0),
      left_wheel_old_pos_(0.0),
      right_wheel_old_pos_(0.0),
      velocity_rolling_window_size_(velocity_rolling_window_size),
      linear_x_accumulator_(velocity_rolling_window_size),
      linear_y_accumulator_(velocity_rolling_window_size),
      angular_accumulator_(velocity_rolling_window_size) {}

void Odometry::init(const rclcpp::Time& time) {
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

// bool Odometry::updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time& time) {
//   const double dt = time.seconds() - timestamp_.seconds();
//   if (dt < 0.0001) {
//     return false;  // Interval too small to integrate with
//   }
//   // Compute linear and angular diff:
//   // const double linear = (left_vel + right_vel) * 0.5;
//   // // Now there is a bug about scout angular velocity
//   // const double angular = (right_vel - left_vel) / wheel_separation_;

//   // Integrate odometry:
//   // integrateExact(linear, angular);

//   // timestamp_ = time;

//   // // Estimate speeds using a rolling mean to filter them out:
//   // linear_accumulator_.accumulate(linear / dt);
//   // angular_accumulator_.accumulate(angular / dt);

//   // linear_ = linear_accumulator_.getRollingMean();
//   // angular_ = angular_accumulator_.getRollingMean();

//   return true;
// }

bool Odometry::update(const std::array<double, 2>& steering_positions,
                      const std::array<double, 2>& wheel_velocities,
                      const rclcpp::Time& time) {
                        return true;
                      }

void Odometry::update(double linear_x, double linear_y, double angular, const rclcpp::Time& time) {
  /// Save last linear and angular velocity:
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear_x * dt, linear_y * dt, angular * dt);
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

void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular) {
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear_x * std::cos(direction);
  y_ += linear_y * std::sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear_x, double linear_y, double angular) {
  if (fabs(angular) < 1e-6) {
    integrateRungeKutta2(linear_x, linear_y, angular);
  } else {
    // TODO revise exact integration
    /// Exact integration (should solve problems when angular is zero):
    // const double heading_old = heading_;
    // const double r = linear / angular;
    // heading_ += angular;
    // x_ += r * (std::sin(heading_) - std::sin(heading_old));
    // y_ += -r * (std::cos(heading_) - std::cos(heading_old));
  }
}

void Odometry::resetAccumulators() {
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace franka_example_controllers