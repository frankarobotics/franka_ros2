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

#include <algorithm>
#include <array>

#include <Eigen/Dense>
#include <diff_drive_controller/speed_limiter.hpp>
#include <franka_mobile/swerve_drive_controller_parameters.hpp>

namespace franka_mobile {
namespace detail {

/**
 * @brief Non-owning view of the three per-axis command limiters.
 *
 * A missing limiter (nullptr) disables limiting on every axis; the controller
 * passes all three or none.
 */
struct AxisLimiters {
  diff_drive_controller::SpeedLimiter* linear_x{nullptr};
  diff_drive_controller::SpeedLimiter* linear_y{nullptr};
  diff_drive_controller::SpeedLimiter* angular_z{nullptr};
};

/**
 * @brief Planar command under test plus the history the limiter differentiates against.
 *
 * previous_commands[0] is the second-to-last command, [1] is the last command.
 */
struct CommandState {
  double& linear_x;
  double& linear_y;
  double& angular_z;
  std::array<Eigen::Vector3d, 2>& previous_commands;
};

/**
 * @brief Applies the swerve command-limiting policy through the full limiter.
 *
 * Acceleration limiting holds the previous command when the update period is zero,
 * while the velocity clamp runs on every call. The jerk stage continues the last
 * step (v0 + (v0 - v1)). Negative periods are clamped to zero before reaching the
 * limiter. Advances the command history on every call, including when limiting is
 * disabled or a limiter is missing.
 *
 * Runs on the 1 kHz path: no allocation, no throw, no logging.
 *
 * @param limiters Per-axis limiters; nullptrs disable limiting on every axis.
 * @param speed_limits_enabled Master switch from parameters.
 * @param command Commanded velocities, clamped in place, plus command history,
 * shifted forward with the (possibly clamped) command.
 * @param limiter_dt Update period in seconds; values <= 0 hold the previous command
 * under acceleration limiting while the velocity clamp still applies.
 */
inline void applyCommandLimits(AxisLimiters limiters,
                               bool speed_limits_enabled,
                               CommandState command,
                               double limiter_dt) {
  const double last_linear_x = command.previous_commands[1].x();
  const double second_to_last_linear_x = command.previous_commands[0].x();

  const double last_linear_y = command.previous_commands[1].y();
  const double second_to_last_linear_y = command.previous_commands[0].y();

  const double last_angular = command.previous_commands[1].z();
  const double second_to_last_angular = command.previous_commands[0].z();

  // The full limiter holds the previous command at dt == 0 (the acceleration stage
  // clamps dv to [0, 0]) while the velocity clamp still runs. The jerk stage continues the
  // last step (v0 + (v0 - v1)). Negative periods never reach the
  // limiter: std::clamp with low > high is undefined behaviour.
  if (speed_limits_enabled && limiters.linear_x != nullptr && limiters.linear_y != nullptr &&
      limiters.angular_z != nullptr) {
    const double sanitized_dt = std::max(limiter_dt, 0.0);
    limiters.linear_x->limit(command.linear_x, last_linear_x, second_to_last_linear_x,
                             sanitized_dt);
    limiters.linear_y->limit(command.linear_y, last_linear_y, second_to_last_linear_y,
                             sanitized_dt);
    limiters.angular_z->limit(command.angular_z, last_angular, second_to_last_angular,
                              sanitized_dt);
  }
  command.previous_commands[0] = command.previous_commands[1];
  command.previous_commands[1] =
      Eigen::Vector3d(command.linear_x, command.linear_y, command.angular_z);
}

/**
 * @brief Builds one axis limiter from its generated parameter block.
 *
 * Single home of the parameter-to-argument mapping shared by production
 * (SwerveDriveController::on_configure) and the limiter gtests. Accepts any of
 * swerve_drive_controller::Params::Linear::X, ::Linear::Y or ::Angular::Z, which
 * share field names.
 *
 * Uses the same deprecated 9-argument SpeedLimiter constructor production has
 * always used: min_acceleration feeds the max_deceleration slot, matching the old
 * convention (max_deceleration <= 0), and max_acceleration feeds max_acceleration.
 */
template <typename AxisParameters>
[[nodiscard]] auto makeSpeedLimiter(const AxisParameters& axis_parameters)
    -> diff_drive_controller::SpeedLimiter {
  return diff_drive_controller::SpeedLimiter(
      axis_parameters.has_velocity_limits, axis_parameters.has_acceleration_limits,
      axis_parameters.has_jerk_limits, axis_parameters.min_velocity, axis_parameters.max_velocity,
      axis_parameters.min_acceleration, axis_parameters.max_acceleration, axis_parameters.min_jerk,
      axis_parameters.max_jerk);
}

}  // namespace detail
}  // namespace franka_mobile
