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

#include <Eigen/Dense>

namespace franka_example_controllers {
namespace detail {

// Fixed dimensions of the Cartesian impedance problem. Canonical home of the sizes the
// controller and its tests share; kept out of the controller's public class API.
constexpr int num_joints{7};
constexpr int num_cartesian_dof{6};

/**
 * Damped pseudo-inverse of a 6x7 Jacobian, (J J^T + damping^2 I)^-1 J.
 *
 * Allocation-free fixed-size formulation; safe to call on the real-time thread.
 * Numerically equivalent to the SVD-based damped inverse with the same damping.
 *
 * @param jacobian 6x7 zero Jacobian
 * @param damping Damping factor keeping near-singular directions bounded
 * @return 6x7 damped pseudo-inverse
 */
inline Eigen::Matrix<double, num_cartesian_dof, num_joints> computeDampedPseudoInverse(
    const Eigen::Matrix<double, num_cartesian_dof, num_joints>& jacobian,
    double damping) {
  Eigen::Matrix<double, num_cartesian_dof, num_cartesian_dof> square =
      jacobian * jacobian.transpose();
  square.diagonal().array() += damping * damping;
  return square.ldlt().solve(jacobian);
}

}  // namespace detail
}  // namespace franka_example_controllers
