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

#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace franka_gazebo_hardware {

/**
 * Pure gravity-compensation math, free of any Gazebo dependency.
 *
 * Builds a pinocchio model from a URDF string, maps the simulated joints to
 * their pinocchio configuration/velocity indices, and computes the per-joint
 * generalized-gravity torque for a given configuration. The simulation system
 * feeds it joint positions read from the entity-component manager and applies
 * the returned torque to the effort commands; this class never touches Gazebo,
 * which makes the model and torque math directly unit-testable.
 */
class GravityCompensationModel {
 public:
  /**
   * A simulated joint whose position contributes to the gravity configuration.
   */
  struct ConfigurationJoint {
    std::string joint_name;
    int configuration_index = 0;
    /**< Continuous joints are packed as (cos(theta), sin(theta)) and occupy two nq entries. */
    bool is_continuous = false;
  };

  /**
   * A simulated joint that receives gravity compensation on its effort command.
   */
  struct EffortJoint {
    std::string joint_name;
    int velocity_index = 0;
  };

  /**
   * Builds the pinocchio model and the joint index mappings. Only joints listed
   * in simulated_joint_names become configuration joints; the subset that is also
   * in effort_joint_names additionally becomes an effort joint.
   *
   * @param urdf_xml The robot URDF as an XML string
   * @param simulated_joint_names Joints whose live position is fed to the model
   * @param effort_joint_names Joints that receive a gravity-compensation torque
   * @throws std::runtime_error / std::invalid_argument on an unparseable URDF
   */
  auto build(const std::string& urdf_xml,
             const std::set<std::string>& simulated_joint_names,
             const std::set<std::string>& effort_joint_names) -> void;

  /**
   * Computes the generalized-gravity torque for each effort joint. Allocation-free
   * hot path used by the simulation system on every update cycle.
   *
   * @param position_values Positions aligned with configurationJoints() (same order, size)
   * @param effort_torques Output buffer the caller pre-sizes to effortJoints().size();
   *   filled with the per-effort-joint torque in effortJoints() order
   */
  auto computeGravityTorque(const std::vector<double>& position_values,
                            std::vector<double>& effort_torques) -> void;

  /**
   * Convenience value-returning overload for tests; allocates the result vector
   * and is not for the hot path.
   *
   * @param position_values Positions aligned with configurationJoints() (same order, size)
   * @return std::vector<double> Per-effort-joint torque in effortJoints() order
   */
  auto computeGravityTorque(const std::vector<double>& position_values) -> std::vector<double>;

  /**
   * @return const pinocchio::Model& The pinocchio model built from the URDF
   */
  auto model() const -> const pinocchio::Model& { return model_; }

  /**
   * @return const std::vector<ConfigurationJoint>& The simulated joints whose
   *   positions are packed into the gravity configuration, in configuration order
   */
  auto configurationJoints() const -> const std::vector<ConfigurationJoint>& {
    return configuration_joints_;
  }

  /**
   * @return const std::vector<EffortJoint>& The joints that receive a gravity
   *   torque, in the order computeGravityTorque() writes its output
   */
  auto effortJoints() const -> const std::vector<EffortJoint>& { return effort_joints_; }

 private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd joint_configuration_;
  std::vector<ConfigurationJoint> configuration_joints_;
  std::vector<EffortJoint> effort_joints_;
};

}  // namespace franka_gazebo_hardware
