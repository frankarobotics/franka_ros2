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

#include <franka_gazebo_hardware/gravity_compensation_model.hpp>

#include <cmath>
#include <cstddef>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace franka_gazebo_hardware {

auto GravityCompensationModel::build(const std::string& urdf_xml,
                                     const std::set<std::string>& simulated_joint_names,
                                     const std::set<std::string>& effort_joint_names) -> void {
  configuration_joints_.clear();
  effort_joints_.clear();

  pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
  data_ = pinocchio::Data(model_);
  // Start from the neutral configuration rather than all-zeros so that any
  // movable joint NOT fed by the simulation (passive/mimic joints, or joints not
  // exposed to ros2_control) keeps a valid neutral pose. For continuous joints
  // neutral is (cos, sin) = (1, 0); all-zeros would be the invalid (0, 0).
  //
  // Assumption / known bias: only joints listed in simulated_joint_names get
  // their live position packed into the configuration each cycle. Every other
  // movable joint stays at its neutral pose, so its weight is evaluated as if it
  // never moved. All ros2_control-exposed movable joints ARE simulated, so this
  // only affects joints absent from ros2_control, whose live state the simulation
  // does not hand us.
  joint_configuration_ = pinocchio::neutral(model_);

  int configuration_index = 0;
  int velocity_index = 0;
  for (std::size_t i = 1; i < model_.names.size(); ++i) {
    const std::string& joint_name = model_.names[i];

    if (simulated_joint_names.count(joint_name) != 0) {
      configuration_joints_.push_back({joint_name, configuration_index, model_.nqs[i] == 2});

      if (effort_joint_names.count(joint_name) != 0) {
        effort_joints_.push_back({joint_name, velocity_index});
      }
    }

    configuration_index += model_.nqs[i];
    velocity_index += model_.nvs[i];
  }
}

auto GravityCompensationModel::computeGravityTorque(const std::vector<double>& position_values,
                                                    std::vector<double>& effort_torques) -> void {
  for (std::size_t i = 0; i < configuration_joints_.size(); ++i) {
    const ConfigurationJoint& joint = configuration_joints_[i];
    const double position = position_values[i];
    if (joint.is_continuous) {
      joint_configuration_(joint.configuration_index) = std::cos(position);
      joint_configuration_(joint.configuration_index + 1) = std::sin(position);
    } else {
      joint_configuration_(joint.configuration_index) = position;
    }
  }

  const Eigen::VectorXd& gravity_torque =
      pinocchio::computeGeneralizedGravity(model_, data_, joint_configuration_);

  for (std::size_t i = 0; i < effort_joints_.size(); ++i) {
    effort_torques[i] = gravity_torque(effort_joints_[i].velocity_index);
  }
}

auto GravityCompensationModel::computeGravityTorque(const std::vector<double>& position_values)
    -> std::vector<double> {
  std::vector<double> effort_torques(effort_joints_.size());
  computeGravityTorque(position_values, effort_torques);
  return effort_torques;
}

}  // namespace franka_gazebo_hardware
