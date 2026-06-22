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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include <franka_gazebo_hardware/gravity_compensation_model.hpp>

namespace {

using franka_gazebo_hardware::GravityCompensationModel;

// Analytic torques are scaled by the model's own gravity magnitude (read back from
// pinocchio per test) so the suite stays correct if pinocchio changes its default.

constexpr double kTorqueTolerance = 1e-6;
constexpr double kZeroTorqueTolerance = 1e-9;

constexpr double kPendulumMass = 2.0;
constexpr double kPendulumLength = 1.0;
constexpr double kRevoluteAngle = 0.5;
constexpr double kContinuousAngle = 0.7;

constexpr double kFirstJointAngle = 0.3;
constexpr double kSecondJointAngle = 0.4;

// At kFirstJointAngle the first link's centre of mass swings off the gravity axis,
// so the first joint alone carries mass * g * length * sin(angle) ~= 2.9 N*m, and the
// downstream link only adds to that magnitude. 2.0 N*m is a strict, non-degenerate
// lower bound that catches a zero or wrong-joint result.
constexpr double kMinFirstJointTorque = 2.0;

class GravityCompensationModelTest : public ::testing::Test {
 protected:
  // A single pendulum rotating about the x-axis. The link centre of mass sits at
  // (0, 0, -length) in the link frame, i.e. hanging straight down at angle = 0, so
  // the analytic gravity torque is mass * g * length * sin(angle).
  static auto buildPendulumUrdf(const std::string& joint_type,
                                double mass,
                                double length) -> std::string {
    return R"(<?xml version="1.0"?>
<robot name="pendulum">
  <link name="base_link"/>
  <link name="pendulum_link">
    <inertial>
      <origin xyz="0 0 -)" +
           std::to_string(length) + R"(" rpy="0 0 0"/>
      <mass value=")" +
           std::to_string(mass) + R"("/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="pendulum_joint" type=")" +
           joint_type + R"(">
    <parent link="base_link"/>
    <child link="pendulum_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
</robot>)";
  }

  // Two serial revolute joints; geometry is irrelevant for the mapping test.
  static auto buildTwoJointUrdf() -> std::string {
    return R"(<?xml version="1.0"?>
<robot name="double_pendulum">
  <link name="base_link"/>
  <link name="link_a">
    <inertial>
      <origin xyz="0 0 -1.0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="link_b">
    <inertial>
      <origin xyz="0 0 -1.0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="joint_a" type="revolute">
    <parent link="base_link"/>
    <child link="link_a"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.0" upper="3.0" effort="100" velocity="10"/>
  </joint>
  <joint name="joint_b" type="revolute">
    <parent link="link_a"/>
    <child link="link_b"/>
    <origin xyz="0 0 -2.0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.0" upper="3.0" effort="100" velocity="10"/>
  </joint>
</robot>)";
  }
};

TEST_F(GravityCompensationModelTest,
       givenRevolutePendulumAtAngle_whenComputingTorque_thenMatchesAnalyticValue) {
  // Given a revolute pendulum displaced from the gravity axis.
  GravityCompensationModel model;
  model.build(buildPendulumUrdf("revolute", kPendulumMass, kPendulumLength), {"pendulum_joint"},
              {"pendulum_joint"});

  // When the gravity torque is computed.
  const std::vector<double> torque = model.computeGravityTorque({kRevoluteAngle});

  // Then it equals the analytic pendulum torque mass * g * length * sin(angle).
  const double gravity = model.model().gravity.linear().norm();
  ASSERT_EQ(torque.size(), 1u);
  EXPECT_NEAR(torque[0], kPendulumMass * gravity * kPendulumLength * std::sin(kRevoluteAngle),
              kTorqueTolerance);
}

TEST_F(GravityCompensationModelTest,
       givenCenterOfMassOnGravityAxis_whenComputingTorque_thenTorqueIsZero) {
  // Given a revolute pendulum whose centre of mass lies on the gravity axis.
  GravityCompensationModel model;
  model.build(buildPendulumUrdf("revolute", kPendulumMass, kPendulumLength), {"pendulum_joint"},
              {"pendulum_joint"});

  // When/Then at angle 0 the centre of mass hangs straight down -> no torque.
  EXPECT_NEAR(model.computeGravityTorque({0.0}).at(0), 0.0, kZeroTorqueTolerance);
  // When/Then at angle pi it stands straight up, still on the axis -> no torque.
  EXPECT_NEAR(model.computeGravityTorque({M_PI}).at(0), 0.0, kTorqueTolerance);
}

TEST_F(GravityCompensationModelTest,
       givenContinuousJoint_whenComputingTorque_thenConfigurationPacksCosineAndSine) {
  // Given a continuous pendulum, stored in pinocchio as (cos, sin) -> two nq entries.
  GravityCompensationModel model;
  model.build(buildPendulumUrdf("continuous", kPendulumMass, kPendulumLength), {"pendulum_joint"},
              {"pendulum_joint"});

  // When the model is inspected and the gravity torque computed.
  const std::vector<double> torque = model.computeGravityTorque({kContinuousAngle});

  // Then the configuration occupies two entries flagged continuous, and despite the
  // cos/sin packing the physical torque matches the revolute analytic value.
  const double gravity = model.model().gravity.linear().norm();
  ASSERT_EQ(model.model().nq, 2);
  ASSERT_EQ(model.configurationJoints().size(), 1u);
  EXPECT_TRUE(model.configurationJoints().front().is_continuous);
  ASSERT_EQ(torque.size(), 1u);
  EXPECT_NEAR(torque[0], kPendulumMass * gravity * kPendulumLength * std::sin(kContinuousAngle),
              kTorqueTolerance);
}

TEST_F(GravityCompensationModelTest,
       givenOnlyOneJointHasEffortInterface_whenComputingTorque_thenSingleTorqueMapsToThatJoint) {
  // Given two simulated joints where only joint_a exposes an effort command.
  GravityCompensationModel model;
  model.build(buildTwoJointUrdf(), {"joint_a", "joint_b"}, {"joint_a"});

  // When the torque is computed with one position per configuration joint.
  const std::vector<double> torque =
      model.computeGravityTorque({kFirstJointAngle, kSecondJointAngle});

  // Then both joints are configuration joints but only joint_a is an effort joint.
  ASSERT_EQ(model.configurationJoints().size(), 2u);
  EXPECT_EQ(model.configurationJoints()[0].joint_name, "joint_a");
  EXPECT_EQ(model.configurationJoints()[1].joint_name, "joint_b");
  ASSERT_EQ(model.effortJoints().size(), 1u);
  EXPECT_EQ(model.effortJoints().front().joint_name, "joint_a");

  // And the single returned torque is joint_a's, above its non-degenerate lower bound.
  ASSERT_EQ(torque.size(), 1u);
  EXPECT_GT(std::abs(torque[0]), kMinFirstJointTorque);

  // And mapping effort to joint_b instead yields a different value, proving the
  // mapping selects the correct joint rather than always returning the first.
  GravityCompensationModel model_with_effort_on_second_joint;
  model_with_effort_on_second_joint.build(buildTwoJointUrdf(), {"joint_a", "joint_b"}, {"joint_b"});
  const std::vector<double> second_joint_torque =
      model_with_effort_on_second_joint.computeGravityTorque({kFirstJointAngle, kSecondJointAngle});
  ASSERT_EQ(second_joint_torque.size(), 1u);
  EXPECT_NE(torque[0], second_joint_torque[0]);
}

TEST_F(GravityCompensationModelTest,
       givenUnsimulatedJoint_whenBuildingModel_thenJointIsExcludedFromConfiguration) {
  // Given a two-joint robot where joint_b is not in the simulated set.
  GravityCompensationModel model;

  // When the model is built with only joint_a simulated.
  model.build(buildTwoJointUrdf(), {"joint_a"}, {"joint_a"});

  // Then joint_b appears in neither the configuration nor the effort joints.
  ASSERT_EQ(model.configurationJoints().size(), 1u);
  EXPECT_EQ(model.configurationJoints().front().joint_name, "joint_a");
  ASSERT_EQ(model.effortJoints().size(), 1u);
}

}  // namespace
