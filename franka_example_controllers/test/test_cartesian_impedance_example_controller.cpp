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

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"

#include "franka_example_controllers/detail/cartesian_impedance_math.hpp"
#include "franka_example_controllers/fr3/cartesian_impedance_example_controller.hpp"
#include "franka_hardware/model.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using hardware_interface::StateInterface;

namespace {
// Deterministic stand-in for the libfranka model behind the robot_model state interface.
// Only the two calls the controller makes are overridden; everything else is unused.
class FakeFrankaModel : public franka_hardware::Model {
 public:
  FakeFrankaModel() : franka_hardware::Model() {}

  [[nodiscard]] std::array<double, 7> coriolis(
      const franka::RobotState& /*robot_state*/) const override {
    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  [[nodiscard]] std::array<double, 42> zeroJacobian(
      franka::Frame /*frame*/,
      const franka::RobotState& /*robot_state*/) const override {
    // Full row rank: first six columns identity, last column zero (column-major).
    std::array<double, 42> jacobian{};
    for (size_t i = 0; i < 6; ++i) {
      jacobian[i * 6 + i] = 1.0;
    }
    return jacobian;
  }
};

double encodePointer(const void* pointer) {
  double encoded = 0.0;
  static_assert(sizeof(pointer) == sizeof(encoded));
  std::memcpy(&encoded, &pointer, sizeof(encoded));
  return encoded;
}

constexpr double kTorqueNonzeroThreshold = 1e-9;
constexpr double kDampingFactorReference = 0.2;
constexpr double kPseudoInverseEquivalenceTolerance = 1e-9;
// Motion-target semantics: external equilibrium 1 m off in x, 90 degrees about z.
constexpr double kExternalPositionX = 1.3;
constexpr int kMotionTargetUpdates = 2000;
constexpr int kHoldUpdates = 1000;
// Spin the subscription on these leading updates so a late same-process delivery still
// lands with enough cycles left to converge (no RMW synchronicity assumption).
constexpr int kDeliverySpinUpdates = 200;
// Demo progress after 2 s reads ~+7.7 N on x; the external 1 m offset would read ~+150 N.
constexpr double kDemoProgressTorqueMin = 3.0;
constexpr double kExternalPullTorqueMax = 20.0;
// Converged 90-degree yaw reads ~+7.07 N on the z torque; a chasing orientation reads ~0.
constexpr double kOrientationHoldTorqueMin = 5.0;
constexpr double kOffAxisTorqueTolerance = 1.0;

using CartesianImpedanceExampleController =
    franka_example_controllers::CartesianImpedanceExampleController;

// Reference snapshot of the pre-release damped pseudo-inverse: full-SVD formulation
// (A^T A + λ²I)^-1 A^T with A = J^T. Kept dynamic-Eigen on purpose — it mirrors the exact
// code the fixed-size LDLT formulation replaces.
Eigen::Matrix<double, 6, 7> dampedPseudoInverseReference(
    const Eigen::Matrix<double, 6, 7>& jacobian) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian.transpose().eval(),
                                        Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues();
  Eigen::MatrixXd weighting = jacobian.transpose();
  weighting.setZero();
  for (int i = 0; i < sing_vals.size(); i++) {
    weighting(i, i) = sing_vals(i) / (sing_vals(i) * sing_vals(i) +
                                      kDampingFactorReference * kDampingFactorReference);
  }
  return svd.matrixV() * weighting.transpose() * svd.matrixU().transpose();
}

void expectPseudoInverseEquivalent(const Eigen::Matrix<double, 6, 7>& jacobian) {
  const Eigen::Matrix<double, 6, 7> fixed_size =
      franka_example_controllers::detail::computeDampedPseudoInverse(jacobian,
                                                                     kDampingFactorReference);
  const Eigen::Matrix<double, 6, 7> reference = dampedPseudoInverseReference(jacobian);
  EXPECT_LT((fixed_size - reference).cwiseAbs().maxCoeff(), kPseudoInverseEquivalenceTolerance);
}
}  // namespace

class CartesianImpedanceExampleControllerTest : public ::testing::Test {
 public:
  static void SetUpTestSuite();
  static void TearDownTestSuite();

  void SetUp();
  void TearDown();

  void SetUpController(const std::string& robot_type, const std::string& arm_prefix);

  void AssignFr3v2Interfaces();

 protected:
  std::unique_ptr<franka_example_controllers::CartesianImpedanceExampleController> controller_;

  // Effort command storage, poisoned with NaN in SetUp() so a missing write is visible.
  std::array<double, 7> effort_commands_;
  // Valid fr3v2 joint state: a nonsingular pose with a small joint-1 velocity so the
  // impedance law must produce nonzero torques.
  std::array<double, 7> joint_positions_ = {0.0, -0.785398, 0.0, -2.35619, 0.0, 1.5708, 0.785398};
  std::array<double, 7> joint_velocities_ = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Valid end-effector pose: identity rotation, 0.3/0.1/0.5 m translation (column-major).
  std::array<double, 16> cartesian_pose_ = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0, 0.3, 0.1, 0.5, 1.0};

  FakeFrankaModel fake_model_;
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState> robot_state_box_;
  double encoded_model_pointer_ = 0.0;
  double encoded_state_box_pointer_ = 0.0;

  std::vector<CommandInterface> command_handles_;
  std::vector<StateInterface> state_handles_;
};

void CartesianImpedanceExampleControllerTest::SetUpTestSuite() {
  rclcpp::init(0, nullptr);
}

void CartesianImpedanceExampleControllerTest::TearDownTestSuite() {
  rclcpp::shutdown();
}

void CartesianImpedanceExampleControllerTest::SetUp() {
  controller_ = std::make_unique<franka_example_controllers::CartesianImpedanceExampleController>();
  effort_commands_.fill(std::numeric_limits<double>::quiet_NaN());
}

void CartesianImpedanceExampleControllerTest::TearDown() {
  controller_.reset(nullptr);
}

void CartesianImpedanceExampleControllerTest::SetUpController(const std::string& robot_type,
                                                              const std::string& arm_prefix) {
  const auto result = controller_->init("test_cartesian_impedance_example",
                                        ros2_control_test_assets::minimal_robot_urdf, 30, "",
                                        rclcpp::NodeOptions()
                                            .allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  controller_->get_node()->set_parameter({"robot_type", robot_type});
  controller_->get_node()->set_parameter({"arm_prefix", arm_prefix});
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(CartesianImpedanceExampleControllerTest, interface_names_follow_robot_type) {
  SetUpController("fr3v2", "");

  std::vector<std::string> expected_command_interfaces;
  std::vector<std::string> expected_state_interfaces;
  for (int i = 1; i <= 7; ++i) {
    expected_command_interfaces.push_back("fr3v2_joint" + std::to_string(i) + "/effort");
    expected_state_interfaces.push_back("fr3v2_joint" + std::to_string(i) + "/position");
    expected_state_interfaces.push_back("fr3v2_joint" + std::to_string(i) + "/velocity");
  }

  const auto command_config = controller_->command_interface_configuration();
  EXPECT_EQ(command_config.names, expected_command_interfaces);

  const auto state_config = controller_->state_interface_configuration();
  ASSERT_GE(state_config.names.size(), expected_state_interfaces.size());
  for (size_t i = 0; i < expected_state_interfaces.size(); ++i) {
    EXPECT_EQ(state_config.names[i], expected_state_interfaces[i]);
  }
}

TEST_F(CartesianImpedanceExampleControllerTest, interface_names_follow_arm_prefix) {
  SetUpController("fr3v2", "left");

  const auto command_config = controller_->command_interface_configuration();
  ASSERT_EQ(command_config.names.size(), 7U);
  for (int i = 1; i <= 7; ++i) {
    EXPECT_EQ(command_config.names[i - 1], "left_fr3v2_joint" + std::to_string(i) + "/effort");
  }
}

TEST_F(CartesianImpedanceExampleControllerTest, stale_arm_id_warns_and_keeps_robot_type) {
  // controller_manager NodeOptions: no auto-declare; arm_id arrives as an override, so
  // only the on_init declaration makes it visible to on_configure. Deliberately not via
  // SetUpController(), whose auto-declare options would declare it behind the back of
  // the code under test. No set_parameter for arm_id: the override is the whole point.
  const auto result = controller_->init(
      "test_cartesian_impedance_example", ros2_control_test_assets::minimal_robot_urdf, 30, "",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("arm_id", "fr3_legacy")}));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  controller_->get_node()->set_parameter({"robot_type", "fr3"});
  controller_->get_node()->set_parameter({"arm_prefix", ""});
  testing::internal::CaptureStderr();
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  const std::string log_output = testing::internal::GetCapturedStderr();
  EXPECT_NE(log_output.find("set 'robot_type' instead"), std::string::npos);

  // No fallback: the interface names still follow robot_type, not arm_id.
  const auto command_config = controller_->command_interface_configuration();
  ASSERT_EQ(command_config.names.size(), 7U);
  for (int i = 1; i <= 7; ++i) {
    EXPECT_EQ(command_config.names[i - 1], "fr3_joint" + std::to_string(i) + "/effort");
  }
}

void CartesianImpedanceExampleControllerTest::AssignFr3v2Interfaces() {
  command_handles_.reserve(7);
  for (int i = 0; i < 7; ++i) {
    command_handles_.emplace_back("fr3v2_joint" + std::to_string(i + 1), HW_IF_EFFORT,
                                  &effort_commands_[i]);
  }

  // Declaration order of state_interface_configuration(): joint position/velocity pairs,
  // then the 16 cartesian pose entries, then robot_model and robot_state.
  state_handles_.reserve(32);
  for (int i = 0; i < 7; ++i) {
    state_handles_.emplace_back("fr3v2_joint" + std::to_string(i + 1), HW_IF_POSITION,
                                &joint_positions_[i]);
    state_handles_.emplace_back("fr3v2_joint" + std::to_string(i + 1), HW_IF_VELOCITY,
                                &joint_velocities_[i]);
  }
  for (int i = 0; i < 16; ++i) {
    state_handles_.emplace_back(std::to_string(i), "cartesian_pose_state", &cartesian_pose_[i]);
  }

  franka::RobotState box_state;
  box_state.q = joint_positions_;
  box_state.dq = joint_velocities_;
  box_state.O_T_EE = cartesian_pose_;
  robot_state_box_.set(box_state);

  const auto* model_pointer = static_cast<const franka_hardware::Model*>(&fake_model_);
  encoded_model_pointer_ = encodePointer(model_pointer);
  const auto* box_pointer =
      static_cast<const realtime_tools::RealtimeThreadSafeBox<franka::RobotState>*>(
          &robot_state_box_);
  encoded_state_box_pointer_ = encodePointer(box_pointer);
  state_handles_.emplace_back("fr3v2", "robot_model", &encoded_model_pointer_);
  state_handles_.emplace_back("fr3v2", "robot_state", &encoded_state_box_pointer_);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.reserve(command_handles_.size());
  for (auto& handle : command_handles_) {
    command_ifs.emplace_back(handle);
  }
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.reserve(state_handles_.size());
  for (auto& handle : state_handles_) {
    state_ifs.emplace_back(handle);
  }
  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(CartesianImpedanceExampleControllerTest,
       givenValidFr3v2State_whenFirstUpdate_thenFiniteNonzeroTorquesOnAllEffortInterfaces) {
  SetUpController("fr3v2", "");
  AssignFr3v2Interfaces();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)),
            controller_interface::return_type::OK);

  double max_abs_torque = 0.0;
  for (int i = 0; i < 7; ++i) {
    ASSERT_TRUE(std::isfinite(effort_commands_[i])) << "effort interface " << i << " not finite";
    max_abs_torque = std::max(max_abs_torque, std::abs(effort_commands_[i]));
  }
  // The joint-1 velocity must show up in the impedance law: all-zero output would mean the
  // controller computes nothing while the hardware keeps sending the switch-time zero fill.
  EXPECT_GT(max_abs_torque, kTorqueNonzeroThreshold);
}

TEST_F(CartesianImpedanceExampleControllerTest,
       givenJacobians_whenDampedPseudoInverse_thenMatchesSvdReference) {
  // Full row rank: first six columns identity, last column zero.
  Eigen::Matrix<double, 6, 7> full_rank = Eigen::Matrix<double, 6, 7>::Zero();
  full_rank.leftCols(6).setIdentity();
  expectPseudoInverseEquivalent(full_rank);

  // Near-singular: two rows almost parallel, as near a wrist singularity.
  Eigen::Matrix<double, 6, 7> near_singular = full_rank;
  near_singular.row(5) = near_singular.row(4) + 1e-4 * Eigen::Matrix<double, 1, 7>::Ones();
  expectPseudoInverseEquivalent(near_singular);

  // Rank-deficient: duplicated row, so one Cartesian direction is unreachable.
  Eigen::Matrix<double, 6, 7> rank_deficient = full_rank;
  rank_deficient.row(5) = rank_deficient.row(4);
  expectPseudoInverseEquivalent(rank_deficient);
}

TEST_F(CartesianImpedanceExampleControllerTest,
       givenExternalEquilibriumPose_whenUpdatesRun_thenOrientationHoldsAndPositionFollowsDemo) {
  // Zero velocity so the torques read the Cartesian error directly (no damping term,
  // no nullspace motion on the static joints).
  joint_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  SetUpController("fr3v2", "");
  AssignFr3v2Interfaces();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // External target: 1 m off in x only, 90 degrees about z. The demo trajectory must
  // ignore the position (it keeps generating its own motion) but hold the orientation.
  rclcpp::Node publisher_node("motion_target_test_publisher");
  auto publisher = publisher_node.create_publisher<geometry_msgs::msg::PoseStamped>(
      "/test_cartesian_impedance_example/equilibrium_pose", rclcpp::QoS(10));
  const auto match_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < match_deadline) {
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);

  geometry_msgs::msg::PoseStamped external_target;
  external_target.pose.position.x = kExternalPositionX;
  external_target.pose.position.y = 0.1;
  external_target.pose.position.z = 0.5;
  external_target.pose.orientation.z = 0.7071068;
  external_target.pose.orientation.w = 0.7071068;
  publisher->publish(external_target);
  ASSERT_TRUE(publisher->wait_for_all_acked(std::chrono::seconds(5)));

  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);
  for (int i = 0; i < kMotionTargetUpdates; ++i) {
    if (i < kDeliverySpinUpdates) {
      rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    }
    ASSERT_EQ(controller_->update(rclcpp::Time(0), period), controller_interface::return_type::OK);
  }

  // Orientation converged to the published yaw: tau_z ≈ -10 * (-sin(45°)) ≈ +7.07.
  // A rewrite passing the measured orientation would chase identity and read ~0.
  EXPECT_GT(effort_commands_[5], kOrientationHoldTorqueMin);
  EXPECT_NEAR(effort_commands_[3], 0.0, kOffAxisTorqueTolerance);
  EXPECT_NEAR(effort_commands_[4], 0.0, kOffAxisTorqueTolerance);
  // Position follows the demo trajectory (x progressed ~5 cm in 2 s → ~+7.7 N), not the
  // published 1 m offset (which would read ~+150 N). A stuck demo would read ~0.
  EXPECT_GT(effort_commands_[0], kDemoProgressTorqueMin);
  EXPECT_LT(effort_commands_[0], kExternalPullTorqueMax);

  // Hold: another second of demo motion must not overwrite the external orientation.
  for (int i = 0; i < kHoldUpdates; ++i) {
    ASSERT_EQ(controller_->update(rclcpp::Time(0), period), controller_interface::return_type::OK);
  }
  EXPECT_GT(effort_commands_[5], kOrientationHoldTorqueMin);
}
