#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <franka_mobile/detail/body_twist.hpp>
#include <franka_mobile/detail/swerve_command_limits.hpp>

#include <array>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr double kTol = 1e-9;
static constexpr double kVelocityLimit = 2.0;
static constexpr double kAccelerationLimit = 1.0;
static constexpr double kOversizedCommand = 5.0;
static constexpr double kUnitStep = 1.0;

// Representative axis block: Linear::X, Linear::Y and Angular::Z share field names,
// so the makeSpeedLimiter mapping under test is identical for all three. Production
// (on_configure) instantiates the factory for each of them.
using AxisParameters = swerve_drive_controller::Params::Linear::X;

static AxisParameters makeVelocityOnlyAxis() {
  AxisParameters axis;
  axis.has_velocity_limits = true;
  axis.has_acceleration_limits = false;
  axis.has_jerk_limits = false;
  axis.min_velocity = -kVelocityLimit;
  axis.max_velocity = kVelocityLimit;
  return axis;
}

static AxisParameters makeAccelerationOnlyAxis() {
  AxisParameters axis;
  axis.has_velocity_limits = false;
  axis.has_acceleration_limits = true;
  axis.has_jerk_limits = false;
  axis.min_acceleration = -kAccelerationLimit;
  axis.max_acceleration = kAccelerationLimit;
  return axis;
}

class SwerveCommandLimitsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    history_[0].setZero();
    history_[1].setZero();
  }

  void expectHistory(const Eigen::Vector3d& second_to_last, const Eigen::Vector3d& last) {
    EXPECT_NEAR(history_[0].x(), second_to_last.x(), kTol);
    EXPECT_NEAR(history_[0].y(), second_to_last.y(), kTol);
    EXPECT_NEAR(history_[0].z(), second_to_last.z(), kTol);
    EXPECT_NEAR(history_[1].x(), last.x(), kTol);
    EXPECT_NEAR(history_[1].y(), last.y(), kTol);
    EXPECT_NEAR(history_[1].z(), last.z(), kTol);
  }

  std::array<Eigen::Vector3d, 2> history_;
};

// ===========================================================================
// Command limiting with a non-positive update period
// ===========================================================================

TEST_F(SwerveCommandLimitsTest, WhenDtIsZeroAssertVelocityClampedOnAllAxes) {
  auto linear_x_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  double command_x = kOversizedCommand, command_y = -kOversizedCommand,
         command_z = kOversizedCommand;

  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, true,
      {command_x, command_y, command_z, history_}, 0.0);

  EXPECT_NEAR(command_x, kVelocityLimit, kTol);
  EXPECT_NEAR(command_y, -kVelocityLimit, kTol);
  EXPECT_NEAR(command_z, kVelocityLimit, kTol);
  expectHistory(Eigen::Vector3d::Zero(),
                Eigen::Vector3d(kVelocityLimit, -kVelocityLimit, kVelocityLimit));
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsNegativeAssertVelocityClamped) {
  auto linear_x_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, true,
      {command_x, command_y, command_z, history_}, -0.01);

  EXPECT_NEAR(command_x, kVelocityLimit, kTol);
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsNegativeWithAccelerationLimitsAssertPreviousHeld) {
  auto linear_x_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, true,
      {command_x, command_y, command_z, history_}, -0.01);

  EXPECT_NEAR(command_x, 0.0, kTol);
  EXPECT_NEAR(command_y, 0.0, kTol);
  EXPECT_NEAR(command_z, 0.0, kTol);
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsPositiveAssertVelocityClampedThroughFullLimiter) {
  auto linear_x_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, true,
      {command_x, command_y, command_z, history_}, 0.01);

  EXPECT_NEAR(command_x, kVelocityLimit, kTol);
}

// ===========================================================================
// Acceleration limiting needs a valid time step
// ===========================================================================

TEST_F(SwerveCommandLimitsTest, WhenDtIsPositiveAssertAccelerationLimited) {
  auto linear_x_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, true,
      {command_x, command_y, command_z, history_}, kUnitStep);

  EXPECT_NEAR(command_x, kAccelerationLimit, kTol);
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsZeroWithAccelerationOnlyLimitsAssertPreviousHeld) {
  auto linear_x_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, true,
      {command_x, command_y, command_z, history_}, 0.0);

  EXPECT_NEAR(command_x, 0.0, kTol);
  expectHistory(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsZeroWithNonzeroHistoryAssertPreviousHeld) {
  auto linear_x_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  history_[1] = Eigen::Vector3d(0.5, -0.5, 0.25);
  double command_x = kOversizedCommand, command_y = -kOversizedCommand,
         command_z = kOversizedCommand;

  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, true,
      {command_x, command_y, command_z, history_}, 0.0);

  EXPECT_NEAR(command_x, 0.5, kTol);
  EXPECT_NEAR(command_y, -0.5, kTol);
  EXPECT_NEAR(command_z, 0.25, kTol);
}

// ===========================================================================
// History convention: [0] is second-to-last, [1] is last
// ===========================================================================

TEST_F(SwerveCommandLimitsTest, WhenLimitsDisabledAssertCommandsPassThroughAndHistoryShifts) {
  auto linear_x_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  history_[0] = Eigen::Vector3d(1.0, 2.0, 3.0);
  history_[1] = Eigen::Vector3d(4.0, 5.0, 6.0);
  double command_x = 7.0, command_y = 8.0, command_z = 9.0;

  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, false,
      {command_x, command_y, command_z, history_}, 0.5);

  EXPECT_NEAR(command_x, 7.0, kTol);
  EXPECT_NEAR(command_y, 8.0, kTol);
  EXPECT_NEAR(command_z, 9.0, kTol);
  expectHistory(Eigen::Vector3d(4.0, 5.0, 6.0), Eigen::Vector3d(7.0, 8.0, 9.0));
}

TEST_F(SwerveCommandLimitsTest, WhenLimitersAreMissingAssertCommandsPassThroughAndHistoryShifts) {
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits({nullptr, nullptr, nullptr}, true,
                                            {command_x, command_y, command_z, history_}, 0.0);

  EXPECT_NEAR(command_x, kOversizedCommand, kTol);
  expectHistory(Eigen::Vector3d::Zero(), Eigen::Vector3d(kOversizedCommand, 0.0, 0.0));
}

TEST_F(SwerveCommandLimitsTest, WhenOneLimiterIsMissingAssertNoAxisLimitedAndHistoryShifts) {
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeVelocityOnlyAxis());
  double command_x = kOversizedCommand, command_y = -kOversizedCommand,
         command_z = kOversizedCommand;

  franka_mobile::detail::applyCommandLimits({nullptr, &linear_y_limiter, &angular_z_limiter}, true,
                                            {command_x, command_y, command_z, history_}, 0.0);

  EXPECT_NEAR(command_x, kOversizedCommand, kTol);
  EXPECT_NEAR(command_y, -kOversizedCommand, kTol);
  EXPECT_NEAR(command_z, kOversizedCommand, kTol);
  expectHistory(Eigen::Vector3d::Zero(),
                Eigen::Vector3d(kOversizedCommand, -kOversizedCommand, kOversizedCommand));
}

TEST_F(SwerveCommandLimitsTest, WhenSecondUpdateRunsAssertHistoryFeedsLimiter) {
  auto linear_x_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto linear_y_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  auto angular_z_limiter = franka_mobile::detail::makeSpeedLimiter(makeAccelerationOnlyAxis());
  double command_x = kAccelerationLimit, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, true,
      {command_x, command_y, command_z, history_}, kUnitStep);
  ASSERT_NEAR(command_x, kAccelerationLimit, kTol);

  command_x = kOversizedCommand;
  franka_mobile::detail::applyCommandLimits(
      {&linear_x_limiter, &linear_y_limiter, &angular_z_limiter}, true,
      {command_x, command_y, command_z, history_}, kUnitStep);

  EXPECT_NEAR(command_x, 2.0 * kAccelerationLimit, kTol);
  expectHistory(Eigen::Vector3d(kAccelerationLimit, 0.0, 0.0),
                Eigen::Vector3d(2.0 * kAccelerationLimit, 0.0, 0.0));
}

// ===========================================================================
// Closed-loop twist: the pose differenced into the base frame
// ===========================================================================

static Eigen::Quaterniond yawRotation(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

TEST(BodyTwistTest, WhenTurnedAQuarterAndDrivingForwardAssertTwistIsForward) {
  // Heading +90°: forward in the base is +y in the odometry frame.
  const Eigen::Vector3d twist =
      franka_mobile::detail::bodyTwist(Eigen::Vector3d(1.0, 2.0, 0.0), yawRotation(M_PI_2),
                                       Eigen::Vector3d(1.0, 2.01, 0.0), yawRotation(M_PI_2), 0.02);
  EXPECT_NEAR(twist.x(), 0.5, kTol);
  EXPECT_NEAR(twist.y(), 0.0, kTol);
  EXPECT_NEAR(twist.z(), 0.0, kTol);
}

TEST(BodyTwistTest, WhenDrivingAnArcAssertTwistIsRecovered) {
  // A constant body twist v over a period T moves the base by
  // R(yaw0) * integral_0^T R(wz t) dt * v.
  const Eigen::Vector3d expected(0.3, -0.2, 1.0);
  const double yaw0 = 2.5, period = 0.02, turn = expected.z() * period;
  const Eigen::Vector2d step =
      Eigen::Rotation2Dd(yaw0) *
      Eigen::Vector2d(std::sin(turn) * expected.x() - (1.0 - std::cos(turn)) * expected.y(),
                      (1.0 - std::cos(turn)) * expected.x() + std::sin(turn) * expected.y()) /
      expected.z();
  const Eigen::Vector3d p0(-0.4, 0.7, 0.0);
  const Eigen::Vector3d twist = franka_mobile::detail::bodyTwist(
      p0, yawRotation(yaw0), p0 + Eigen::Vector3d(step.x(), step.y(), 0.0),
      yawRotation(yaw0 + turn), period);
  EXPECT_NEAR(twist.x(), expected.x(), 1e-4);
  EXPECT_NEAR(twist.y(), expected.y(), 1e-4);
  EXPECT_NEAR(twist.z(), expected.z(), kTol);
}

TEST(BodyTwistTest, WhenHeadingCrossesPiAssertTurnIsTheShortWay) {
  const Eigen::Vector3d p(0.0, 0.0, 0.0);
  const Eigen::Vector3d twist = franka_mobile::detail::bodyTwist(
      p, yawRotation(M_PI - 0.01), p, yawRotation(-M_PI + 0.01), 0.02);
  EXPECT_NEAR(twist.z(), 1.0, kTol);
}

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
