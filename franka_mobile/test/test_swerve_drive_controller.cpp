#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <franka_mobile/swerve_drive_controller.hpp>

#include <array>
#include <limits>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr double kTol = 1e-9;
static constexpr double kVelocityLimit = 2.0;
static constexpr double kAccelerationLimit = 1.0;
static constexpr double kOversizedCommand = 5.0;
static constexpr double kUnitStep = 1.0;
static const double kNaN = std::numeric_limits<double>::quiet_NaN();

static diff_drive_controller::SpeedLimiter makeVelocityOnlyLimiter() {
  return diff_drive_controller::SpeedLimiter(-kVelocityLimit, kVelocityLimit, kNaN, kNaN, kNaN,
                                             kNaN, kNaN, kNaN);
}

static diff_drive_controller::SpeedLimiter makeAccelerationOnlyLimiter() {
  return diff_drive_controller::SpeedLimiter(kNaN, kNaN, kNaN, kAccelerationLimit, kNaN, kNaN, kNaN,
                                             kNaN);
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
  auto linear_x_limiter = makeVelocityOnlyLimiter();
  auto linear_y_limiter = makeVelocityOnlyLimiter();
  auto angular_z_limiter = makeVelocityOnlyLimiter();
  double command_x = kOversizedCommand, command_y = -kOversizedCommand,
         command_z = kOversizedCommand;

  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, true, command_x, command_y,
                                            command_z, history_, 0.0);

  EXPECT_NEAR(command_x, kVelocityLimit, kTol);
  EXPECT_NEAR(command_y, -kVelocityLimit, kTol);
  EXPECT_NEAR(command_z, kVelocityLimit, kTol);
  expectHistory(Eigen::Vector3d::Zero(),
                Eigen::Vector3d(kVelocityLimit, -kVelocityLimit, kVelocityLimit));
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsNegativeAssertVelocityClamped) {
  auto linear_x_limiter = makeVelocityOnlyLimiter();
  auto linear_y_limiter = makeVelocityOnlyLimiter();
  auto angular_z_limiter = makeVelocityOnlyLimiter();
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, true, command_x, command_y,
                                            command_z, history_, -0.01);

  EXPECT_NEAR(command_x, kVelocityLimit, kTol);
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsNegativeWithAccelerationLimitsAssertPreviousHeld) {
  auto linear_x_limiter = makeAccelerationOnlyLimiter();
  auto linear_y_limiter = makeAccelerationOnlyLimiter();
  auto angular_z_limiter = makeAccelerationOnlyLimiter();
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, true, command_x, command_y,
                                            command_z, history_, -0.01);

  EXPECT_NEAR(command_x, 0.0, kTol);
  EXPECT_NEAR(command_y, 0.0, kTol);
  EXPECT_NEAR(command_z, 0.0, kTol);
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsPositiveAssertVelocityClampedThroughFullLimiter) {
  auto linear_x_limiter = makeVelocityOnlyLimiter();
  auto linear_y_limiter = makeVelocityOnlyLimiter();
  auto angular_z_limiter = makeVelocityOnlyLimiter();
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, true, command_x, command_y,
                                            command_z, history_, 0.01);

  EXPECT_NEAR(command_x, kVelocityLimit, kTol);
}

// ===========================================================================
// Acceleration limiting needs a valid time step
// ===========================================================================

TEST_F(SwerveCommandLimitsTest, WhenDtIsPositiveAssertAccelerationLimited) {
  auto linear_x_limiter = makeAccelerationOnlyLimiter();
  auto linear_y_limiter = makeAccelerationOnlyLimiter();
  auto angular_z_limiter = makeAccelerationOnlyLimiter();
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, true, command_x, command_y,
                                            command_z, history_, kUnitStep);

  EXPECT_NEAR(command_x, kAccelerationLimit, kTol);
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsZeroWithAccelerationOnlyLimitsAssertPreviousHeld) {
  auto linear_x_limiter = makeAccelerationOnlyLimiter();
  auto linear_y_limiter = makeAccelerationOnlyLimiter();
  auto angular_z_limiter = makeAccelerationOnlyLimiter();
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, true, command_x, command_y,
                                            command_z, history_, 0.0);

  EXPECT_NEAR(command_x, 0.0, kTol);
  expectHistory(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

TEST_F(SwerveCommandLimitsTest, WhenDtIsZeroWithNonzeroHistoryAssertPreviousHeld) {
  auto linear_x_limiter = makeAccelerationOnlyLimiter();
  auto linear_y_limiter = makeAccelerationOnlyLimiter();
  auto angular_z_limiter = makeAccelerationOnlyLimiter();
  history_[1] = Eigen::Vector3d(0.5, -0.5, 0.25);
  double command_x = kOversizedCommand, command_y = -kOversizedCommand,
         command_z = kOversizedCommand;

  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, true, command_x, command_y,
                                            command_z, history_, 0.0);

  EXPECT_NEAR(command_x, 0.5, kTol);
  EXPECT_NEAR(command_y, -0.5, kTol);
  EXPECT_NEAR(command_z, 0.25, kTol);
}

// ===========================================================================
// History convention: [0] is second-to-last, [1] is last
// ===========================================================================

TEST_F(SwerveCommandLimitsTest, WhenLimitsDisabledAssertCommandsPassThroughAndHistoryShifts) {
  auto linear_x_limiter = makeVelocityOnlyLimiter();
  auto linear_y_limiter = makeVelocityOnlyLimiter();
  auto angular_z_limiter = makeVelocityOnlyLimiter();
  history_[0] = Eigen::Vector3d(1.0, 2.0, 3.0);
  history_[1] = Eigen::Vector3d(4.0, 5.0, 6.0);
  double command_x = 7.0, command_y = 8.0, command_z = 9.0;

  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, false, command_x, command_y,
                                            command_z, history_, 0.5);

  EXPECT_NEAR(command_x, 7.0, kTol);
  EXPECT_NEAR(command_y, 8.0, kTol);
  EXPECT_NEAR(command_z, 9.0, kTol);
  expectHistory(Eigen::Vector3d(4.0, 5.0, 6.0), Eigen::Vector3d(7.0, 8.0, 9.0));
}

TEST_F(SwerveCommandLimitsTest, WhenLimitersAreMissingAssertCommandsPassThroughAndHistoryShifts) {
  double command_x = kOversizedCommand, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(nullptr, nullptr, nullptr, true, command_x, command_y,
                                            command_z, history_, 0.0);

  EXPECT_NEAR(command_x, kOversizedCommand, kTol);
  expectHistory(Eigen::Vector3d::Zero(), Eigen::Vector3d(kOversizedCommand, 0.0, 0.0));
}

TEST_F(SwerveCommandLimitsTest, WhenOneLimiterIsMissingAssertNoAxisLimitedAndHistoryShifts) {
  auto linear_y_limiter = makeVelocityOnlyLimiter();
  auto angular_z_limiter = makeVelocityOnlyLimiter();
  double command_x = kOversizedCommand, command_y = -kOversizedCommand,
         command_z = kOversizedCommand;

  franka_mobile::detail::applyCommandLimits(nullptr, &linear_y_limiter, &angular_z_limiter, true,
                                            command_x, command_y, command_z, history_, 0.0);

  EXPECT_NEAR(command_x, kOversizedCommand, kTol);
  EXPECT_NEAR(command_y, -kOversizedCommand, kTol);
  EXPECT_NEAR(command_z, kOversizedCommand, kTol);
  expectHistory(Eigen::Vector3d::Zero(),
                Eigen::Vector3d(kOversizedCommand, -kOversizedCommand, kOversizedCommand));
}

TEST_F(SwerveCommandLimitsTest, WhenSecondUpdateRunsAssertHistoryFeedsLimiter) {
  auto linear_x_limiter = makeAccelerationOnlyLimiter();
  auto linear_y_limiter = makeAccelerationOnlyLimiter();
  auto angular_z_limiter = makeAccelerationOnlyLimiter();
  double command_x = kAccelerationLimit, command_y = 0.0, command_z = 0.0;

  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, true, command_x, command_y,
                                            command_z, history_, kUnitStep);
  ASSERT_NEAR(command_x, kAccelerationLimit, kTol);

  command_x = kOversizedCommand;
  franka_mobile::detail::applyCommandLimits(&linear_x_limiter, &linear_y_limiter,
                                            &angular_z_limiter, true, command_x, command_y,
                                            command_z, history_, kUnitStep);

  EXPECT_NEAR(command_x, 2.0 * kAccelerationLimit, kTol);
  expectHistory(Eigen::Vector3d(kAccelerationLimit, 0.0, 0.0),
                Eigen::Vector3d(2.0 * kAccelerationLimit, 0.0, 0.0));
}

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
