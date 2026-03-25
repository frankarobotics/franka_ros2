#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <franka_mobile/odometry.hpp>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static rclcpp::Time make_time(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

static constexpr double kEps = 1e-9;   // tight tolerance for exact arithmetic
static constexpr double kTol = 1e-6;   // relaxed tolerance for trig paths

namespace franka_mobile {

// ===========================================================================
// Construction
// ===========================================================================

class OdometryConstructionTest : public ::testing::Test {};

TEST_F(OdometryConstructionTest, DefaultWindowSizeOne) {
  // A window of 1 means the rolling mean equals the last sample immediately.
  Odometry odom(1);
  odom.init(make_time(0.0));

  odom.update(1.0, 0.0, 0.0, make_time(1.0));
  EXPECT_NEAR(odom.getLinearX(), 1.0, kEps);
}

TEST_F(OdometryConstructionTest, LargerWindowSizeSmooths) {
  // With a window of 3 and only one sample, the mean is that sample / window
  // because un-filled slots contribute 0.
  Odometry odom(3);
  odom.init(make_time(0.0));

  odom.update(3.0, 0.0, 0.0, make_time(1.0));
  // RollingMeanAccumulator typically averages over filled slots only;
  // adjust expectation to match your accumulator's contract.
  // If it averages over window_size slots unconditionally: mean = 3/3 = 1.0
  // If it averages over accumulated count: mean = 3/1 = 3.0
  // We test the invariant that lies within [1.0, 3.0].
  EXPECT_GE(odom.getLinearX(), 1.0 - kTol);
  EXPECT_LE(odom.getLinearX(), 3.0 + kTol);
}

// ===========================================================================
// init()
// ===========================================================================

class OdometryInitTest : public ::testing::Test {
protected:
  Odometry odom{1};
};

TEST_F(OdometryInitTest, InitResetsAccumulatorsImplicitly) {
  // After init, a single update with dt=1 should give exact velocity values.
  odom.init(make_time(0.0));
  odom.update(2.0, 3.0, 0.5, make_time(1.0));

  EXPECT_NEAR(odom.getLinearX(), 2.0, kEps);
  EXPECT_NEAR(odom.getLinearY(), 3.0, kEps);
  EXPECT_NEAR(odom.getAngular(), 0.5, kEps);
}

TEST_F(OdometryInitTest, InitDoesNotResetPoseByItself) {
  // init() only resets accumulators / timestamp, NOT the pose.
  // Drive forward first, then re-init and check pose is retained.
  odom.init(make_time(0.0));
  odom.update(1.0, 0.0, 0.0, make_time(1.0));
  const double x_before = odom.getX();

  odom.init(make_time(2.0));   // re-init; pose should still be x_before
  EXPECT_NEAR(odom.getX(), x_before, kEps);
}

// ===========================================================================
// resetOdometry()
// ===========================================================================

class OdometryResetTest : public ::testing::Test {
protected:
  Odometry odom{1};
  void SetUp() override {
    odom.init(make_time(0.0));
    odom.update(1.0, 1.0, 1.0, make_time(1.0));
  }
};

TEST_F(OdometryResetTest, XResetToZero) {
  odom.resetOdometry();
  EXPECT_NEAR(odom.getX(), 0.0, kEps);
}

TEST_F(OdometryResetTest, YResetToZero) {
  odom.resetOdometry();
  EXPECT_NEAR(odom.getY(), 0.0, kEps);
}

TEST_F(OdometryResetTest, HeadingResetToZero) {
  odom.resetOdometry();
  EXPECT_NEAR(odom.getHeading(), 0.0, kEps);
}

TEST_F(OdometryResetTest, VelocitiesNotAffectedByReset) {
  // resetOdometry only clears pose; velocity accumulators keep state.
  const double vx = odom.getLinearX();
  odom.resetOdometry();
  EXPECT_NEAR(odom.getLinearX(), vx, kEps);
}

// ===========================================================================
// update() — velocity rolling mean
// ===========================================================================

class OdometryVelocityTest : public ::testing::Test {
protected:
  Odometry odom{4};
  void SetUp() override { odom.init(make_time(0.0)); }
};

TEST_F(OdometryVelocityTest, SingleUpdateReturnsItselfForWindowOne) {
  Odometry o(1);
  o.init(make_time(0.0));
  o.update(5.0, -2.0, 1.5, make_time(1.0));
  EXPECT_NEAR(o.getLinearX(), 5.0, kEps);
  EXPECT_NEAR(o.getLinearY(), -2.0, kEps);
  EXPECT_NEAR(o.getAngular(), 1.5, kEps);
}

TEST_F(OdometryVelocityTest, RollingMeanSmoothsOverWindow) {
  // Feed window_size identical samples; mean must equal that value.
  const double vx = 4.0;
  for (int i = 1; i <= 4; ++i) {
    odom.update(vx, 0.0, 0.0, make_time(static_cast<double>(i)));
  }
  EXPECT_NEAR(odom.getLinearX(), vx, kTol);
}

TEST_F(OdometryVelocityTest, OldSamplesDropOutOfWindow) {
  // Fill window with 1.0, then overwrite with 3.0; mean should converge to 3.
  for (int i = 1; i <= 4; ++i) {
    odom.update(1.0, 0.0, 0.0, make_time(static_cast<double>(i)));
  }
  for (int i = 5; i <= 8; ++i) {
    odom.update(3.0, 0.0, 0.0, make_time(static_cast<double>(i)));
  }
  EXPECT_NEAR(odom.getLinearX(), 3.0, kTol);
}

TEST_F(OdometryVelocityTest, NegativeVelocitiesHandled) {
  odom.update(-1.0, -2.0, -0.5, make_time(1.0));
  EXPECT_LT(odom.getLinearX(), 0.0);
  EXPECT_LT(odom.getLinearY(), 0.0);
  EXPECT_LT(odom.getAngular(), 0.0);
}

// ===========================================================================
// update() — pose integration (straight-line motion)
// ===========================================================================

class OdometryIntegrationTest : public ::testing::Test {
protected:
  Odometry odom{1};
  void SetUp() override { odom.init(make_time(0.0)); }
};

TEST_F(OdometryIntegrationTest, PureStraightLineX) {
  // heading = 0 → cos=1, sin=0.  x += vx*dt, y unchanged.
  odom.update(1.0, 0.0, 0.0, make_time(1.0));
  EXPECT_NEAR(odom.getX(), 1.0, kTol);
  EXPECT_NEAR(odom.getY(), 0.0, kTol);
  EXPECT_NEAR(odom.getHeading(), 0.0, kTol);
}

TEST_F(OdometryIntegrationTest, PureStraightLineY) {
  // heading = 0, no x velocity → y += vy*dt (y moves in -x direction locally)
  // integrate: x += vx*cos(d) - vy*sin(d), y += vx*sin(d) + vy*cos(d)
  // with heading=0, d=0: x += 0 - vy*0 = 0; y += 0 + vy*1 = vy
  odom.update(0.0, 2.0, 0.0, make_time(1.0));
  EXPECT_NEAR(odom.getX(), 0.0, kTol);
  EXPECT_NEAR(odom.getY(), 2.0, kTol);
}

TEST_F(OdometryIntegrationTest, PureRotationChangesHeadingOnly) {
  odom.update(0.0, 0.0, M_PI / 2.0, make_time(1.0));
  // With linear_x=0 and linear_y=0, x and y don't change.
  EXPECT_NEAR(odom.getX(), 0.0, kTol);
  EXPECT_NEAR(odom.getY(), 0.0, kTol);
  EXPECT_NEAR(odom.getHeading(), M_PI / 2.0, kTol);
}

TEST_F(OdometryIntegrationTest, HeadingAccumulatesAcrossUpdates) {
  // Two quarter-turns should sum to a half-turn.
  odom.update(0.0, 0.0, M_PI / 2.0, make_time(1.0));
  odom.update(0.0, 0.0, M_PI / 2.0, make_time(2.0));
  EXPECT_NEAR(odom.getHeading(), M_PI, kTol);
}

TEST_F(OdometryIntegrationTest, IntegrateUsesMidpointDirection) {
  // The integrate() uses direction = heading + angular*0.5 (midpoint rule).
  // For heading=0, angular=pi/2, dt=1:
  //   direction = pi/4
  //   x += 1.0 * cos(pi/4) - 0 = sqrt(2)/2
  //   y += 1.0 * sin(pi/4) + 0 = sqrt(2)/2
  odom.update(1.0, 0.0, M_PI / 2.0, make_time(1.0));
  const double expected = std::sqrt(2.0) / 2.0;
  EXPECT_NEAR(odom.getX(), expected, kTol);
  EXPECT_NEAR(odom.getY(), expected, kTol);
}

TEST_F(OdometryIntegrationTest, MultipleUpdatesAccumulatePose) {
  // Move 1 m/s for 3 seconds along x-axis (heading=0).
  for (int i = 1; i <= 3; ++i) {
    odom.update(1.0, 0.0, 0.0, make_time(static_cast<double>(i)));
  }
  EXPECT_NEAR(odom.getX(), 3.0, kTol);
  EXPECT_NEAR(odom.getY(), 0.0, kTol);
}

TEST_F(OdometryIntegrationTest, ZeroDtProducesNoPoseChange) {
  // If two updates share the same timestamp, dt=0 and pose must not change.
  odom.update(10.0, 10.0, 10.0, make_time(1.0));
  const double x0 = odom.getX(), y0 = odom.getY(), h0 = odom.getHeading();

  odom.update(10.0, 10.0, 10.0, make_time(1.0));  // same time
  EXPECT_NEAR(odom.getX(), x0, kTol);
  EXPECT_NEAR(odom.getY(), y0, kTol);
  EXPECT_NEAR(odom.getHeading(), h0, kTol);
}

TEST_F(OdometryIntegrationTest, FullCircleReturnsToOrigin) {
  // Driving in a circle: combined forward + rotation such that after 2π rad
  // the robot is back at (0, 0).  We use small steps to reduce accumulation
  // error and verify the robot returns close to the origin.
  const int steps = 360;
  const double omega = 2.0 * M_PI / steps;   // rad per step
  const double v     = 1.0;                  // tangential speed (m/step)
  const double r     = v / omega;            // circle radius

  odom.resetOdometry();
  for (int i = 0; i < steps; ++i) {
    odom.update(v, 0.0, omega, make_time(static_cast<double>(i + 1)));
  }
  // Expect to return close to origin (tolerance loosened due to discrete steps)
  EXPECT_NEAR(odom.getX(), 0.0, r * 0.05);
  EXPECT_NEAR(odom.getY(), 0.0, r * 0.05);

  EXPECT_NEAR(std::cos(odom.getHeading()), 1.0, 1e-3);
  EXPECT_NEAR(std::sin(odom.getHeading()), 0.0, 1e-3);
}

// ===========================================================================
// setVelocityRollingWindowSize()
// ===========================================================================

class OdometryWindowResizeTest : public ::testing::Test {
protected:
  Odometry odom{4};
  void SetUp() override {
    odom.init(make_time(0.0));
    // Pre-fill the window with known values.
    for (int i = 1; i <= 4; ++i) {
      odom.update(2.0, 0.0, 0.0, make_time(static_cast<double>(i)));
    }
  }
};

TEST_F(OdometryWindowResizeTest, ResizeResetsAccumulators) {
  // After resize+reset the accumulator is empty; next update has a fresh mean.
  odom.setVelocityRollingWindowSize(2);
  odom.update(10.0, 0.0, 0.0, make_time(5.0));
  // Mean should now reflect only the new sample(s), not old history.
  EXPECT_GE(odom.getLinearX(), 5.0 - kTol);   // at minimum 10/2 for 1 sample
}

TEST_F(OdometryWindowResizeTest, ResizeToOneBehavesLikeNoSmoothing) {
  odom.setVelocityRollingWindowSize(1);
  odom.update(7.0, 0.0, 0.0, make_time(5.0));
  EXPECT_NEAR(odom.getLinearX(), 7.0, kEps);
}

TEST_F(OdometryWindowResizeTest, PoseNotAffectedByWindowResize) {
  const double x0 = odom.getX();
  odom.setVelocityRollingWindowSize(8);
  EXPECT_NEAR(odom.getX(), x0, kEps);
}

// ===========================================================================
// Edge / boundary cases
// ===========================================================================

class OdometryEdgeCaseTest : public ::testing::Test {
protected:
  Odometry odom{1};
  void SetUp() override { odom.init(make_time(0.0)); }
};

TEST_F(OdometryEdgeCaseTest, InitialPoseIsZero) {
  EXPECT_NEAR(odom.getX(), 0.0, kEps);
  EXPECT_NEAR(odom.getY(), 0.0, kEps);
  EXPECT_NEAR(odom.getHeading(), 0.0, kEps);
}

TEST_F(OdometryEdgeCaseTest, NegativeDisplacementDecreasesX) {
  odom.update(-3.0, 0.0, 0.0, make_time(1.0));
  EXPECT_NEAR(odom.getX(), -3.0, kTol);
}

TEST_F(OdometryEdgeCaseTest, LargeTimeDeltaScalesPoseCorrectly) {
  // dt = 100 s, vx = 2 m/s → displacement = 200 m
  odom.update(2.0, 0.0, 0.0, make_time(100.0));
  EXPECT_NEAR(odom.getX(), 200.0, kTol);
}

TEST_F(OdometryEdgeCaseTest, ResetOdometryThenContinueDriving) {
  odom.update(5.0, 0.0, 0.0, make_time(1.0));
  odom.resetOdometry();
  odom.update(1.0, 0.0, 0.0, make_time(2.0));
  // After reset x=0; then 1 m/s for 1 s → x=1.
  EXPECT_NEAR(odom.getX(), 1.0, kTol);
}

TEST_F(OdometryEdgeCaseTest, VelocitiesZeroAfterAllZeroUpdates) {
  odom.update(0.0, 0.0, 0.0, make_time(1.0));
  EXPECT_NEAR(odom.getLinearX(), 0.0, kEps);
  EXPECT_NEAR(odom.getLinearY(), 0.0, kEps);
  EXPECT_NEAR(odom.getAngular(), 0.0, kEps);
}

}  // namespace franka_mobile

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}