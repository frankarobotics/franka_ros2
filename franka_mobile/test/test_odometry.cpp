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

static constexpr double kEps = 1e-9;
static constexpr double kTol = 1e-6;


// ===========================================================================
// Construction
// ===========================================================================

class OdometryConstructionTest : public ::testing::Test {};

TEST_F(OdometryConstructionTest, WhenWindowSizeIsOneAssertNoSmoothing) {
  franka_mobile::Odometry odom(1);
  odom.init(make_time(0.0));

  odom.update(1.0, 0.0, 0.0, make_time(1.0));
  ASSERT_NEAR(odom.getLinearX(), 1.0, kEps);
}

TEST_F(OdometryConstructionTest, WhenWindowSizeIsGreaterThanOneAssertSmoothingApplied) {
  franka_mobile::Odometry odom(3);
  odom.init(make_time(0.0));

  odom.update(3.0, 0.0, 0.0, make_time(1.0));

  ASSERT_GE(odom.getLinearX(), 1.0 - kTol);
  ASSERT_LE(odom.getLinearX(), 3.0 + kTol);
}

// ===========================================================================
// init()
// ===========================================================================

class OdometryInitTest : public ::testing::Test {
protected:
  franka_mobile::Odometry odom{1};
};

TEST_F(OdometryInitTest, WhenInitCalledAssertAccumulatorsReset) {
  odom.init(make_time(0.0));
  odom.update(2.0, 3.0, 0.5, make_time(1.0));

  ASSERT_NEAR(odom.getLinearX(), 2.0, kEps);
  ASSERT_NEAR(odom.getLinearY(), 3.0, kEps);
  ASSERT_NEAR(odom.getAngular(), 0.5, kEps);
}

TEST_F(OdometryInitTest, WhenInitCalledAssertPoseNotReset) {
  odom.init(make_time(0.0));
  odom.update(1.0, 0.0, 0.0, make_time(1.0));
  const double x_before = odom.getX();

  odom.init(make_time(2.0));
  ASSERT_NEAR(odom.getX(), x_before, kEps);
}

// ===========================================================================
// resetOdometry()
// ===========================================================================

class OdometryResetTest : public ::testing::Test {
protected:
  franka_mobile::Odometry odom{1};
  void SetUp() override {
    odom.init(make_time(0.0));
    odom.update(1.0, 1.0, 1.0, make_time(1.0));
  }
};

TEST_F(OdometryResetTest, WhenResetOdometryCalledAssertXIsZero) {
  odom.resetOdometry();
  ASSERT_NEAR(odom.getX(), 0.0, kEps);
}

TEST_F(OdometryResetTest, WhenResetOdometryCalledAssertYIsZero) {
  odom.resetOdometry();
  ASSERT_NEAR(odom.getY(), 0.0, kEps);
}

TEST_F(OdometryResetTest, WhenResetOdometryCalledAssertHeadingIsZero) {
  odom.resetOdometry();
  ASSERT_NEAR(odom.getHeading(), 0.0, kEps);
}

TEST_F(OdometryResetTest, WhenResetOdometryCalledAssertVelocitiesUnchanged) {
  const double vx = odom.getLinearX();
  odom.resetOdometry();
  ASSERT_NEAR(odom.getLinearX(), vx, kEps);
}

// ===========================================================================
// update() — velocity rolling mean
// ===========================================================================

class OdometryVelocityTest : public ::testing::Test {
protected:
  franka_mobile::Odometry odom{4};
  void SetUp() override { odom.init(make_time(0.0)); }
};

TEST_F(OdometryVelocityTest, WhenSingleUpdateWithWindowOneAssertVelocityEqualsInput) {
  franka_mobile::Odometry o(1);
  o.init(make_time(0.0));
  o.update(5.0, -2.0, 1.5, make_time(1.0));
  ASSERT_NEAR(o.getLinearX(), 5.0, kEps);
  ASSERT_NEAR(o.getLinearY(), -2.0, kEps);
  ASSERT_NEAR(o.getAngular(), 1.5, kEps);
}

TEST_F(OdometryVelocityTest, WhenMultipleUpdatesWithinWindowAssertRollingMeanComputed) {
  const double vx = 4.0;
  for (int i = 1; i <= 4; ++i) {
    odom.update(vx, 0.0, 0.0, make_time(static_cast<double>(i)));
  }
  ASSERT_NEAR(odom.getLinearX(), vx, kTol);
}

TEST_F(OdometryVelocityTest, WhenWindowExceededAssertOldSamplesDropped) {
  for (int i = 1; i <= 4; ++i) {
    odom.update(1.0, 0.0, 0.0, make_time(static_cast<double>(i)));
  }
  for (int i = 5; i <= 8; ++i) {
    odom.update(3.0, 0.0, 0.0, make_time(static_cast<double>(i)));
  }
  ASSERT_NEAR(odom.getLinearX(), 3.0, kTol);
}

TEST_F(OdometryVelocityTest, WhenNegativeVelocitiesProvidedAssertHandledCorrectly) {
  odom.update(-1.0, -2.0, -0.5, make_time(1.0));
  ASSERT_LT(odom.getLinearX(), 0.0);
  ASSERT_LT(odom.getLinearY(), 0.0);
  ASSERT_LT(odom.getAngular(), 0.0);
}

// ===========================================================================
// update() — pose integration
// ===========================================================================

class OdometryIntegrationTest : public ::testing::Test {
protected:
  franka_mobile::Odometry odom{1};
  void SetUp() override { odom.init(make_time(0.0)); }
};

TEST_F(OdometryIntegrationTest, WhenMovingStraightAlongXAssertXIncreasesOnly) {
  odom.update(1.0, 0.0, 0.0, make_time(1.0));
  ASSERT_NEAR(odom.getX(), 1.0, kTol);
  ASSERT_NEAR(odom.getY(), 0.0, kTol);
  ASSERT_NEAR(odom.getHeading(), 0.0, kTol);
}

TEST_F(OdometryIntegrationTest, WhenMovingStraightAlongYAssertYIncreasesOnly) {
  odom.update(0.0, 2.0, 0.0, make_time(1.0));
  ASSERT_NEAR(odom.getX(), 0.0, kTol);
  ASSERT_NEAR(odom.getY(), 2.0, kTol);
}

TEST_F(OdometryIntegrationTest, WhenOnlyRotatingAssertOnlyHeadingChanges) {
  odom.update(0.0, 0.0, M_PI / 2.0, make_time(1.0));
  ASSERT_NEAR(odom.getX(), 0.0, kTol);
  ASSERT_NEAR(odom.getY(), 0.0, kTol);
  ASSERT_NEAR(odom.getHeading(), M_PI / 2.0, kTol);
}

TEST_F(OdometryIntegrationTest, WhenMultipleRotationsAssertHeadingAccumulates) {
  odom.update(0.0, 0.0, M_PI / 2.0, make_time(1.0));
  odom.update(0.0, 0.0, M_PI / 2.0, make_time(2.0));
  ASSERT_NEAR(odom.getHeading(), M_PI, kTol);
}

TEST_F(OdometryIntegrationTest, WhenRotatingAndMovingAssertMidpointIntegrationUsed) {
  odom.update(1.0, 0.0, M_PI / 2.0, make_time(1.0));
  const double expected = std::sqrt(2.0) / 2.0;
  ASSERT_NEAR(odom.getX(), expected, kTol);
  ASSERT_NEAR(odom.getY(), expected, kTol);
}

TEST_F(OdometryIntegrationTest, WhenMultipleUpdatesAssertPoseAccumulates) {
  for (int i = 1; i <= 3; ++i) {
    odom.update(1.0, 0.0, 0.0, make_time(static_cast<double>(i)));
  }
  ASSERT_NEAR(odom.getX(), 3.0, kTol);
  ASSERT_NEAR(odom.getY(), 0.0, kTol);
}

TEST_F(OdometryIntegrationTest, WhenDeltaTimeIsZeroAssertNoPoseChange) {
  odom.update(10.0, 10.0, 10.0, make_time(1.0));
  const double x0 = odom.getX(), y0 = odom.getY(), h0 = odom.getHeading();

  odom.update(10.0, 10.0, 10.0, make_time(1.0));
  ASSERT_NEAR(odom.getX(), x0, kTol);
  ASSERT_NEAR(odom.getY(), y0, kTol);
  ASSERT_NEAR(odom.getHeading(), h0, kTol);
}

TEST_F(OdometryIntegrationTest, WhenDrivingFullCircleAssertReturnToOrigin) {
  const int steps = 360;
  const double omega = 2.0 * M_PI / steps;
  const double v     = 1.0;
  const double r     = v / omega;

  odom.resetOdometry();
  for (int i = 0; i < steps; ++i) {
    odom.update(v, 0.0, omega, make_time(static_cast<double>(i + 1)));
  }

  ASSERT_NEAR(odom.getX(), 0.0, r * 0.05);
  ASSERT_NEAR(odom.getY(), 0.0, r * 0.05);

  ASSERT_NEAR(std::cos(odom.getHeading()), 1.0, 1e-3);
  ASSERT_NEAR(std::sin(odom.getHeading()), 0.0, 1e-3);
}

// ===========================================================================
// setVelocityRollingWindowSize()
// ===========================================================================

class OdometryWindowResizeTest : public ::testing::Test {
protected:
  franka_mobile::Odometry odom{4};
  void SetUp() override {
    odom.init(make_time(0.0));
    for (int i = 1; i <= 4; ++i) {
      odom.update(2.0, 0.0, 0.0, make_time(static_cast<double>(i)));
    }
  }
};

TEST_F(OdometryWindowResizeTest, WhenWindowSizeChangedAssertAccumulatorsReset) {
  odom.setVelocityRollingWindowSize(2);
  odom.update(10.0, 0.0, 0.0, make_time(5.0));
  ASSERT_GE(odom.getLinearX(), 5.0 - kTol);
}

TEST_F(OdometryWindowResizeTest, WhenWindowSizeSetToOneAssertNoSmoothing) {
  odom.setVelocityRollingWindowSize(1);
  odom.update(7.0, 0.0, 0.0, make_time(5.0));
  ASSERT_NEAR(odom.getLinearX(), 7.0, kEps);
}

TEST_F(OdometryWindowResizeTest, WhenWindowSizeChangedAssertPoseUnaffected) {
  const double x0 = odom.getX();
  odom.setVelocityRollingWindowSize(8);
  ASSERT_NEAR(odom.getX(), x0, kEps);
}

// ===========================================================================
// Edge / boundary cases
// ===========================================================================

class OdometryEdgeCaseTest : public ::testing::Test {
protected:
  franka_mobile::Odometry odom{1};
  void SetUp() override { odom.init(make_time(0.0)); }
};

TEST_F(OdometryEdgeCaseTest, WhenInitializedAssertPoseIsZero) {
  ASSERT_NEAR(odom.getX(), 0.0, kEps);
  ASSERT_NEAR(odom.getY(), 0.0, kEps);
  ASSERT_NEAR(odom.getHeading(), 0.0, kEps);
}

TEST_F(OdometryEdgeCaseTest, WhenNegativeVelocityAppliedAssertXDecreases) {
  odom.update(-3.0, 0.0, 0.0, make_time(1.0));
  ASSERT_NEAR(odom.getX(), -3.0, kTol);
}

TEST_F(OdometryEdgeCaseTest, WhenLargeDeltaTimeAssertPoseScalesCorrectly) {
  odom.update(2.0, 0.0, 0.0, make_time(100.0));
  ASSERT_NEAR(odom.getX(), 200.0, kTol);
}

TEST_F(OdometryEdgeCaseTest, WhenResetThenUpdateAssertMotionFromZero) {
  odom.update(5.0, 0.0, 0.0, make_time(1.0));
  odom.resetOdometry();
  odom.update(1.0, 0.0, 0.0, make_time(2.0));
  ASSERT_NEAR(odom.getX(), 1.0, kTol);
}

TEST_F(OdometryEdgeCaseTest, WhenAllVelocitiesZeroAssertReportedZero) {
  odom.update(0.0, 0.0, 0.0, make_time(1.0));
  ASSERT_NEAR(odom.getLinearX(), 0.0, kEps);
  ASSERT_NEAR(odom.getLinearY(), 0.0, kEps);
  ASSERT_NEAR(odom.getAngular(), 0.0, kEps);
}

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
