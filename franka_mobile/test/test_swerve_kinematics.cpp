#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <franka_mobile/swerve_kinematics.hpp>

#include <array>
#include <cmath>
#include <limits>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr double kEps = 1e-9;
static constexpr double kTol = 1e-6;

static constexpr double kL = 1.0;
static constexpr double kR = 1.0;

static franka_mobile::SwerveKinematics make_default_kinematics() {
  return franka_mobile::SwerveKinematics(
      {Eigen::Vector2d{kL, 0.0}, Eigen::Vector2d{-kL, 0.0}}, kR);
}

static std::array<double, 2> angles(double a0, double a1) { return {a0, a1}; }
static std::array<double, 2> speeds(double s0, double s1) { return {s0, s1}; }


// ===========================================================================
// Constructor validation
// ===========================================================================

class SwerveKinematicsConstructorTest : public ::testing::Test {};

TEST_F(SwerveKinematicsConstructorTest, WhenParametersValidAssertNoThrow) {
  ASSERT_NO_THROW(make_default_kinematics());
}

TEST_F(SwerveKinematicsConstructorTest, WhenRadiusTooSmallAssertThrows) {
  ASSERT_THROW(
      franka_mobile::SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}}, 1e-4),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, WhenRadiusZeroAssertThrows) {
  ASSERT_THROW(
      franka_mobile::SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}}, 0.0),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, WhenRadiusNegativeAssertThrows) {
  ASSERT_THROW(
      franka_mobile::SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}}, -0.5),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, WhenRadiusInfiniteAssertThrows) {
  ASSERT_THROW(
      franka_mobile::SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}},
                       std::numeric_limits<double>::infinity()),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, WhenRadiusNaNAssertThrows) {
  ASSERT_THROW(
      franka_mobile::SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}},
                       std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, WhenWheelPositionZeroAssertThrows) {
  ASSERT_THROW(
      franka_mobile::SwerveKinematics({Eigen::Vector2d{0, 0}, Eigen::Vector2d{-1, 0}}, kR),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, WhenBothWheelPositionsZeroAssertThrows) {
  ASSERT_THROW(
      franka_mobile::SwerveKinematics({Eigen::Vector2d{0, 0}, Eigen::Vector2d{0, 0}}, kR),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, WhenWheelPositionNearlyZeroAssertThrows) {
  ASSERT_THROW(
      franka_mobile::SwerveKinematics({Eigen::Vector2d{5e-4, 5e-4}, Eigen::Vector2d{-1, 0}}, kR),
      std::invalid_argument);
}

// ===========================================================================
// forwardKinematics — closed-form
// ===========================================================================

class SwerveForwardKinematicsTest : public ::testing::Test {
protected:
  franka_mobile::SwerveKinematics sk = make_default_kinematics();
  double vx{}, vy{}, wz{};
};

TEST_F(SwerveForwardKinematicsTest, WhenPureTranslationXAssertVelocityCorrect) {
  const bool ok = sk.forwardKinematics(angles(0, 0), speeds(1, 1), vx, vy, wz);
  ASSERT_TRUE(ok);
  ASSERT_NEAR(vx, 1.0, kTol);
  ASSERT_NEAR(vy, 0.0, kTol);
  ASSERT_NEAR(wz, 0.0, kTol);
}

TEST_F(SwerveForwardKinematicsTest, WhenPureTranslationXNegativeSpeedAssertVelocityCorrect) {
  sk.forwardKinematics(angles(0, 0), speeds(-1, -1), vx, vy, wz);
  ASSERT_NEAR(vx, -1.0, kTol);
  ASSERT_NEAR(vy, 0.0, kTol);
  ASSERT_NEAR(wz, 0.0, kTol);
}

TEST_F(SwerveForwardKinematicsTest, WhenPureTranslationYAssertVelocityCorrect) {
  sk.forwardKinematics(angles(M_PI_2, M_PI_2), speeds(1, 1), vx, vy, wz);
  ASSERT_NEAR(vx, 0.0, kTol);
  ASSERT_NEAR(vy, 1.0, kTol);
  ASSERT_NEAR(wz, 0.0, kTol);
}

TEST_F(SwerveForwardKinematicsTest, WhenPureRotationAssertAngularVelocityCorrect) {
  sk.forwardKinematics(angles(M_PI_2, -M_PI_2), speeds(1, 1), vx, vy, wz);
  ASSERT_NEAR(vx, 0.0, kTol);
  ASSERT_NEAR(vy, 0.0, kTol);
  ASSERT_NEAR(wz, 1.0, kTol);
}

TEST_F(SwerveForwardKinematicsTest, WhenPureRotationNegativeAssertAngularVelocityCorrect) {
  sk.forwardKinematics(angles(-M_PI_2, M_PI_2), speeds(1, 1), vx, vy, wz);
  ASSERT_NEAR(wz, -1.0, kTol);
}

TEST_F(SwerveForwardKinematicsTest, WhenMixedMotionAssertCorrectOutput) {
  sk.forwardKinematics(angles(M_PI / 4.0, M_PI_2), speeds(1, 1), vx, vy, wz);
  const double sq2 = std::sqrt(2.0);
  ASSERT_NEAR(vx, sq2 / 4.0, kTol);
  ASSERT_NEAR(vy, (sq2 / 2.0 + 1.0) / 2.0, kTol);
  ASSERT_NEAR(wz, (sq2 / 2.0 - 1.0) / 2.0, kTol);
}

TEST_F(SwerveForwardKinematicsTest, WhenWheelRadiusChangesAssertScaling) {
  const double r2 = 2.0;
  franka_mobile::SwerveKinematics sk2({Eigen::Vector2d{kL, 0}, Eigen::Vector2d{-kL, 0}}, r2);
  double vx2{}, vy2{}, wz2{};
  sk.forwardKinematics(angles(0, 0), speeds(1, 1), vx, vy, wz);
  sk2.forwardKinematics(angles(0, 0), speeds(1, 1), vx2, vy2, wz2);
  ASSERT_NEAR(vx2, vx * r2, kTol);
}

TEST_F(SwerveForwardKinematicsTest, WhenValidInputAssertReturnsTrue) {
  ASSERT_TRUE(sk.forwardKinematics(angles(0, 0), speeds(1, 1), vx, vy, wz));
}

// ===========================================================================
// QR
// ===========================================================================

class SwerveForwardKinematicsQrTest : public ::testing::Test {
protected:
  franka_mobile::SwerveKinematics sk = make_default_kinematics();
  double vx{}, vy{}, wz{};
};

TEST_F(SwerveForwardKinematicsQrTest, WhenPureTranslationXAssertMatchesClosedForm) {
  double vx_cf{}, vy_cf{}, wz_cf{};
  sk.forwardKinematics(angles(0, 0), speeds(1, 1), vx_cf, vy_cf, wz_cf);
  sk.forwardKinematicsQr(angles(0, 0), speeds(1, 1), vx, vy, wz);
  ASSERT_NEAR(vx, vx_cf, kTol);
  ASSERT_NEAR(vy, vy_cf, kTol);
  ASSERT_NEAR(wz, wz_cf, kTol);
}

TEST_F(SwerveForwardKinematicsQrTest, WhenPureRotationAssertMatchesClosedForm) {
  double vx_cf{}, vy_cf{}, wz_cf{};
  sk.forwardKinematics(angles(M_PI_2, -M_PI_2), speeds(1, 1), vx_cf, vy_cf, wz_cf);
  sk.forwardKinematicsQr(angles(M_PI_2, -M_PI_2), speeds(1, 1), vx, vy, wz);
  ASSERT_NEAR(wz, wz_cf, kTol);
}

TEST_F(SwerveForwardKinematicsQrTest, WhenMixedMotionAssertMatchesClosedForm) {
  double vx_cf{}, vy_cf{}, wz_cf{};
  sk.forwardKinematics(angles(M_PI / 4.0, M_PI_2), speeds(1, 1), vx_cf, vy_cf, wz_cf);
  sk.forwardKinematicsQr(angles(M_PI / 4.0, M_PI_2), speeds(1, 1), vx, vy, wz);
  ASSERT_NEAR(vx, vx_cf, kTol);
}

TEST_F(SwerveForwardKinematicsQrTest, WhenZeroInputAssertZeroOutput) {
  sk.forwardKinematicsQr(angles(0, 0), speeds(0, 0), vx, vy, wz);
  ASSERT_NEAR(vx, 0.0, kTol);
  ASSERT_NEAR(vy, 0.0, kTol);
  ASSERT_NEAR(wz, 0.0, kTol);
}

TEST_F(SwerveForwardKinematicsQrTest, WhenValidInputAssertReturnsTrue) {
  ASSERT_TRUE(sk.forwardKinematicsQr(angles(0, 0), speeds(1, 1), vx, vy, wz));
}

// ===========================================================================
// inverse validation
// ===========================================================================

class SwerveInverseKinematicsValidationTest : public ::testing::Test {
protected:
  franka_mobile::SwerveKinematics sk = make_default_kinematics();
  std::array<double, 2> sa{}, ws{};
};

TEST_F(SwerveInverseKinematicsValidationTest, WhenVxInfiniteAssertFalse) {
  ASSERT_FALSE(sk.inverseKinematics(std::numeric_limits<double>::infinity(), 0, 0, sa, ws));
}

TEST_F(SwerveInverseKinematicsValidationTest, WhenVyInfiniteAssertFalse) {
  ASSERT_FALSE(sk.inverseKinematics(0, std::numeric_limits<double>::infinity(), 0, sa, ws));
}

TEST_F(SwerveInverseKinematicsValidationTest, WhenWzInfiniteAssertFalse) {
  ASSERT_FALSE(sk.inverseKinematics(0, 0, std::numeric_limits<double>::infinity(), sa, ws));
}

TEST_F(SwerveInverseKinematicsValidationTest, WhenVxNaNAssertFalse) {
  ASSERT_FALSE(sk.inverseKinematics(std::numeric_limits<double>::quiet_NaN(), 0, 0, sa, ws));
}

TEST_F(SwerveInverseKinematicsValidationTest, WhenZeroInputAssertTrue) {
  ASSERT_TRUE(sk.inverseKinematics(0, 0, 0, sa, ws));
}

// ===========================================================================
// inverse correctness
// ===========================================================================

class SwerveInverseKinematicsTest : public ::testing::Test {
protected:
  franka_mobile::SwerveKinematics sk = make_default_kinematics();
  std::array<double, 2> sa{0, 0}, ws{0, 0};
};

TEST_F(SwerveInverseKinematicsTest, WhenPureTranslationXAssertWheelsPointForward) {
  sk.inverseKinematics(1.0, 0.0, 0.0, sa, ws);
  ASSERT_NEAR(sa[0], 0.0, kTol);
  ASSERT_NEAR(sa[1], 0.0, kTol);
}

TEST_F(SwerveInverseKinematicsTest, WhenPureTranslationXAssertEqualSpeeds) {
  sk.inverseKinematics(1.0, 0.0, 0.0, sa, ws);
  ASSERT_NEAR(ws[0], ws[1], kTol);
}

TEST_F(SwerveInverseKinematicsTest, WhenPureTranslationXAssertCorrectSpeed) {
  sk.inverseKinematics(1.0, 0.0, 0.0, sa, ws);
  ASSERT_NEAR(ws[0], 1.0, kTol);
}

TEST_F(SwerveInverseKinematicsTest, WhenPureTranslationYAssertWheelsPointSideways) {
  sk.inverseKinematics(0.0, 1.0, 0.0, sa, ws);
  ASSERT_NEAR(sa[0], M_PI_2, kTol);
}

TEST_F(SwerveInverseKinematicsTest, WhenPureRotationAssertEqualSpeeds) {
  sk.inverseKinematics(0.0, 0.0, 1.0, sa, ws);
  ASSERT_NEAR(std::fabs(ws[0]), std::fabs(ws[1]), kTol);
}

TEST_F(SwerveInverseKinematicsTest, WhenPureRotationAssertCorrectSpeed) {
  sk.inverseKinematics(0.0, 0.0, 1.0, sa, ws);
  ASSERT_NEAR(ws[0], 1.0, kTol);
}

TEST_F(SwerveInverseKinematicsTest, WhenZeroInputAssertSpeedsZero) {
  sk.inverseKinematics(0.0, 0.0, 0.0, sa, ws);
  ASSERT_NEAR(ws[0], 0.0, kTol);
}

TEST_F(SwerveInverseKinematicsTest, WhenAngleFlipNeededAssertSpeedNegated) {
  sk.inverseKinematics(1.0, 0.0, 0.0, sa, ws);

  std::array<double, 2> sa2{sa}, ws2{ws};
  sk.inverseKinematics(-1.0, 0.0, 0.0, sa2, ws2);

  ASSERT_LE(std::fabs(sa2[0] - sa[0]), M_PI_2 + kTol);
  ASSERT_LT(ws2[0], 0.0);
}

// ===========================================================================
// Round trip
// ===========================================================================

class SwerveRoundTripTest : public ::testing::Test {
protected:
  franka_mobile::SwerveKinematics sk = make_default_kinematics();

  void check_round_trip(double cmd_vx, double cmd_vy, double cmd_wz) {
    std::array<double, 2> sa{0, 0}, ws{0, 0};
    ASSERT_TRUE(sk.inverseKinematics(cmd_vx, cmd_vy, cmd_wz, sa, ws));

    double vx{}, vy{}, wz{};
    ASSERT_TRUE(sk.forwardKinematics(sa, ws, vx, vy, wz));

    ASSERT_NEAR(vx, cmd_vx, kTol);
    ASSERT_NEAR(vy, cmd_vy, kTol);
    ASSERT_NEAR(wz, cmd_wz, kTol);
  }
};

TEST_F(SwerveRoundTripTest, WhenRoundTripPureTranslationXAssertConsistency) { check_round_trip(1.0, 0.0, 0.0); }
TEST_F(SwerveRoundTripTest, WhenRoundTripPureTranslationYAssertConsistency) { check_round_trip(0.0, 1.0, 0.0); }
TEST_F(SwerveRoundTripTest, WhenRoundTripPureRotationAssertConsistency) { check_round_trip(0.0, 0.0, 1.0); }
TEST_F(SwerveRoundTripTest, WhenRoundTripDiagonalMotionAssertConsistency) { check_round_trip(1.0, 1.0, 0.0); }
TEST_F(SwerveRoundTripTest, WhenRoundTripMixedMotionAssertConsistency) { check_round_trip(0.5, 0.3, 0.2); }
TEST_F(SwerveRoundTripTest, WhenRoundTripNegativeVelocitiesAssertConsistency) { check_round_trip(-1.0, -0.5, -0.3); }

// QR

class SwerveRoundTripQrTest : public ::testing::Test {
protected:
  franka_mobile::SwerveKinematics sk = make_default_kinematics();

  void check_round_trip_qr(double cmd_vx, double cmd_vy, double cmd_wz) {
    std::array<double, 2> sa{0, 0}, ws{0, 0};
    ASSERT_TRUE(sk.inverseKinematics(cmd_vx, cmd_vy, cmd_wz, sa, ws));

    double vx{}, vy{}, wz{};
    ASSERT_TRUE(sk.forwardKinematicsQr(sa, ws, vx, vy, wz));

    ASSERT_NEAR(vx, cmd_vx, kTol);
    ASSERT_NEAR(vy, cmd_vy, kTol);
    ASSERT_NEAR(wz, cmd_wz, kTol);
  }
};

TEST_F(SwerveRoundTripQrTest, WhenRoundTripQrPureTranslationXAssertConsistency) { check_round_trip_qr(1.0, 0.0, 0.0); }
TEST_F(SwerveRoundTripQrTest, WhenRoundTripQrPureRotationAssertConsistency) { check_round_trip_qr(0.0, 0.0, 1.0); }
TEST_F(SwerveRoundTripQrTest, WhenRoundTripQrMixedMotionAssertConsistency) { check_round_trip_qr(0.5, 0.3, 0.2); }

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
