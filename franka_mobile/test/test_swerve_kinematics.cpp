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

// Standard symmetric wheel layout: wheels at (+L, 0) and (-L, 0).
// With wheel_radius = 1.0 the math stays clean.
static constexpr double kL = 1.0;
static constexpr double kR = 1.0;

static franka_mobile::SwerveKinematics make_default_kinematics() {
  return franka_mobile::SwerveKinematics(
      {Eigen::Vector2d{kL, 0.0}, Eigen::Vector2d{-kL, 0.0}}, kR);
}

// Build steering_angles and wheel_speeds arrays from per-wheel pairs.
static std::array<double, 2> angles(double a0, double a1) { return {a0, a1}; }
static std::array<double, 2> speeds(double s0, double s1) { return {s0, s1}; }

namespace franka_mobile {

// ===========================================================================
// Constructor validation
// ===========================================================================

class SwerveKinematicsConstructorTest : public ::testing::Test {};

TEST_F(SwerveKinematicsConstructorTest, ValidParametersDoNotThrow) {
  EXPECT_NO_THROW(make_default_kinematics());
}

TEST_F(SwerveKinematicsConstructorTest, SmallPositiveRadiusThrows) {
  EXPECT_THROW(
      SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}}, 1e-4),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, ZeroRadiusThrows) {
  EXPECT_THROW(
      SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}}, 0.0),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, NegativeRadiusThrows) {
  EXPECT_THROW(
      SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}}, -0.5),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, InfiniteRadiusThrows) {
  EXPECT_THROW(
      SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}},
                       std::numeric_limits<double>::infinity()),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, NanRadiusThrows) {
  EXPECT_THROW(
      SwerveKinematics({Eigen::Vector2d{1, 0}, Eigen::Vector2d{-1, 0}},
                       std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, ZeroWheelPositionThrows) {
  EXPECT_THROW(
      SwerveKinematics({Eigen::Vector2d{0, 0}, Eigen::Vector2d{-1, 0}}, kR),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, BothZeroWheelPositionsThrows) {
  EXPECT_THROW(
      SwerveKinematics({Eigen::Vector2d{0, 0}, Eigen::Vector2d{0, 0}}, kR),
      std::invalid_argument);
}

TEST_F(SwerveKinematicsConstructorTest, NearlyZeroWheelPositionThrows) {
  // Below the 1e-3 norm threshold.
  EXPECT_THROW(
      SwerveKinematics({Eigen::Vector2d{5e-4, 5e-4}, Eigen::Vector2d{-1, 0}}, kR),
      std::invalid_argument);
}

// ===========================================================================
// forwardKinematics — closed-form
// ===========================================================================

class SwerveForwardKinematicsTest : public ::testing::Test {
protected:
  SwerveKinematics sk = make_default_kinematics();
  double vx{}, vy{}, wz{};
};

// --- Pure translation along X ------------------------------------------

TEST_F(SwerveForwardKinematicsTest, PureTranslationX_VelocityCorrect) {
  // Both wheels pointing at 0 rad, speed = 1/R → wheel linear speed = 1 m/s
  const bool ok = sk.forwardKinematics(angles(0, 0), speeds(1, 1), vx, vy, wz);
  ASSERT_TRUE(ok);
  EXPECT_NEAR(vx, 1.0, kTol);
  EXPECT_NEAR(vy, 0.0, kTol);
  EXPECT_NEAR(wz, 0.0, kTol);
}

TEST_F(SwerveForwardKinematicsTest, PureTranslationX_NegativeSpeed) {
  sk.forwardKinematics(angles(0, 0), speeds(-1, -1), vx, vy, wz);
  EXPECT_NEAR(vx, -1.0, kTol);
  EXPECT_NEAR(vy, 0.0, kTol);
  EXPECT_NEAR(wz, 0.0, kTol);
}

// --- Pure translation along Y ------------------------------------------

TEST_F(SwerveForwardKinematicsTest, PureTranslationY_VelocityCorrect) {
  // Both wheels at pi/2, speed = 1 → vy = 1
  sk.forwardKinematics(angles(M_PI_2, M_PI_2), speeds(1, 1), vx, vy, wz);
  EXPECT_NEAR(vx, 0.0, kTol);
  EXPECT_NEAR(vy, 1.0, kTol);
  EXPECT_NEAR(wz, 0.0, kTol);
}

// --- Pure rotation ------------------------------------------------------
// Wheels at (+1,0) and (-1,0).
// For wz=1: wheel 0 tangent at pi/2 (speed=1), wheel 1 tangent at -pi/2 (speed=1).
// cross2d(v,r): v.y*r.x - v.x*r.y
//   wheel0: 1*1 - 0*0 = 1 ;  wheel1: (-1)*(-1) - 0*0 = 1
// norm_sq = 1+1 = 2  →  wz = 2/2 = 1 ✓

TEST_F(SwerveForwardKinematicsTest, PureRotation_AngularVelocityCorrect) {
  sk.forwardKinematics(angles(M_PI_2, -M_PI_2), speeds(1, 1), vx, vy, wz);
  EXPECT_NEAR(vx, 0.0, kTol);
  EXPECT_NEAR(vy, 0.0, kTol);
  EXPECT_NEAR(wz, 1.0, kTol);
}

TEST_F(SwerveForwardKinematicsTest, PureRotation_NegativeAngularVelocity) {
  sk.forwardKinematics(angles(-M_PI_2, M_PI_2), speeds(1, 1), vx, vy, wz);
  EXPECT_NEAR(vx, 0.0, kTol);
  EXPECT_NEAR(vy, 0.0, kTol);
  EXPECT_NEAR(wz, -1.0, kTol);
}

// --- Mixed motion -------------------------------------------------------

TEST_F(SwerveForwardKinematicsTest, MixedMotion_TranslationAndRotation) {
  // Manually compute expected: wheel0 angle=pi/4, wheel1 angle=pi/2, both speed=1
  // v0 = (cos(pi/4), sin(pi/4)) = (√2/2, √2/2)
  // v1 = (cos(pi/2), sin(pi/2)) = (0,     1    )
  // mean: vx = √2/4, vy = (√2/2+1)/2
  // cross2d(v0, r0=[1,0]) = v0.y*1 - v0.x*0 = √2/2
  // cross2d(v1, r1=[-1,0]) = v1.y*(-1) - v1.x*0 = -1
  // wz = (√2/2 - 1) / 2
  sk.forwardKinematics(angles(M_PI / 4.0, M_PI_2), speeds(1, 1), vx, vy, wz);
  const double sq2 = std::sqrt(2.0);
  EXPECT_NEAR(vx, sq2 / 4.0, kTol);
  EXPECT_NEAR(vy, (sq2 / 2.0 + 1.0) / 2.0, kTol);
  EXPECT_NEAR(wz, (sq2 / 2.0 - 1.0) / 2.0, kTol);
}

// --- Wheel-radius scaling -----------------------------------------------

TEST_F(SwerveForwardKinematicsTest, WheelRadiusScalesOutputLinearly) {
  const double r2 = 2.0;
  SwerveKinematics sk2({Eigen::Vector2d{kL, 0}, Eigen::Vector2d{-kL, 0}}, r2);
  double vx2{}, vy2{}, wz2{};
  sk.forwardKinematics(angles(0, 0), speeds(1, 1), vx, vy, wz);
  sk2.forwardKinematics(angles(0, 0), speeds(1, 1), vx2, vy2, wz2);
  EXPECT_NEAR(vx2, vx * r2, kTol);
}

// --- Returns true -------------------------------------------------------

TEST_F(SwerveForwardKinematicsTest, ReturnsTrueOnSuccess) {
  EXPECT_TRUE(sk.forwardKinematics(angles(0, 0), speeds(1, 1), vx, vy, wz));
}

// ===========================================================================
// forwardKinematicsQr — QR least-squares path
// ===========================================================================

class SwerveForwardKinematicsQrTest : public ::testing::Test {
protected:
  SwerveKinematics sk = make_default_kinematics();
  double vx{}, vy{}, wz{};
};

TEST_F(SwerveForwardKinematicsQrTest, PureTranslationX_MatchesClosedForm) {
  double vx_cf{}, vy_cf{}, wz_cf{};
  sk.forwardKinematics(angles(0, 0), speeds(1, 1), vx_cf, vy_cf, wz_cf);
  sk.forwardKinematicsQr(angles(0, 0), speeds(1, 1), vx, vy, wz);
  EXPECT_NEAR(vx, vx_cf, kTol);
  EXPECT_NEAR(vy, vy_cf, kTol);
  EXPECT_NEAR(wz, wz_cf, kTol);
}

TEST_F(SwerveForwardKinematicsQrTest, PureRotation_MatchesClosedForm) {
  double vx_cf{}, vy_cf{}, wz_cf{};
  sk.forwardKinematics(angles(M_PI_2, -M_PI_2), speeds(1, 1), vx_cf, vy_cf, wz_cf);
  sk.forwardKinematicsQr(angles(M_PI_2, -M_PI_2), speeds(1, 1), vx, vy, wz);
  EXPECT_NEAR(vx, vx_cf, kTol);
  EXPECT_NEAR(vy, vy_cf, kTol);
  EXPECT_NEAR(wz, wz_cf, kTol);
}

TEST_F(SwerveForwardKinematicsQrTest, MixedMotion_MatchesClosedForm) {
  double vx_cf{}, vy_cf{}, wz_cf{};
  sk.forwardKinematics(angles(M_PI / 4.0, M_PI_2), speeds(1, 1), vx_cf, vy_cf, wz_cf);
  sk.forwardKinematicsQr(angles(M_PI / 4.0, M_PI_2), speeds(1, 1), vx, vy, wz);
  EXPECT_NEAR(vx, vx_cf, kTol);
  EXPECT_NEAR(vy, vy_cf, kTol);
  EXPECT_NEAR(wz, wz_cf, kTol);
}

TEST_F(SwerveForwardKinematicsQrTest, ZeroInputYieldsZeroVelocities) {
  sk.forwardKinematicsQr(angles(0, 0), speeds(0, 0), vx, vy, wz);
  EXPECT_NEAR(vx, 0.0, kTol);
  EXPECT_NEAR(vy, 0.0, kTol);
  EXPECT_NEAR(wz, 0.0, kTol);
}

TEST_F(SwerveForwardKinematicsQrTest, ReturnsTrueOnSuccess) {
  EXPECT_TRUE(sk.forwardKinematicsQr(angles(0, 0), speeds(1, 1), vx, vy, wz));
}

// ===========================================================================
// inverseKinematics — input validation
// ===========================================================================

class SwerveInverseKinematicsValidationTest : public ::testing::Test {
protected:
  SwerveKinematics sk = make_default_kinematics();
  std::array<double, 2> sa{}, ws{};
};

TEST_F(SwerveInverseKinematicsValidationTest, InfiniteVxReturnsFalse) {
  EXPECT_FALSE(sk.inverseKinematics(
      std::numeric_limits<double>::infinity(), 0, 0, sa, ws));
}

TEST_F(SwerveInverseKinematicsValidationTest, InfiniteVyReturnsFalse) {
  EXPECT_FALSE(sk.inverseKinematics(
      0, std::numeric_limits<double>::infinity(), 0, sa, ws));
}

TEST_F(SwerveInverseKinematicsValidationTest, InfiniteWzReturnsFalse) {
  EXPECT_FALSE(sk.inverseKinematics(
      0, 0, std::numeric_limits<double>::infinity(), sa, ws));
}

TEST_F(SwerveInverseKinematicsValidationTest, NanVxReturnsFalse) {
  EXPECT_FALSE(sk.inverseKinematics(
      std::numeric_limits<double>::quiet_NaN(), 0, 0, sa, ws));
}

TEST_F(SwerveInverseKinematicsValidationTest, ZeroInputReturnsTrue) {
  EXPECT_TRUE(sk.inverseKinematics(0, 0, 0, sa, ws));
}

// ===========================================================================
// inverseKinematics — correctness
// ===========================================================================

class SwerveInverseKinematicsTest : public ::testing::Test {
protected:
  SwerveKinematics sk = make_default_kinematics();
  std::array<double, 2> sa{0, 0}, ws{0, 0};
};

// --- Pure translation along X -------------------------------------------

TEST_F(SwerveInverseKinematicsTest, PureTranslationX_BothWheelsPointForward) {
  sk.inverseKinematics(1.0, 0.0, 0.0, sa, ws);
  EXPECT_NEAR(sa[0], 0.0, kTol);
  EXPECT_NEAR(sa[1], 0.0, kTol);
}

TEST_F(SwerveInverseKinematicsTest, PureTranslationX_EqualSpeeds) {
  sk.inverseKinematics(1.0, 0.0, 0.0, sa, ws);
  EXPECT_NEAR(ws[0], ws[1], kTol);
}

TEST_F(SwerveInverseKinematicsTest, PureTranslationX_CorrectSpeed) {
  // speed = |v| / wheel_radius = 1.0 / 1.0 = 1.0
  sk.inverseKinematics(1.0, 0.0, 0.0, sa, ws);
  EXPECT_NEAR(ws[0], 1.0, kTol);
}

// --- Pure translation along Y -------------------------------------------

TEST_F(SwerveInverseKinematicsTest, PureTranslationY_BothWheelsPointSideways) {
  sk.inverseKinematics(0.0, 1.0, 0.0, sa, ws);
  EXPECT_NEAR(sa[0], M_PI_2, kTol);
  EXPECT_NEAR(sa[1], M_PI_2, kTol);
}

// --- Pure rotation ------------------------------------------------------
TEST_F(SwerveInverseKinematicsTest, PureRotation_EqualSpeeds) {
  sk.inverseKinematics(0.0, 0.0, 1.0, sa, ws);
  EXPECT_NEAR(std::fabs(ws[0]), std::fabs(ws[1]), kTol);
}

TEST_F(SwerveInverseKinematicsTest, PureRotation_CorrectSpeed) {
  sk.inverseKinematics(0.0, 0.0, 1.0, sa, ws);
  EXPECT_NEAR(ws[0], 1.0, kTol);
}

// --- Zero input ---------------------------------------------------------

TEST_F(SwerveInverseKinematicsTest, ZeroInput_SpeedsAreZero) {
  sk.inverseKinematics(0.0, 0.0, 0.0, sa, ws);
  EXPECT_NEAR(ws[0], 0.0, kTol);
  EXPECT_NEAR(ws[1], 0.0, kTol);
}

// --- Angle-flip optimisation --------------------------------------------
// When the required angle differs from the current angle by more than pi/2,
// the implementation flips the direction and negates speed to avoid
// over-rotating the steering mechanism.

TEST_F(SwerveInverseKinematicsTest, AngleFlip_SpeedNegatedInsteadOfLargeSteer) {
  // First call: wheels pointing at 0.
  sk.inverseKinematics(1.0, 0.0, 0.0, sa, ws);
  ASSERT_NEAR(sa[0], 0.0, kTol);

  // Second call: command vx = -1 (exact opposite direction).
  // |pi - 0| = pi > pi/2 → flip should be triggered.
  // Resulting angle stays near 0 (or near pi) and speed is negated.
  std::array<double, 2> sa2{sa}, ws2{ws};
  sk.inverseKinematics(-1.0, 0.0, 0.0, sa2, ws2);

  // After flip the steering change is at most pi/2 and speed is negative.
  EXPECT_LE(std::fabs(sa2[0] - sa[0]), M_PI_2 + kTol);
  EXPECT_LT(ws2[0], 0.0);
}

// ===========================================================================
// Round-trip: inverseKinematics → forwardKinematics
// ===========================================================================

class SwerveRoundTripTest : public ::testing::Test {
protected:
  SwerveKinematics sk = make_default_kinematics();

  void check_round_trip(double cmd_vx, double cmd_vy, double cmd_wz) {
    std::array<double, 2> sa{0, 0}, ws{0, 0};
    ASSERT_TRUE(sk.inverseKinematics(cmd_vx, cmd_vy, cmd_wz, sa, ws));

    double vx{}, vy{}, wz{};
    ASSERT_TRUE(sk.forwardKinematics(sa, ws, vx, vy, wz));

    EXPECT_NEAR(vx, cmd_vx, kTol) << "vx mismatch for cmd=(" << cmd_vx << "," << cmd_vy << "," << cmd_wz << ")";
    EXPECT_NEAR(vy, cmd_vy, kTol) << "vy mismatch";
    EXPECT_NEAR(wz, cmd_wz, kTol) << "wz mismatch";
  }
};

TEST_F(SwerveRoundTripTest, PureTranslationX) { check_round_trip(1.0, 0.0, 0.0); }
TEST_F(SwerveRoundTripTest, PureTranslationY) { check_round_trip(0.0, 1.0, 0.0); }
TEST_F(SwerveRoundTripTest, PureRotation)     { check_round_trip(0.0, 0.0, 1.0); }
TEST_F(SwerveRoundTripTest, DiagonalMotion)   { check_round_trip(1.0, 1.0, 0.0); }
TEST_F(SwerveRoundTripTest, MixedMotion)      { check_round_trip(0.5, 0.3, 0.2); }
TEST_F(SwerveRoundTripTest, NegativeVelocities) { check_round_trip(-1.0, -0.5, -0.3); }

// Same round-trip via the QR path.
class SwerveRoundTripQrTest : public ::testing::Test {
protected:
  SwerveKinematics sk = make_default_kinematics();

  void check_round_trip_qr(double cmd_vx, double cmd_vy, double cmd_wz) {
    std::array<double, 2> sa{0, 0}, ws{0, 0};
    ASSERT_TRUE(sk.inverseKinematics(cmd_vx, cmd_vy, cmd_wz, sa, ws));

    double vx{}, vy{}, wz{};
    ASSERT_TRUE(sk.forwardKinematicsQr(sa, ws, vx, vy, wz));

    EXPECT_NEAR(vx, cmd_vx, kTol);
    EXPECT_NEAR(vy, cmd_vy, kTol);
    EXPECT_NEAR(wz, cmd_wz, kTol);
  }
};

TEST_F(SwerveRoundTripQrTest, PureTranslationX)  { check_round_trip_qr(1.0, 0.0, 0.0); }
TEST_F(SwerveRoundTripQrTest, PureRotation)      { check_round_trip_qr(0.0, 0.0, 1.0); }
TEST_F(SwerveRoundTripQrTest, MixedMotion)       { check_round_trip_qr(0.5, 0.3, 0.2); }

}  // namespace franka_mobile

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}