/**
 * @file test_drone_ekf.cpp
 * @brief Sanity tests for the 16-state quaternion EKF drone predict step.
 *
 * Tests are white-box: construct an Ekf object, set state directly,
 * call predictDt(), and assert physical invariants from the drone dynamics.
 *
 * No ROS node or rclcpp types required — predictDt() takes a plain double.
 */

#include <cmath>
#include <gtest/gtest.h>

#include <robot_localization/ekf.hpp>
#include <robot_localization/filter_common.hpp>

using robot_localization::Ekf;
using robot_localization::StateMemberQw;
using robot_localization::StateMemberQx;
using robot_localization::StateMemberQy;
using robot_localization::StateMemberQz;
using robot_localization::StateMemberVx;
using robot_localization::StateMemberVy;
using robot_localization::StateMemberVz;
using robot_localization::StateMemberVroll;
using robot_localization::StateMemberAx;
using robot_localization::StateMemberAy;
using robot_localization::StateMemberAz;
using robot_localization::StateMemberX;
using robot_localization::StateMemberY;
using robot_localization::StateMemberZ;
using robot_localization::STATE_SIZE;

/// Create a minimally configured Ekf (zero process noise, unit covariance).
static Ekf makeFilter()
{
  Ekf ekf;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  ekf.setProcessNoiseCovariance(Q);
  ekf.setEstimateErrorCovariance(P);
  return ekf;
}

/**
 * At hover (identity orientation, zero velocity), predict() should leave
 * position unchanged and keep the quaternion unit-length.
 */
TEST(DroneEkfPredict, HoverPositionUnchanged)
{
  Ekf ekf = makeFilter();
  Eigen::VectorXd s = Eigen::VectorXd::Zero(STATE_SIZE);
  s(StateMemberQw) = 1.0;  // identity quaternion
  ekf.setState(s);

  ekf.predictDt(0.01);

  const Eigen::VectorXd & out = ekf.getState();
  EXPECT_NEAR(out(StateMemberX), 0.0, 1e-9);
  EXPECT_NEAR(out(StateMemberY), 0.0, 1e-9);
  EXPECT_NEAR(out(StateMemberZ), 0.0, 1e-9);
}

/**
 * Quaternion must remain unit length after repeated predict() calls.
 * Runs 100 steps with a non-zero angular rate to exercise quaternion kinematics.
 */
TEST(DroneEkfPredict, QuaternionNormPreserved)
{
  Ekf ekf = makeFilter();
  Eigen::VectorXd s = Eigen::VectorXd::Zero(STATE_SIZE);
  s(StateMemberQw) = 1.0;
  s(StateMemberVroll) = 0.5;  // rad/s — exercises Ξ(q)·w update
  ekf.setState(s);

  for (int i = 0; i < 100; ++i) {
    ekf.predictDt(0.01);
  }

  const Eigen::VectorXd & out = ekf.getState();
  const double norm = std::sqrt(
    out(StateMemberQw) * out(StateMemberQw) +
    out(StateMemberQx) * out(StateMemberQx) +
    out(StateMemberQy) * out(StateMemberQy) +
    out(StateMemberQz) * out(StateMemberQz));
  EXPECT_NEAR(norm, 1.0, 1e-6);
}

/**
 * At hover orientation (Qw=1, body axis = world axis), body-frame gravity
 * components Ax and Ay must be ≈0 (gravity acts along -z_body at hover).
 * Az = 0 by hover assumption.
 */
TEST(DroneEkfPredict, GravityAtHover)
{
  Ekf ekf = makeFilter();
  Eigen::VectorXd s = Eigen::VectorXd::Zero(STATE_SIZE);
  s(StateMemberQw) = 1.0;
  ekf.setState(s);

  ekf.predictDt(0.01);

  const Eigen::VectorXd & out = ekf.getState();
  // Ax = -2g*(Qw*Qy + Qx*Qz) = 0 at hover (Qy=Qz=Qx=0)
  EXPECT_NEAR(out(StateMemberAx), 0.0, 1e-9);
  // Ay = -2g*(Qy*Qz - Qw*Qx) = 0 at hover
  EXPECT_NEAR(out(StateMemberAy), 0.0, 1e-9);
  // Az = 0 always (hover assumption)
  EXPECT_NEAR(out(StateMemberAz), 0.0, 1e-9);
}

/**
 * Tilting by 45° around the y-axis (pure Qy rotation) must produce a
 * non-zero body-frame Ax from gravity coupling.
 * q = cos(22.5°) + sin(22.5°)*j  (a 45° pitch rotation)
 * Ax = -2g*(cos(π/8)*sin(π/8) + 0) ≈ -9.81*sin(45°) ≈ -6.94 m/s²
 */
TEST(DroneEkfPredict, GravityCouplingWithTilt)
{
  Ekf ekf = makeFilter();
  Eigen::VectorXd s = Eigen::VectorXd::Zero(STATE_SIZE);
  s(StateMemberQw) = std::cos(M_PI / 8.0);
  s(StateMemberQy) = std::sin(M_PI / 8.0);
  ekf.setState(s);

  ekf.predictDt(0.01);

  const Eigen::VectorXd & out = ekf.getState();
  // |Ax| = 2g*|Qw*Qy| = 2*9.81*cos(π/8)*sin(π/8) = 9.81*sin(π/4) ≈ 6.94
  EXPECT_GT(std::abs(out(StateMemberAx)), 1.0);
}

/**
 * Angular rates must decay each step by factor (1 - dt/tau_w) = 0.9.
 * Starting from Vroll = 1 rad/s, after one step of dt=0.01s expect 0.9 rad/s.
 */
TEST(DroneEkfPredict, AngularRateDecay)
{
  Ekf ekf = makeFilter();
  Eigen::VectorXd s = Eigen::VectorXd::Zero(STATE_SIZE);
  s(StateMemberQw)    = 1.0;
  s(StateMemberVroll) = 1.0;  // 1 rad/s initial roll rate
  ekf.setState(s);

  const double dt     = 0.01;
  const double tau_w  = 0.1;
  const double expect = 1.0 * (1.0 - dt / tau_w);  // = 0.9

  ekf.predictDt(dt);

  EXPECT_NEAR(ekf.getState()(StateMemberVroll), expect, 1e-9);
}
