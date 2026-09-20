#include "ekf/nodes/mock_publisher.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <random>

using namespace std::chrono_literals;

namespace ekf
{

// Thread-local RNG and helper for white Gaussian noise
static thread_local std::mt19937 rng{std::random_device{}()};
static double gaussian(double mean, double stddev)
{
  std::normal_distribution<double> dist(mean, stddev);
  return dist(rng);
}

static void normalize_quaternion(geometry_msgs::msg::Quaternion & quat)
{
  const double norm = std::sqrt(
    quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
  if (norm < 1e-9) {
    quat.x = quat.y = quat.z = 0.0;
    quat.w = 1.0;
    return;
  }
  const double inv_norm = 1.0 / norm;
  quat.x *= inv_norm;
  quat.y *= inv_norm;
  quat.z *= inv_norm;
  quat.w *= inv_norm;
}

// Noise standard deviations — AGI-style for VIO, lower for vision
static constexpr double AGI_ERR_POS = 1.0e-2;
static constexpr double AGI_ERR_VEL = 1.0e-2;
static constexpr double AGI_ERR_ATT = 1.0e-3;
static constexpr double AGI_ERR_OME = 1.0e-1;
static constexpr double DEFAULT_ERR_POS = 8.0e-3;
static constexpr double DEFAULT_ERR_REST = 1.0e-3;

// ── Covariance helpers (ROS 6×6 row-major: [x y z roll pitch yaw]) ──

static void set_pose_covariance(
  nav_msgs::msg::Odometry & msg, double pos_std, double att_std)
{
  auto & cov = msg.pose.covariance;
  cov.fill(0.0);
  const double pos_var = pos_std * pos_std;
  const double att_var = att_std * att_std;
  cov[0] = pos_var;   // x
  cov[7] = pos_var;   // y
  cov[14] = pos_var;  // z
  cov[21] = att_var;  // roll
  cov[28] = att_var;  // pitch
  cov[35] = att_var;  // yaw
}

static void set_twist_covariance(
  nav_msgs::msg::Odometry & msg, double lin_vel_std, double ang_vel_std)
{
  auto & cov = msg.twist.covariance;
  cov.fill(0.0);
  const double lin_var = lin_vel_std * lin_vel_std;
  const double ang_var = ang_vel_std * ang_vel_std;
  cov[0] = lin_var;   // vx
  cov[7] = lin_var;   // vy
  cov[14] = lin_var;  // vz
  cov[21] = ang_var;  // wx
  cov[28] = ang_var;  // wy
  cov[35] = ang_var;  // wz
}

static void set_pose_covariance(
  geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  double pos_std, double att_std)
{
  auto & cov = msg.pose.covariance;
  cov.fill(0.0);
  const double pos_var = pos_std * pos_std;
  const double att_var = att_std * att_std;
  cov[0] = pos_var;
  cov[7] = pos_var;
  cov[14] = pos_var;
  cov[21] = att_var;
  cov[28] = att_var;
  cov[35] = att_var;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario: REST  (hover at origin)
// ══════════════════════════════════════════════════════════════════════

void MockPublisher::test_resting_drone(nav_msgs::msg::Odometry & msg)
{
  msg.pose.pose.position.x = gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, AGI_ERR_ATT);
  msg.pose.pose.position.z = gaussian(0.0, AGI_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, AGI_ERR_ATT);
  q.y = gaussian(0.0, AGI_ERR_ATT);
  q.z = gaussian(0.0, AGI_ERR_ATT);
  q.w = 1.0 + gaussian(0.0, AGI_ERR_ATT);
  normalize_quaternion(q);
  msg.twist.twist.linear.x = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.y = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.z = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.angular.x = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.y = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.z = gaussian(0.0, AGI_ERR_OME);
  set_pose_covariance(msg, AGI_ERR_POS, AGI_ERR_ATT);
  set_twist_covariance(msg, AGI_ERR_VEL, AGI_ERR_OME);
}

void MockPublisher::test_resting_drone(
  geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  msg.pose.pose.position.x = gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, DEFAULT_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, DEFAULT_ERR_REST);
  q.y = gaussian(0.0, DEFAULT_ERR_REST);
  q.z = gaussian(0.0, DEFAULT_ERR_REST);
  q.w = 1.0 + gaussian(0.0, DEFAULT_ERR_REST);
  normalize_quaternion(q);
  set_pose_covariance(msg, DEFAULT_ERR_POS, DEFAULT_ERR_REST);
}

void MockPublisher::test_resting_control(geometry_msgs::msg::TwistStamped & msg)
{
  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario: CONST_VEL  (steady 0.05 m/s in X)
// ══════════════════════════════════════════════════════════════════════

void MockPublisher::test_constant_velocity_drone(nav_msgs::msg::Odometry & msg)
{
  constexpr double vx = 0.05;
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t = static_cast<double>(count_) * dt;

  msg.pose.pose.position.x = vx * t + gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, AGI_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, AGI_ERR_ATT);
  q.y = gaussian(0.0, AGI_ERR_ATT);
  q.z = gaussian(0.0, AGI_ERR_ATT);
  q.w = 1.0 + gaussian(0.0, AGI_ERR_ATT);
  normalize_quaternion(q);
  msg.twist.twist.linear.x = vx + gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.y = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.z = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.angular.x = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.y = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.z = gaussian(0.0, AGI_ERR_OME);
  set_pose_covariance(msg, AGI_ERR_POS, AGI_ERR_ATT);
  set_twist_covariance(msg, AGI_ERR_VEL, AGI_ERR_OME);
}

void MockPublisher::test_constant_velocity_drone(
  geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  constexpr double vx = 0.05;
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t = static_cast<double>(count_) * dt;

  msg.pose.pose.position.x = vx * t + gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, DEFAULT_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, DEFAULT_ERR_REST);
  q.y = gaussian(0.0, DEFAULT_ERR_REST);
  q.z = gaussian(0.0, DEFAULT_ERR_REST);
  q.w = 1.0 + gaussian(0.0, DEFAULT_ERR_REST);
  normalize_quaternion(q);
  set_pose_covariance(msg, DEFAULT_ERR_POS, DEFAULT_ERR_REST);
}

void MockPublisher::test_constant_velocity_control(
  geometry_msgs::msg::TwistStamped & msg)
{
  msg.twist.linear.x = 0.05;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario: TURN  (yaw rotation ω_z = 0.5 rad/s)
// ══════════════════════════════════════════════════════════════════════

void MockPublisher::test_constant_turn(nav_msgs::msg::Odometry & msg)
{
  constexpr double omega = 0.5;
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t = static_cast<double>(count_) * dt;
  const double yaw = omega * t;

  msg.pose.pose.position.x = gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, AGI_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, AGI_ERR_ATT);
  q.y = gaussian(0.0, AGI_ERR_ATT);
  q.z = std::sin(0.5 * yaw) + gaussian(0.0, AGI_ERR_ATT);
  q.w = std::cos(0.5 * yaw) + gaussian(0.0, AGI_ERR_ATT);
  normalize_quaternion(q);
  msg.twist.twist.linear.x = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.y = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.z = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.angular.x = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.y = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.z = omega + gaussian(0.0, AGI_ERR_OME);
  set_pose_covariance(msg, AGI_ERR_POS, AGI_ERR_ATT);
  set_twist_covariance(msg, AGI_ERR_VEL, AGI_ERR_OME);
}

void MockPublisher::test_constant_turn(
  geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  constexpr double omega = 0.5;
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t = static_cast<double>(count_) * dt;
  const double yaw = omega * t;

  msg.pose.pose.position.x = gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, DEFAULT_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, DEFAULT_ERR_REST);
  q.y = gaussian(0.0, DEFAULT_ERR_REST);
  q.z = std::sin(0.5 * yaw) + gaussian(0.0, DEFAULT_ERR_REST);
  q.w = std::cos(0.5 * yaw) + gaussian(0.0, DEFAULT_ERR_REST);
  normalize_quaternion(q);
  set_pose_covariance(msg, DEFAULT_ERR_POS, DEFAULT_ERR_REST);
}

void MockPublisher::test_constant_turn_control(
  geometry_msgs::msg::TwistStamped & msg)
{
  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.5;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario: ACCEL  (constant 0.5 m/s² in X)
// ══════════════════════════════════════════════════════════════════════

void MockPublisher::test_constant_accel(nav_msgs::msg::Odometry & msg)
{
  constexpr double accel = 0.5;
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t = static_cast<double>(count_) * dt;

  msg.pose.pose.position.x = 0.5 * accel * t * t + gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, AGI_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, AGI_ERR_ATT);
  q.y = gaussian(0.0, AGI_ERR_ATT);
  q.z = gaussian(0.0, AGI_ERR_ATT);
  q.w = 1.0 + gaussian(0.0, AGI_ERR_ATT);
  normalize_quaternion(q);
  msg.twist.twist.linear.x = accel * t + gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.y = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.z = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.angular.x = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.y = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.z = gaussian(0.0, AGI_ERR_OME);
  set_pose_covariance(msg, AGI_ERR_POS, AGI_ERR_ATT);
  set_twist_covariance(msg, AGI_ERR_VEL, AGI_ERR_OME);
}

void MockPublisher::test_constant_accel(
  geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  constexpr double accel = 0.5;
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t = static_cast<double>(count_) * dt;

  msg.pose.pose.position.x = 0.5 * accel * t * t + gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, DEFAULT_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, DEFAULT_ERR_REST);
  q.y = gaussian(0.0, DEFAULT_ERR_REST);
  q.z = gaussian(0.0, DEFAULT_ERR_REST);
  q.w = 1.0 + gaussian(0.0, DEFAULT_ERR_REST);
  normalize_quaternion(q);
  set_pose_covariance(msg, DEFAULT_ERR_POS, DEFAULT_ERR_REST);
}

void MockPublisher::test_constant_accel_control(
  geometry_msgs::msg::TwistStamped & msg)
{
  constexpr double accel = 0.5;
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t = static_cast<double>(count_) * dt;
  msg.twist.linear.x = accel * t;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario: SPRINT  (aggressive straight-line acceleration / deceleration)
//
// Three phases driven by elapsed time t = count_ / vio_rate_hz_:
//   [0,        t_ramp):         constant accel SPRINT_A  → vx ramps 0→v_max
//   [t_ramp,   T-t_ramp):       cruise at SPRINT_VMAX
//   [T-t_ramp, T]:              constant decel -SPRINT_A → vx ramps v_max→0
//
// No rotation throughout: quaternion stays identity.
// VIO orientation is disabled in hard_test_ekf.yaml, so the EKF must
// maintain identity orientation purely through the physics prediction model.
// Ground truth: vx(t) as above, x(t) integrated analytically.
// ══════════════════════════════════════════════════════════════════════
static constexpr double SPRINT_A    = 5.0;   // m/s²  — typical racing accel
static constexpr double SPRINT_VMAX = 8.0;   // m/s   — competitive FPV speed

static double sprint_vx(double t, double duration_sec)
{
  const double t_ramp = SPRINT_VMAX / SPRINT_A;
  if (t < t_ramp) {
    return SPRINT_A * t;
  } else if (t < duration_sec - t_ramp) {
    return SPRINT_VMAX;
  } else {
    return std::max(0.0, SPRINT_VMAX - SPRINT_A * (t - (duration_sec - t_ramp)));
  }
}

static double sprint_x(double t, double duration_sec)
{
  const double t_ramp = SPRINT_VMAX / SPRINT_A;
  const double x_ramp = 0.5 * SPRINT_A * t_ramp * t_ramp;
  if (t < t_ramp) {
    return 0.5 * SPRINT_A * t * t;
  } else if (t < duration_sec - t_ramp) {
    return x_ramp + SPRINT_VMAX * (t - t_ramp);
  } else {
    const double t3 = t - (duration_sec - t_ramp);
    return x_ramp + SPRINT_VMAX * (duration_sec - 2.0 * t_ramp) +
           SPRINT_VMAX * t3 - 0.5 * SPRINT_A * t3 * t3;
  }
}

void MockPublisher::test_sprint(nav_msgs::msg::Odometry & msg)
{
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t  = static_cast<double>(count_) * dt;
  const double vx = sprint_vx(t, duration_sec_);

  msg.pose.pose.position.x = sprint_x(t, duration_sec_) + gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, AGI_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, AGI_ERR_ATT);
  q.y = gaussian(0.0, AGI_ERR_ATT);
  q.z = gaussian(0.0, AGI_ERR_ATT);
  q.w = 1.0 + gaussian(0.0, AGI_ERR_ATT);
  normalize_quaternion(q);
  msg.twist.twist.linear.x = vx + gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.y = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.z = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.angular.x = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.y = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.z = gaussian(0.0, AGI_ERR_OME);
  set_pose_covariance(msg, AGI_ERR_POS, AGI_ERR_ATT);
  set_twist_covariance(msg, AGI_ERR_VEL, AGI_ERR_OME);
}

void MockPublisher::test_sprint(geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t  = static_cast<double>(count_) * dt;

  msg.pose.pose.position.x = sprint_x(t, duration_sec_) + gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, DEFAULT_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, DEFAULT_ERR_REST);
  q.y = gaussian(0.0, DEFAULT_ERR_REST);
  q.z = gaussian(0.0, DEFAULT_ERR_REST);
  q.w = 1.0 + gaussian(0.0, DEFAULT_ERR_REST);
  normalize_quaternion(q);
  set_pose_covariance(msg, DEFAULT_ERR_POS, DEFAULT_ERR_REST);
}

void MockPublisher::test_sprint_control(geometry_msgs::msg::TwistStamped & msg)
{
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t  = static_cast<double>(count_) * dt;
  msg.twist.linear.x = sprint_vx(t, duration_sec_);
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario: SLALOM  (forward flight + sinusoidal yaw)
//
// The drone flies at constant forward body-speed SLALOM_VFW while the
// heading oscillates: wz(t) = SLALOM_WZMAX · cos(SLALOM_OMEGA · t).
// This integrates to yaw(t) = (SLALOM_WZMAX / SLALOM_OMEGA) · sin(SLALOM_OMEGA · t),
// giving yaw_amp ≈ ±1.91 rad (±109°) — large, fast heading swings.
//
// World-frame position is accumulated via Euler integration in the VIO
// timer (x_w_, y_w_), so vision can serve the same growing position.
//
// VIO provides body-frame velocity and angular rate only — NO orientation.
// Sparse vision corrections at 5 Hz are the sole absolute heading source.
// This is the toughest orientation scenario: wide swings, long dead-reckoning.
// ══════════════════════════════════════════════════════════════════════
static constexpr double SLALOM_VFW   = 3.0;              // m/s forward (body x)
static constexpr double SLALOM_WZMAX = 1.5;              // rad/s peak yaw rate
static constexpr double SLALOM_OMEGA = 2.0 * M_PI / 8.0; // rad/s  (8 s period)

void MockPublisher::test_slalom(nav_msgs::msg::Odometry & msg)
{
  const double dt  = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t   = static_cast<double>(count_) * dt;
  const double wz  = SLALOM_WZMAX * std::cos(SLALOM_OMEGA * t);

  // Euler-integrate world position and heading (updated each VIO tick)
  yaw_w_ += wz * dt;
  x_w_   += SLALOM_VFW * std::cos(yaw_w_) * dt;
  y_w_   += SLALOM_VFW * std::sin(yaw_w_) * dt;

  msg.pose.pose.position.x = x_w_ + gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.y = y_w_ + gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, AGI_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, AGI_ERR_ATT);
  q.y = gaussian(0.0, AGI_ERR_ATT);
  q.z = std::sin(0.5 * yaw_w_) + gaussian(0.0, AGI_ERR_ATT);
  q.w = std::cos(0.5 * yaw_w_) + gaussian(0.0, AGI_ERR_ATT);
  normalize_quaternion(q);
  // Body-frame twist: drone always faces its heading
  msg.twist.twist.linear.x = SLALOM_VFW + gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.y = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.z = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.angular.x = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.y = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.z = wz + gaussian(0.0, AGI_ERR_OME);
  set_pose_covariance(msg, AGI_ERR_POS, AGI_ERR_ATT);
  set_twist_covariance(msg, AGI_ERR_VEL, AGI_ERR_OME);
}

void MockPublisher::test_slalom(geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  // Uses the world position already updated by the VIO callback
  msg.pose.pose.position.x = x_w_ + gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.y = y_w_ + gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, DEFAULT_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, DEFAULT_ERR_REST);
  q.y = gaussian(0.0, DEFAULT_ERR_REST);
  q.z = std::sin(0.5 * yaw_w_) + gaussian(0.0, DEFAULT_ERR_REST);
  q.w = std::cos(0.5 * yaw_w_) + gaussian(0.0, DEFAULT_ERR_REST);
  normalize_quaternion(q);
  set_pose_covariance(msg, DEFAULT_ERR_POS, DEFAULT_ERR_REST);
}

void MockPublisher::test_slalom_control(geometry_msgs::msg::TwistStamped & msg)
{
  const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t  = static_cast<double>(count_) * dt;
  msg.twist.linear.x  = SLALOM_VFW;
  msg.twist.linear.y  = 0.0;
  msg.twist.linear.z  = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = SLALOM_WZMAX * std::cos(SLALOM_OMEGA * t);
}

// ══════════════════════════════════════════════════════════════════════
// Scenario: PITCH_DIVE  (oscillating climb-dive at speed)
//
// The drone flies forward at PITCH_VFW m/s while its pitch oscillates
// between ±PITCH_AMP radians (≈ ±40°): pitch(t) = PITCH_AMP · sin(PITCH_OMEGA · t).
// This drives alternating climbs and dives, coupling the x and z axes.
//
// World-frame position is Euler-integrated (x_w_, z_w_) each VIO tick.
// VIO provides body-frame velocity and pitch rate (angular.y) — NO orientation.
// The EKF must reconstruct pitch from the angular rate signal and the
// gravity-coupling term (Ax = -2g·(qw·qy + qx·qz)) in the drone physics model.
// Pitch up to ±40° places the filter near the gimbal-lock region of the
// legacy Euler EKF while remaining nominal for the quaternion EKF.
// ══════════════════════════════════════════════════════════════════════
static constexpr double PITCH_VFW   = 4.0;               // m/s forward base speed
static constexpr double PITCH_AMP   = 0.7;               // rad ≈ 40° peak pitch
static constexpr double PITCH_OMEGA = 2.0 * M_PI / 6.0;  // rad/s  (6 s period)

void MockPublisher::test_pitch_dive(nav_msgs::msg::Odometry & msg)
{
  const double dt       = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t        = static_cast<double>(count_) * dt;
  pitch_w_              = PITCH_AMP * std::sin(PITCH_OMEGA * t);
  const double wpitch   = PITCH_AMP * PITCH_OMEGA * std::cos(PITCH_OMEGA * t);

  // Euler-integrate world position: x forward, z climbs/dives with pitch
  x_w_ += PITCH_VFW * std::cos(pitch_w_) * dt;
  z_w_ += PITCH_VFW * std::sin(pitch_w_) * dt;

  msg.pose.pose.position.x = x_w_ + gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.z = z_w_ + gaussian(0.0, AGI_ERR_POS);
  // Pure pitch quaternion (y-axis rotation in ENU body frame)
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, AGI_ERR_ATT);
  q.y = std::sin(0.5 * pitch_w_) + gaussian(0.0, AGI_ERR_ATT);
  q.z = gaussian(0.0, AGI_ERR_ATT);
  q.w = std::cos(0.5 * pitch_w_) + gaussian(0.0, AGI_ERR_ATT);
  normalize_quaternion(q);
  // Body-frame twist: forward velocity + pitch rate; lateral/roll/yaw zero
  msg.twist.twist.linear.x = PITCH_VFW + gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.y = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.z = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.angular.x = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.y = wpitch + gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.z = gaussian(0.0, AGI_ERR_OME);
  set_pose_covariance(msg, AGI_ERR_POS, AGI_ERR_ATT);
  set_twist_covariance(msg, AGI_ERR_VEL, AGI_ERR_OME);
}

void MockPublisher::test_pitch_dive(geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  msg.pose.pose.position.x = x_w_ + gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.z = z_w_ + gaussian(0.0, DEFAULT_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, DEFAULT_ERR_REST);
  q.y = std::sin(0.5 * pitch_w_) + gaussian(0.0, DEFAULT_ERR_REST);
  q.z = gaussian(0.0, DEFAULT_ERR_REST);
  q.w = std::cos(0.5 * pitch_w_) + gaussian(0.0, DEFAULT_ERR_REST);
  normalize_quaternion(q);
  set_pose_covariance(msg, DEFAULT_ERR_POS, DEFAULT_ERR_REST);
}

void MockPublisher::test_pitch_dive_control(geometry_msgs::msg::TwistStamped & msg)
{
  const double dt    = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t     = static_cast<double>(count_) * dt;
  const double pitch = PITCH_AMP * std::sin(PITCH_OMEGA * t);
  msg.twist.linear.x  = PITCH_VFW * std::cos(pitch);
  msg.twist.linear.y  = 0.0;
  msg.twist.linear.z  = PITCH_VFW * std::sin(pitch);
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario: DROPOUT  (constant turn with periodic VIO blackouts)
//
// Standard turning flight (omega=0.5 rad/s, vx=2 m/s helical trajectory)
// identical to the easy 'turn' scenario, but VIO is silenced for
// DROP_DUR seconds every DROP_PERIOD seconds, simulating visual-tracking
// loss in fast gate passages.
//
// During a dropout the EKF receives NO VIO corrections and must dead-reckon
// orientation exclusively from the angular-rate decay model (ekf.cpp) and
// the sparse vision corrections (5 Hz in hard_test_ekf.yaml).
// This directly tests the quality of the quaternion predict step.
// ══════════════════════════════════════════════════════════════════════
static constexpr double DROPOUT_OMEGA  = 0.5;   // rad/s yaw rate
static constexpr double DROPOUT_VX     = 2.0;   // m/s forward → helical path
static constexpr double DROP_PERIOD    = 7.0;   // s, period between blackouts
static constexpr double DROP_DUR       = 2.5;   // s, VIO-silent window per period

void MockPublisher::test_dropout(nav_msgs::msg::Odometry & msg)
{
  const double dt  = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t   = static_cast<double>(count_) * dt;
  const double yaw = DROPOUT_OMEGA * t;

  msg.pose.pose.position.x = DROPOUT_VX * t + gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, AGI_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, AGI_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, AGI_ERR_ATT);
  q.y = gaussian(0.0, AGI_ERR_ATT);
  q.z = std::sin(0.5 * yaw) + gaussian(0.0, AGI_ERR_ATT);
  q.w = std::cos(0.5 * yaw) + gaussian(0.0, AGI_ERR_ATT);
  normalize_quaternion(q);
  msg.twist.twist.linear.x = DROPOUT_VX + gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.y = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.linear.z = gaussian(0.0, AGI_ERR_VEL);
  msg.twist.twist.angular.x = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.y = gaussian(0.0, AGI_ERR_OME);
  msg.twist.twist.angular.z = DROPOUT_OMEGA + gaussian(0.0, AGI_ERR_OME);
  set_pose_covariance(msg, AGI_ERR_POS, AGI_ERR_ATT);
  set_twist_covariance(msg, AGI_ERR_VEL, AGI_ERR_OME);
}

void MockPublisher::test_dropout(geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  const double dt  = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
  const double t   = static_cast<double>(count_) * dt;
  const double yaw = DROPOUT_OMEGA * t;

  msg.pose.pose.position.x = DROPOUT_VX * t + gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.y = gaussian(0.0, DEFAULT_ERR_POS);
  msg.pose.pose.position.z = gaussian(0.0, DEFAULT_ERR_POS);
  auto & q = msg.pose.pose.orientation;
  q.x = gaussian(0.0, DEFAULT_ERR_REST);
  q.y = gaussian(0.0, DEFAULT_ERR_REST);
  q.z = std::sin(0.5 * yaw) + gaussian(0.0, DEFAULT_ERR_REST);
  q.w = std::cos(0.5 * yaw) + gaussian(0.0, DEFAULT_ERR_REST);
  normalize_quaternion(q);
  set_pose_covariance(msg, DEFAULT_ERR_POS, DEFAULT_ERR_REST);
}

void MockPublisher::test_dropout_control(geometry_msgs::msg::TwistStamped & msg)
{
  msg.twist.linear.x  = DROPOUT_VX;
  msg.twist.linear.y  = 0.0;
  msg.twist.linear.z  = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = DROPOUT_OMEGA;
}

// ══════════════════════════════════════════════════════════════════════
// Constructor
// ══════════════════════════════════════════════════════════════════════

MockPublisher::MockPublisher()
  : Node("mock_publisher"), count_(0)
{
  scenario_ = this->declare_parameter<std::string>("scenario", "const_vel");
  duration_sec_ = this->declare_parameter<double>("duration_sec", 60.0);
  rng_seed_ = this->declare_parameter<int>("rng_seed", 42);
  vio_rate_hz_ = this->declare_parameter<double>("vio_rate_hz", 100.0);
  vision_rate_hz_ = this->declare_parameter<double>("vision_rate_hz", 60.0);
  control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 100.0);

  rng.seed(static_cast<std::mt19937::result_type>(rng_seed_));

  static const std::array<std::string, 8> valid = {
    "rest", "const_vel", "turn", "accel",
    "sprint", "slalom", "pitch_dive", "dropout"};
  if (std::find(valid.begin(), valid.end(), scenario_) == valid.end()) {
    RCLCPP_WARN(this->get_logger(),
      "Unknown scenario '%s', defaulting to const_vel", scenario_.c_str());
    scenario_ = "const_vel";
  }

  auto clamp_rate = [this](double rate, double fallback, const char * name) {
    if (rate <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "%s must be > 0 Hz. Using %.2f Hz.", name, fallback);
      return fallback;
    }
    return rate;
  };
  vio_rate_hz_ = clamp_rate(vio_rate_hz_, 100.0, "vio_rate_hz");
  vision_rate_hz_ = clamp_rate(vision_rate_hz_, 5.0, "vision_rate_hz");

  if (duration_sec_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "duration_sec must be > 0. Using 1.0s.");
    duration_sec_ = 1.0;
  }

  // Publishers — same topic names as the legacy tests
  vio_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/vio/odometry", rclcpp::SensorDataQoS());
  vision_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/vision/pose", rclcpp::SensorDataQoS());
  control_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/control/command", rclcpp::QoS(10));

  auto to_dur = [](double seconds) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  };

  // ── VIO timer ──
  vio_timer_ = this->create_wall_timer(to_dur(1.0 / vio_rate_hz_), [this]() {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    if (scenario_ == "rest") { test_resting_drone(msg); }
    else if (scenario_ == "turn") { test_constant_turn(msg); }
    else if (scenario_ == "accel") { test_constant_accel(msg); }
    else if (scenario_ == "sprint") { test_sprint(msg); }
    else if (scenario_ == "slalom") { test_slalom(msg); }
    else if (scenario_ == "pitch_dive") { test_pitch_dive(msg); }
    else if (scenario_ == "dropout") {
      // Suppress publish during blackout windows; still increment count_ below
      const double dt_d = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
      const double t_d  = static_cast<double>(count_) * dt_d;
      const double t_in_period = std::fmod(t_d, DROP_PERIOD);
      if (t_in_period >= DROP_PERIOD - DROP_DUR) {
        ++count_;
        return;
      }
      test_dropout(msg);
    }
    else { test_constant_velocity_drone(msg); }

    vio_publisher_->publish(msg);
    ++count_;
  });

  // ── Vision timer ──
  vision_timer_ = this->create_wall_timer(to_dur(1.0 / vision_rate_hz_), [this]() {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";

    if (scenario_ == "rest") { test_resting_drone(msg); }
    else if (scenario_ == "turn") { test_constant_turn(msg); }
    else if (scenario_ == "accel") { test_constant_accel(msg); }
    else if (scenario_ == "sprint") { test_sprint(msg); }
    else if (scenario_ == "slalom") { test_slalom(msg); }
    else if (scenario_ == "pitch_dive") { test_pitch_dive(msg); }
    else if (scenario_ == "dropout") { test_dropout(msg); }
    else { test_constant_velocity_drone(msg); }

    vision_publisher_->publish(msg);
  });

  // ── Control timer ──
  control_timer_ = this->create_wall_timer(to_dur(1.0 / control_rate_hz_), [this]() {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    if (scenario_ == "rest") { test_resting_control(msg); }
    else if (scenario_ == "const_vel") { test_constant_velocity_control(msg); }
    else if (scenario_ == "turn") { test_constant_turn_control(msg); }
    else if (scenario_ == "accel") { test_constant_accel_control(msg); }
    else if (scenario_ == "sprint") { test_sprint_control(msg); }
    else if (scenario_ == "slalom") { test_slalom_control(msg); }
    else if (scenario_ == "pitch_dive") { test_pitch_dive_control(msg); }
    else if (scenario_ == "dropout") { test_dropout_control(msg); }

    control_publisher_->publish(msg);
  });

  // ── Auto-shutdown timer ──
  shutdown_timer_ = this->create_wall_timer(to_dur(duration_sec_), [this]() {
    RCLCPP_INFO(this->get_logger(), "Scenario '%s' complete, shutting down.", scenario_.c_str());
    if (shutdown_timer_) { shutdown_timer_->cancel(); }
    rclcpp::shutdown();
  });

  RCLCPP_INFO(this->get_logger(),
    "MockPublisher: scenario=%s, duration=%.1fs, vio=%.0fHz, vision=%.0fHz, seed=%d",
    scenario_.c_str(), duration_sec_, vio_rate_hz_, vision_rate_hz_, rng_seed_);
}

}  // namespace ekf

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ekf::MockPublisher>());
  rclcpp::shutdown();
  return 0;
}
