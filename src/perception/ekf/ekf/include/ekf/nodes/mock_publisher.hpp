// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace ekf
{

class MockPublisher : public rclcpp::Node
{
public:
  /** Constructor — declares parameters and creates publishers/timers. */
  MockPublisher();
  ~MockPublisher() override = default;

private:
  // Timers
  rclcpp::TimerBase::SharedPtr vio_timer_;
  rclcpp::TimerBase::SharedPtr vision_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vio_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vision_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr control_publisher_;

  size_t count_;

  // Scenario generators — VIO (Odometry)
  void test_resting_drone(nav_msgs::msg::Odometry & msg);
  void test_constant_velocity_drone(nav_msgs::msg::Odometry & msg);
  void test_constant_turn(nav_msgs::msg::Odometry & msg);
  void test_constant_accel(nav_msgs::msg::Odometry & msg);

  // Scenario generators — Vision (PoseWithCovarianceStamped)
  void test_resting_drone(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void test_constant_velocity_drone(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void test_constant_turn(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void test_constant_accel(geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  // Scenario generators — Control (TwistStamped)
  void test_resting_control(geometry_msgs::msg::TwistStamped & msg);
  void test_constant_velocity_control(geometry_msgs::msg::TwistStamped & msg);
  void test_constant_turn_control(geometry_msgs::msg::TwistStamped & msg);
  void test_constant_accel_control(geometry_msgs::msg::TwistStamped & msg);

  // Hard scenario generators — VIO (Odometry)
  void test_sprint(nav_msgs::msg::Odometry & msg);
  void test_slalom(nav_msgs::msg::Odometry & msg);
  void test_pitch_dive(nav_msgs::msg::Odometry & msg);
  void test_dropout(nav_msgs::msg::Odometry & msg);

  // Hard scenario generators — Vision (PoseWithCovarianceStamped)
  void test_sprint(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void test_slalom(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void test_pitch_dive(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void test_dropout(geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  // Hard scenario generators — Control (TwistStamped)
  void test_sprint_control(geometry_msgs::msg::TwistStamped & msg);
  void test_slalom_control(geometry_msgs::msg::TwistStamped & msg);
  void test_pitch_dive_control(geometry_msgs::msg::TwistStamped & msg);
  void test_dropout_control(geometry_msgs::msg::TwistStamped & msg);

  std::string scenario_;
  double duration_sec_;
  int rng_seed_;
  double vio_rate_hz_;
  double vision_rate_hz_;
  double control_rate_hz_;

  // World-frame state for scenarios requiring Euler-integrated position/heading.
  // Updated once per VIO timer tick; read by vision timer for pose messages.
  double x_w_{0.0};
  double y_w_{0.0};
  double z_w_{0.0};
  double yaw_w_{0.0};
  double pitch_w_{0.0};
};

}  // namespace ekf
