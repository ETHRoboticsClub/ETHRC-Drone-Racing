// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace ekf
{

class MockPublisher : public rclcpp::Node
{
public:
  /**
   * Constructor
   */
  MockPublisher();

  /**
   * Destructor
   */
  ~MockPublisher() override = default;

private:
  // Timers for periodic publishing
  rclcpp::TimerBase::SharedPtr vio_timer_;
  rclcpp::TimerBase::SharedPtr vision_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vio_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vision_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  // Counter for generating varying data
  size_t count_;
};

} // namespace ekf
