// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

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

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vio_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vision_publisher_;

  // Counter for generating varying data
  size_t count_;

  // Helper function to generate mock data
    void test_resting_drone(nav_msgs::msg::Odometry& msg);
    void test_resting_drone(geometry_msgs::msg::PoseWithCovarianceStamped& msg);
    void test_constant_velocity_drone(nav_msgs::msg::Odometry& msg);
    void test_constant_velocity_drone(geometry_msgs::msg::PoseWithCovarianceStamped& msg);
};

} // namespace ekf
