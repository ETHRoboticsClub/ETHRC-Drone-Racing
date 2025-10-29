// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace ekf
{

/**
 * EKFNode
 *
 * ROS 2 node that implements an Extended Kalman Filter for sensor fusion.
 * Subscribes to VIO and Vision pose data.
 */
class EKFNode : public rclcpp::Node
{
public:
  /**
   * Constructor
   */
  EKFNode();

  /**
   * Destructor
   */
  ~EKFNode() override = default;

private:
  /**
   * Callback for VIO odometry messages
   *
   * @param msg VIO odometry message containing pose and twist
   */
  void vioCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /**
   * Callback for Vision pose messages
   *
   * @param msg Vision pose message with covariance
   */
  void visionCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * Initialize the EKF state with default values
   */
  void initialize();

  /**
   * Publish the current filtered odometry estimate
   */
  void publishOdometry();

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vision_subscription_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  // State variables
  double px_, py_, pz_;              // position
  double qx_, qy_, qz_, qw_;         // orientation (quaternion)
  double vx_, vy_, vz_;              // linear velocity
  double wx_, wy_, wz_;              // angular velocity
  bool is_initialized_;
};

} // namespace ekf
