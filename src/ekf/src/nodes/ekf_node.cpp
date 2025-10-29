#include "ekf/nodes/ekf_node.hpp"

namespace ekf
{

EKFNode::EKFNode()
    : Node("ekf_node"), is_initialized_(false)
{
  // VIO subscriber
  vio_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vio/odometry",
      rclcpp::SensorDataQoS(),
      std::bind(&EKFNode::vioCallback, this, std::placeholders::_1));

  // Vision subscriber
  vision_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/vision/pose",
      rclcpp::SensorDataQoS(),
      std::bind(&EKFNode::visionCallback, this, std::placeholders::_1));

  // Odometry publisher
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/ekf/odometry",
      10);

  RCLCPP_INFO(this->get_logger(), "EKF Node initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribed to: /vio/odometry, /vision/pose, /imu/data");
  RCLCPP_INFO(this->get_logger(), "Publishing to: /ekf/odometry");
}

void EKFNode::vioCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!is_initialized_)
  {
    initialize();
  }

  // Update position from VIO
  px_ = msg->pose.pose.position.x;
  py_ = msg->pose.pose.position.y;
  pz_ = msg->pose.pose.position.z;

  // Update orientation from VIO
  qx_ = msg->pose.pose.orientation.x;
  qy_ = msg->pose.pose.orientation.y;
  qz_ = msg->pose.pose.orientation.z;
  qw_ = msg->pose.pose.orientation.w;

  // Update linear velocity from VIO
  vx_ = msg->twist.twist.linear.x;
  vy_ = msg->twist.twist.linear.y;
  vz_ = msg->twist.twist.linear.z;

  // Update angular velocity from VIO
  wx_ = msg->twist.twist.angular.x;
  wy_ = msg->twist.twist.angular.y;
  wz_ = msg->twist.twist.angular.z;

  publishOdometry();
}

void EKFNode::visionCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (!is_initialized_)
  {
    initialize();
  }

  // Update position from vision
  px_ = msg->pose.pose.position.x;
  py_ = msg->pose.pose.position.y;
  pz_ = msg->pose.pose.position.z;

  // Update orientation from vision
  qx_ = msg->pose.pose.orientation.x;
  qy_ = msg->pose.pose.orientation.y;
  qz_ = msg->pose.pose.orientation.z;
  qw_ = msg->pose.pose.orientation.w;

  publishOdometry();
}

void EKFNode::initialize()
{
  // Initialize position to zero
  px_ = 0.0;
  py_ = 0.0;
  pz_ = 0.0;

  // Initialize orientation to identity quaternion
  qx_ = 0.0;
  qy_ = 0.0;
  qz_ = 0.0;
  qw_ = 1.0;

  // Initialize velocities to zero
  vx_ = 0.0;
  vy_ = 0.0;
  vz_ = 0.0;

  wx_ = 0.0;
  wy_ = 0.0;
  wz_ = 0.0;
}

void EKFNode::publishOdometry()
{
  auto msg = nav_msgs::msg::Odometry();
  msg.header.stamp = this->now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  // Position
  msg.pose.pose.position.x = px_;
  msg.pose.pose.position.y = py_;
  msg.pose.pose.position.z = pz_;

  // Orientation
  msg.pose.pose.orientation.x = qx_;
  msg.pose.pose.orientation.y = qy_;
  msg.pose.pose.orientation.z = qz_;
  msg.pose.pose.orientation.w = qw_;

  // Linear velocity
  msg.twist.twist.linear.x = vx_;
  msg.twist.twist.linear.y = vy_;
  msg.twist.twist.linear.z = vz_;

  // Angular velocity
  msg.twist.twist.angular.x = wx_;
  msg.twist.twist.angular.y = wy_;
  msg.twist.twist.angular.z = wz_;

  odom_publisher_->publish(msg);
}

} // namespace ekf

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ekf::EKFNode>());
  rclcpp::shutdown();
  return 0;
}
