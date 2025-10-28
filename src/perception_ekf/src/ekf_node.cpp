#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include "perception_ekf/ekf.hpp"

class EKFNode : public rclcpp::Node
{
public:
  EKFNode()
      : Node("ekf_node")
  {
    // VIO subscriber
    vio_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/vio/odometry",
        rclcpp::SensorDataQoS(), // Not sure about this, who has more experience with ROS2 QoS settings?
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          // Show the EKF received VIO pose
          const auto &p = msg->pose.pose.position;
          RCLCPP_INFO(this->get_logger(), "VIO Position: (%.3f, %.3f, %.3f)", p.x, p.y, p.z);

          // Show the EKF received VIO velocity
          const auto &v = msg->twist.twist.linear;
          RCLCPP_INFO(this->get_logger(), "VIO Velocity: (%.3f, %.3f, %.3f)", v.x, v.y, v.z);

          // Show the EKF received VIO orientation
          const auto &o = msg->pose.pose.orientation;
          RCLCPP_INFO(this->get_logger(), "VIO Orientation: (%.3f, %.3f, %.3f, %.3f)", o.x, o.y, o.z, o.w);

          // Show the timestamp
          RCLCPP_INFO(this->get_logger(), "VIO Timestamp: %u.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
        });

    // Vision subscriber
    vision_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>( // Maybe use PoseStamped?
        "/vision/pose",
        rclcpp::SensorDataQoS(), // Not sure about this, who has more experience with ROS2 QoS settings?
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
        {
          // Show the EKF received Vision pose
          const auto &p = msg->pose.pose.position;
          RCLCPP_INFO(this->get_logger(), "Vision Position: (%.3f, %.3f, %.3f)", p.x, p.y, p.z);

          // Show the timestamp
          RCLCPP_INFO(this->get_logger(), "Vision Timestamp: %u.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);

          // Show the EKF received Vision covariance
          RCLCPP_INFO(this->get_logger(), "Vision Covariance: [[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                      msg->pose.covariance[0], msg->pose.covariance[7], msg->pose.covariance[14],
                      msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35]);
        });
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr vio_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr vision_subscription_;
  // Add any additional subscriptions here
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}