#include "ekf/nodes/mock_publisher.hpp"

using namespace std::chrono_literals;

namespace ekf
{

MockPublisher::MockPublisher()
    : Node("mock_publisher"), count_(0)
{
    vio_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/vio/odometry", rclcpp::SensorDataQoS());

    vision_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/vision/pose", rclcpp::SensorDataQoS());

    // Actual publisher will obviously have a higher rate than this mock
    vio_timer_ = this->create_wall_timer(
        1s, [this]()
        {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "odom";
            msg.child_frame_id = "base_link";

            // Simulate some changing position
            double x = 0.05 * static_cast<double>(count_);
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = 6.9;
            msg.pose.pose.position.z = 4.20;

            // Simulate rotation (in this case no rotation)
            msg.pose.pose.orientation.w = 1.0;

            // Simulate some constant velocity
            msg.twist.twist.linear.x = 0.1;
            msg.twist.twist.linear.y = 0.0;
            msg.twist.twist.linear.z = 0.0;
            msg.twist.twist.angular.x = 0.0;
            msg.twist.twist.angular.y = 0.0;
            msg.twist.twist.angular.z = 0.0;

            RCLCPP_INFO(this->get_logger(), "Publishing VIO Position: (%.3f, %.3f, %.3f)",
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Publishing VIO Orientation: (%.3f, %.3f, %.3f, %.3f)",
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w);
            RCLCPP_INFO(this->get_logger(), "Publishing VIO Linear Velocity: (%.3f, %.3f, %.3f)",
                        msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y,
                        msg.twist.twist.linear.z);
            RCLCPP_INFO(this->get_logger(), "Publishing VIO Angular Velocity: (%.3f, %.3f, %.3f)",
                        msg.twist.twist.angular.x,
                        msg.twist.twist.angular.y,
                        msg.twist.twist.angular.z);
            vio_publisher_->publish(msg); });

    vision_timer_ = this->create_wall_timer(
        1s, [this]()
        {
            geometry_msgs::msg::PoseWithCovarianceStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "odom";

            // keep it near the VIO pose; pretend it's a correction
            double x = 0.05 * static_cast<double>(count_) + 0.01; // small offset
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = 6.9 + 0.02;  // small offset from VIO
            msg.pose.pose.position.z = 4.20 - 0.01; // small offset from VIO

            msg.pose.pose.orientation.w = 1.0;

            // set a simple diagonal covariance for demo
            msg.pose.covariance.fill(0.0);
            msg.pose.covariance[0] = 0.01;
            msg.pose.covariance[7] = 0.02;
            msg.pose.covariance[14] = 0.03;
            msg.pose.covariance[21] = 0.04;
            msg.pose.covariance[28] = 0.05;
            msg.pose.covariance[35] = 0.06;

            RCLCPP_INFO(this->get_logger(), "Publishing Vision Pose: (%.3f, %.3f, %.3f)",
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Publishing Vision Orientation: (%.3f, %.3f, %.3f, %.3f)",
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w);
            RCLCPP_INFO(this->get_logger(), "Publishing Vision Covariance: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
                        msg.pose.covariance[0],
                        msg.pose.covariance[7],
                        msg.pose.covariance[14],
                        msg.pose.covariance[21],
                        msg.pose.covariance[28],
                        msg.pose.covariance[35]);
            vision_publisher_->publish(msg);

            // Increment count after both messages are published
            ++count_; });
}

} // namespace ekf

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ekf::MockPublisher>());
    rclcpp::shutdown();
    return 0;
}