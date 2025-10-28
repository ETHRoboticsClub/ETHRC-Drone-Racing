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

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS());

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

            RCLCPP_INFO(this->get_logger(), "Publishing VIO Odometry: (%.3f, %.3f, %.3f)",
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z);
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
            vision_publisher_->publish(msg);

            // Increment count after both messages are published
            ++count_; });

    // IMU runs at higher rate (100Hz for mock purposes)
    imu_timer_ = this->create_wall_timer(
        10ms, [this]()
        {
            sensor_msgs::msg::Imu msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "imu_link";

            // Simulate some accelerations (including gravity on z-axis)
            msg.linear_acceleration.x = 0.1;
            msg.linear_acceleration.y = 0.05;
            msg.linear_acceleration.z = 9.81;  // gravity

            // Simulate angular velocities
            msg.angular_velocity.x = 0.01;
            msg.angular_velocity.y = 0.02;
            msg.angular_velocity.z = 0.0;

            msg.linear_acceleration_covariance.fill(0.0);
            msg.linear_acceleration_covariance[0] = 0.01;  // x variance
            msg.linear_acceleration_covariance[4] = 0.01;  // y variance
            msg.linear_acceleration_covariance[8] = 0.01;  // z variance

            msg.angular_velocity_covariance.fill(0.0);
            msg.angular_velocity_covariance[0] = 0.001;  // x variance
            msg.angular_velocity_covariance[4] = 0.001;  // y variance
            msg.angular_velocity_covariance[8] = 0.001;  // z variance

            msg.orientation_covariance.fill(0.0);
            msg.orientation_covariance[0] = -1.0;

            imu_publisher_->publish(msg); });
}

} // namespace ekf

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ekf::MockPublisher>());
    rclcpp::shutdown();
    return 0;
}