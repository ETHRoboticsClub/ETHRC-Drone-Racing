#include "ekf/nodes/mock_publisher.hpp"
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

    static double default_err_pos = 2.0e-3;
    static double default_err_rest = 1.0e-3;

    static double agi_err_pos = 1.0e-2;
    static double agi_err_vel = 1.0e-2;
    static double agi_err_att = 1.0e-3;
    static double agi_err_ome = 1.0e-1;


    void MockPublisher::test_resting_drone(nav_msgs::msg::Odometry& msg)
    {
        
        msg.pose.pose.position.x = gaussian(0.0, agi_err_pos);
        msg.pose.pose.position.y = gaussian(0.0, agi_err_att);
        msg.pose.pose.position.z = gaussian(0.0, agi_err_pos);

        msg.pose.pose.orientation.x = gaussian(0.0, agi_err_att);
        msg.pose.pose.orientation.y = gaussian(0.0, agi_err_att);
        msg.pose.pose.orientation.z = gaussian(0.0, agi_err_att);
        msg.pose.pose.orientation.w = 1.0 + gaussian(0.0, agi_err_att);

        msg.twist.twist.linear.x = gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.y = gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.z = gaussian(0.0, agi_err_vel);

        msg.twist.twist.angular.x = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.y = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.z = gaussian(0.0, agi_err_ome);
    }

    void MockPublisher::test_resting_drone(geometry_msgs::msg::PoseWithCovarianceStamped& msg)
    {
        msg.pose.pose.position.x = gaussian(0.0, default_err_pos);
        msg.pose.pose.position.y = gaussian(0.0, default_err_pos);
        msg.pose.pose.position.z = gaussian(0.0, default_err_pos);

        msg.pose.pose.orientation.x = gaussian(0.0, default_err_rest);
        msg.pose.pose.orientation.y = gaussian(0.0, default_err_rest);
        msg.pose.pose.orientation.z = gaussian(0.0, default_err_rest);
        msg.pose.pose.orientation.w = 1.0 + gaussian(0.0, default_err_rest);
    }

MockPublisher::MockPublisher()
    : Node("mock_publisher"), count_(0)
{
    vio_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/vio/odometry", rclcpp::SensorDataQoS());

    vision_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/vision/pose", rclcpp::SensorDataQoS());

    // Actual publisher will obviously have a higher rate than this mock
    vio_timer_ = this->create_wall_timer(
        10ms, [this]()
        {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "odom";
            msg.child_frame_id = "base_link";

            /* 
            // Simulate some changing position
            double x = 0.05 * static_cast<double>(count_);
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = 6.9;
            msg.pose.pose.position.z = 4.20;

            // Simulate rotation (in this case no rotation)
            msg.pose.pose.orientation.w = 1.0;

            // Simulate some constant velocity
            msg.twist.twist.linear.x = 0.0;
            msg.twist.twist.linear.y = 0.0;
            msg.twist.twist.linear.z = 0.0;
            msg.twist.twist.angular.x = 0.0;
            msg.twist.twist.angular.y = 0.0;
            msg.twist.twist.angular.z = 0.0;
            */

            /*
            =====================
            TEST CASES
            =====================
            */

            // Testing of resting drone
            test_resting_drone(msg);

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
        200ms, [this]()
        {
            geometry_msgs::msg::PoseWithCovarianceStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "base_link";

            /*          
            // keep it near the VIO pose; pretend it's a correction
            double x = 0.05 * static_cast<double>(count_) + 0.01; // small offset
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = 6.9 + 0.02;  // small offset from VIO
            msg.pose.pose.position.z = 4.20 - 0.01; // small offset from VIO

            // Testing of resting drone
            msg.pose.pose.position.x = 0.0;
            msg.pose.pose.position.y = 0.0;
            msg.pose.pose.position.z = 0.0;

            msg.pose.pose.orientation.w = 1.0;
            */

            // set a simple diagonal covariance for demo
            msg.pose.covariance.fill(0.0);
            msg.pose.covariance[0] = 0.01;
            msg.pose.covariance[7] = 0.02;
            msg.pose.covariance[14] = 0.03;
            msg.pose.covariance[21] = 0.04;
            msg.pose.covariance[28] = 0.05;
            msg.pose.covariance[35] = 0.06;

            /*
            =====================
            TEST CASES
            =====================
            */

            // Testing of resting drone
            test_resting_drone(msg);

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