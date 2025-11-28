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

    static void normalize_quaternion(geometry_msgs::msg::Quaternion &quat)
    {
        const double norm = std::sqrt(
            quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);

        if (norm < 1e-9)
        {
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

    static double default_err_pos = 8.0e-3; // Previously 2.0e-3;
    static double default_err_rest = 1.0e-3;

    static double agi_err_pos = 1.0e-2;
    static double agi_err_vel = 1.0e-2;
    static double agi_err_att = 1.0e-3;
    static double agi_err_ome = 1.0e-1;

    // Helper utilities to populate reasonable measurement covariances
    // from the configured 1σ errors above. These are ROS 6x6 covariances
    // laid out row-major for [x y z roll pitch yaw].
    static void set_pose_covariance(nav_msgs::msg::Odometry &msg,
                                    double pos_std,
                                    double att_std)
    {
        auto &cov = msg.pose.covariance;
        cov.fill(0.0);

        const double pos_var = pos_std * pos_std;
        const double att_var = att_std * att_std;

        // Position variances
        cov[0] = pos_var;  // x
        cov[7] = pos_var;  // y
        cov[14] = pos_var; // z

        // Orientation variances (small-angle roll/pitch/yaw)
        cov[21] = att_var; // roll
        cov[28] = att_var; // pitch
        cov[35] = att_var; // yaw
    }

    static void set_twist_covariance(nav_msgs::msg::Odometry &msg,
                                     double lin_vel_std,
                                     double ang_vel_std)
    {
        auto &cov = msg.twist.covariance;
        cov.fill(0.0);

        const double lin_var = lin_vel_std * lin_vel_std;
        const double ang_var = ang_vel_std * ang_vel_std;

        // Linear velocity variances
        cov[0] = lin_var;  // vx
        cov[7] = lin_var;  // vy
        cov[14] = lin_var; // vz

        // Angular velocity variances
        cov[21] = ang_var; // wx
        cov[28] = ang_var; // wy
        cov[35] = ang_var; // wz
    }

    static void set_pose_covariance(geometry_msgs::msg::PoseWithCovarianceStamped &msg,
                                    double pos_std,
                                    double att_std)
    {
        auto &cov = msg.pose.covariance;
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

    void MockPublisher::test_resting_drone(nav_msgs::msg::Odometry &msg)
    {

        msg.pose.pose.position.x = gaussian(0.0, agi_err_pos);
        msg.pose.pose.position.y = gaussian(0.0, agi_err_att);
        msg.pose.pose.position.z = gaussian(0.0, agi_err_pos);

        auto &quat = msg.pose.pose.orientation;
        quat.x = gaussian(0.0, agi_err_att);
        quat.y = gaussian(0.0, agi_err_att);
        quat.z = gaussian(0.0, agi_err_att);
        quat.w = 1.0 + gaussian(0.0, agi_err_att);
        normalize_quaternion(quat);

        msg.twist.twist.linear.x = gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.y = gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.z = gaussian(0.0, agi_err_vel);

        msg.twist.twist.angular.x = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.y = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.z = gaussian(0.0, agi_err_ome);

        set_pose_covariance(msg, agi_err_pos, agi_err_att);
        set_twist_covariance(msg, agi_err_vel, agi_err_ome);
    }

    void MockPublisher::test_resting_drone(geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        msg.pose.pose.position.x = gaussian(0.0, default_err_pos);
        msg.pose.pose.position.y = gaussian(0.0, default_err_pos);
        msg.pose.pose.position.z = gaussian(0.0, default_err_pos);

        auto &quat = msg.pose.pose.orientation;
        quat.x = gaussian(0.0, default_err_rest);
        quat.y = gaussian(0.0, default_err_rest);
        quat.z = gaussian(0.0, default_err_rest);
        quat.w = 1.0 + gaussian(0.0, default_err_rest);
        normalize_quaternion(quat);
        set_pose_covariance(msg, default_err_pos, default_err_rest);
    }

    void MockPublisher::test_resting_control(geometry_msgs::msg::TwistStamped &msg)
    {
        // No motion command
        msg.twist.linear.x = 0.0;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.0;
    }

    void MockPublisher::test_constant_velocity_drone(nav_msgs::msg::Odometry &msg)
    {
        // Constant velocity motion in x with small measurement noise
        const double vx = 0.05; // m/s
        const double vy = 0.0;
        const double vz = 0.0;
        const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
        const double t = static_cast<double>(count_) * dt;

        // Position = v * t + noise
        msg.pose.pose.position.x = vx * t + gaussian(0.0, agi_err_pos);
        msg.pose.pose.position.y = vy * t + gaussian(0.0, agi_err_pos);
        msg.pose.pose.position.z = vz * t + gaussian(0.0, agi_err_pos);

        // Nearly identity orientation with small attitude noise
        auto &quat = msg.pose.pose.orientation;
        quat.x = gaussian(0.0, agi_err_att);
        quat.y = gaussian(0.0, agi_err_att);
        quat.z = gaussian(0.0, agi_err_att);
        quat.w = 1.0 + gaussian(0.0, agi_err_att);
        normalize_quaternion(quat);

        // Linear velocity measurements around the constant value
        msg.twist.twist.linear.x = vx + gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.y = vy + gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.z = vz + gaussian(0.0, agi_err_vel);

        // Angular velocity close to zero with noise
        msg.twist.twist.angular.x = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.y = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.z = gaussian(0.0, agi_err_ome);

        set_pose_covariance(msg, agi_err_pos, agi_err_att);
        set_twist_covariance(msg, agi_err_vel, agi_err_ome);
    }

    void MockPublisher::test_constant_velocity_drone(geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        // Match the same constant velocity profile as VIO
        const double vx = 0.05; // m/s
        const double vy = 0.0;
        const double vz = 0.0;
        const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
        const double t = static_cast<double>(count_) * dt;

        msg.pose.pose.position.x = vx * t + gaussian(0.0, default_err_pos);
        msg.pose.pose.position.y = vy * t + gaussian(0.0, default_err_pos);
        msg.pose.pose.position.z = vz * t + gaussian(0.0, default_err_pos);

        auto &quat = msg.pose.pose.orientation;
        quat.x = gaussian(0.0, default_err_rest);
        quat.y = gaussian(0.0, default_err_rest);
        quat.z = gaussian(0.0, default_err_rest);
        quat.w = 1.0 + gaussian(0.0, default_err_rest);
        normalize_quaternion(quat);
        set_pose_covariance(msg, default_err_pos, default_err_rest);
    }

    void MockPublisher::test_constant_velocity_control(geometry_msgs::msg::TwistStamped &msg)
    {
        // Match test_constant_velocity_drone: vx = 0.05 m/s
        msg.twist.linear.x = 0.05;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.0;
    }

    void MockPublisher::test_constant_turn(nav_msgs::msg::Odometry &msg)
    {
        const double omega = 0.5; // rad/s
        const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
        const double t = static_cast<double>(count_) * dt;
        const double yaw = omega * t;

        msg.pose.pose.position.x = gaussian(0.0, agi_err_pos);
        msg.pose.pose.position.y = gaussian(0.0, agi_err_pos);
        msg.pose.pose.position.z = gaussian(0.0, agi_err_pos);

        auto &quat = msg.pose.pose.orientation;
        quat.x = gaussian(0.0, agi_err_att);
        quat.y = gaussian(0.0, agi_err_att);
        quat.z = std::sin(0.5 * yaw) + gaussian(0.0, agi_err_att);
        quat.w = std::cos(0.5 * yaw) + gaussian(0.0, agi_err_att);
        normalize_quaternion(quat);

        msg.twist.twist.linear.x = gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.y = gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.z = gaussian(0.0, agi_err_vel);

        msg.twist.twist.angular.x = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.y = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.z = omega + gaussian(0.0, agi_err_ome);

        set_pose_covariance(msg, agi_err_pos, agi_err_att);
        set_twist_covariance(msg, agi_err_vel, agi_err_ome);
    }

    void MockPublisher::test_constant_turn(geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        const double omega = 0.5; // rad/s
        const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
        const double t = static_cast<double>(count_) * dt;
        const double yaw = omega * t;

        msg.pose.pose.position.x = gaussian(0.0, default_err_pos);
        msg.pose.pose.position.y = gaussian(0.0, default_err_pos);
        msg.pose.pose.position.z = gaussian(0.0, default_err_pos);

        auto &quat = msg.pose.pose.orientation;
        quat.x = gaussian(0.0, default_err_rest);
        quat.y = gaussian(0.0, default_err_rest);
        quat.z = std::sin(0.5 * yaw) + gaussian(0.0, default_err_rest);
        quat.w = std::cos(0.5 * yaw) + gaussian(0.0, default_err_rest);
        normalize_quaternion(quat);
        set_pose_covariance(msg, default_err_pos, default_err_rest);
    }

    void MockPublisher::test_constant_turn_control(geometry_msgs::msg::TwistStamped &msg)
    {
        // Match test_constant_turn: yaw rate omega = 0.5 rad/s
        msg.twist.linear.x = 0.0;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.5;
    }

    void MockPublisher::test_constant_accel(nav_msgs::msg::Odometry &msg)
    {
        const double accel = 0.5; // m/s^2
        const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
        const double t = static_cast<double>(count_) * dt;
        const double position_x = 0.5 * accel * t * t;
        const double velocity_x = accel * t;

        msg.pose.pose.position.x = position_x + gaussian(0.0, agi_err_pos);
        msg.pose.pose.position.y = gaussian(0.0, agi_err_pos);
        msg.pose.pose.position.z = gaussian(0.0, agi_err_pos);

        auto &quat = msg.pose.pose.orientation;
        quat.x = gaussian(0.0, agi_err_att);
        quat.y = gaussian(0.0, agi_err_att);
        quat.z = gaussian(0.0, agi_err_att);
        quat.w = 1.0 + gaussian(0.0, agi_err_att);
        normalize_quaternion(quat);

        msg.twist.twist.linear.x = velocity_x + gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.y = gaussian(0.0, agi_err_vel);
        msg.twist.twist.linear.z = gaussian(0.0, agi_err_vel);

        msg.twist.twist.angular.x = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.y = gaussian(0.0, agi_err_ome);
        msg.twist.twist.angular.z = gaussian(0.0, agi_err_ome);

        set_pose_covariance(msg, agi_err_pos, agi_err_att);
        set_twist_covariance(msg, agi_err_vel, agi_err_ome);
    }

    void MockPublisher::test_constant_accel(geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        const double accel = 0.5; // m/s^2
        const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
        const double t = static_cast<double>(count_) * dt;
        const double position_x = 0.5 * accel * t * t;

        msg.pose.pose.position.x = position_x + gaussian(0.0, default_err_pos);
        msg.pose.pose.position.y = gaussian(0.0, default_err_pos);
        msg.pose.pose.position.z = gaussian(0.0, default_err_pos);

        auto &quat = msg.pose.pose.orientation;
        quat.x = gaussian(0.0, default_err_rest);
        quat.y = gaussian(0.0, default_err_rest);
        quat.z = gaussian(0.0, default_err_rest);
        quat.w = 1.0 + gaussian(0.0, default_err_rest);
        normalize_quaternion(quat);
        set_pose_covariance(msg, default_err_pos, default_err_rest);
    }

    void MockPublisher::test_constant_accel_control(geometry_msgs::msg::TwistStamped &msg)
    {
        // Match test_constant_accel: vx(t) = a * t
        const double accel = 0.5; // m/s^2 (same as in test_constant_accel)
        const double dt = (vio_rate_hz_ > 0.0) ? (1.0 / vio_rate_hz_) : 0.01;
        const double t = static_cast<double>(count_) * dt;
        const double velocity_x = accel * t;

        msg.twist.linear.x = velocity_x;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.0;
    }

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

        static const std::array<std::string, 4> valid_scenarios = {"rest", "const_vel", "turn", "accel"};
        if (std::find(valid_scenarios.begin(), valid_scenarios.end(), scenario_) == valid_scenarios.end())
        {
            RCLCPP_WARN(this->get_logger(), "Unknown scenario '%s', defaulting to const_vel", scenario_.c_str());
            scenario_ = "const_vel";
        }

        auto clamp_rate = [this](double rate, double fallback, const char *name)
        {
            if (rate <= 0.0)
            {
                RCLCPP_WARN(this->get_logger(), "%s must be > 0 Hz. Using %.2f Hz.", name, fallback);
                return fallback;
            }
            return rate;
        };

        vio_rate_hz_ = clamp_rate(vio_rate_hz_, 100.0, "vio_rate_hz");
        vision_rate_hz_ = clamp_rate(vision_rate_hz_, 5.0, "vision_rate_hz");

        if (duration_sec_ <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "duration_sec must be > 0. Using 1.0s.");
            duration_sec_ = 1.0;
        }

        vio_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/vio/odometry", rclcpp::SensorDataQoS());

        vision_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/vision/pose", rclcpp::SensorDataQoS());

        control_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/control/command", rclcpp::QoS(10));

        auto to_duration = [](double seconds)
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds));
        };

        vio_timer_ = this->create_wall_timer(
            to_duration(1.0 / vio_rate_hz_), [this]()
            {
                nav_msgs::msg::Odometry msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = "odom";
                msg.child_frame_id = "base_link";

                if (scenario_ == "rest")
                {
                    test_resting_drone(msg);
                }
                else if (scenario_ == "turn")
                {
                    test_constant_turn(msg);
                }
                else if (scenario_ == "accel")
                {
                    test_constant_accel(msg);
                }
                else
                {
                    test_constant_velocity_drone(msg);
                }

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
                vio_publisher_->publish(msg);
                ++count_; });

        vision_timer_ = this->create_wall_timer(
            to_duration(1.0 / vision_rate_hz_), [this]()
            {
                geometry_msgs::msg::PoseWithCovarianceStamped msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = "odom";

                if (scenario_ == "rest")
                {
                    test_resting_drone(msg);
                }
                else if (scenario_ == "turn")
                {
                    test_constant_turn(msg);
                }
                else if (scenario_ == "accel")
                {
                    test_constant_accel(msg);
                }
                else
                {
                    test_constant_velocity_drone(msg);
                }

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
                vision_publisher_->publish(msg); });

        control_timer_ = this->create_wall_timer(
            to_duration(1.0 / control_rate_hz_), [this]()
            {
                geometry_msgs::msg::TwistStamped msg;
                msg.header.stamp = this->now();
                // Control is in base_link frame (what robot_localization expects)
                msg.header.frame_id = "base_link";

                if (scenario_ == "rest")
                {
                    test_resting_control(msg);
                }
                else if (scenario_ == "const_vel")
                {
                    test_constant_velocity_control(msg);
                }
                else if (scenario_ == "turn")
                {
                    test_constant_turn_control(msg);
                }
                else if (scenario_ == "accel")
                {
                    test_constant_accel_control(msg);
                }

                RCLCPP_INFO(this->get_logger(),
                            "Publishing control: lin (%.3f, %.3f, %.3f), ang (%.3f, %.3f, %.3f)",
                            msg.twist.linear.x,  msg.twist.linear.y,  msg.twist.linear.z,
                            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);

                control_publisher_->publish(msg); });

        shutdown_timer_ = this->create_wall_timer(
            to_duration(duration_sec_), [this]()
            {
                RCLCPP_INFO(this->get_logger(), "Scenario complete, shutting down.");
                if (shutdown_timer_)
                {
                    shutdown_timer_->cancel();
                }
                rclcpp::shutdown(); });
    }

} // namespace ekf

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ekf::MockPublisher>());
    rclcpp::shutdown();
    return 0;
}
