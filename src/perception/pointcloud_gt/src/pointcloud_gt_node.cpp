#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

class PointCloudGT : public rclcpp::Node
{
public:
    PointCloudGT() : Node("pointcloud_GT"), marker_id_(0)
    {
        // Declare parameter
        this->declare_parameter<std::string>("world_frame", "world");

        // Get parameter
        world_frame_ = this->get_parameter("world_frame").as_string();

        // Subscribe to odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry",
            10,
            std::bind(&PointCloudGT::odomCallback, this, std::placeholders::_1));

        // Publisher for Marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/PointCloud_GT",
            10);

        RCLCPP_INFO(this->get_logger(), "PointCloud_GT node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /odometry");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /PointCloud_GT");
        RCLCPP_INFO(this->get_logger(), "World frame: %s", world_frame_.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Create a sphere marker for this position
        auto marker = std::make_unique<visualization_msgs::msg::Marker>();

        marker->header.frame_id = world_frame_;
        marker->header.stamp = msg->header.stamp;
        marker->ns = "odometry_trail";
        marker->id = marker_id_++;
        marker->type = visualization_msgs::msg::Marker::SPHERE;
        marker->action = visualization_msgs::msg::Marker::ADD;

        // Position from odometry
        marker->pose.position.x = msg->pose.pose.position.x;
        marker->pose.position.y = msg->pose.pose.position.y;
        marker->pose.position.z = msg->pose.pose.position.z;
        marker->pose.orientation.w = 1.0;

        // Blue color (different from ArUco green)
        marker->color.r = 0.0;
        marker->color.g = 0.0;
        marker->color.b = 1.0;
        marker->color.a = 1.0;

        // Size of the sphere
        marker->scale.x = 0.08;
        marker->scale.y = 0.08;
        marker->scale.z = 0.08;

        marker->lifetime = rclcpp::Duration(0, 0); // Permanent marker

        marker_pub_->publish(std::move(marker));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    int marker_id_;
    std::string world_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudGT>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
