#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class GazeboGroundThruth : public rclcpp::Node
{
public:
    GazeboGroundThruth() : Node("gazebo_ground_thruth"), marker_id_(0)
    {
        // Declare parameter
        this->declare_parameter<std::string>("world_frame", "world");

        // Get parameter
        world_frame_ = this->get_parameter("world_frame").as_string();

        // Subscribe to odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry",
            10,
            std::bind(&GazeboGroundThruth::odomCallback, this, std::placeholders::_1));

        // Publisher for position trail (spheres)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/gazebo_ground_thruth",
            10);

        // Publisher for orientation arrow
        arrow_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/gazebo_ground_thruth_arrow",
            10);

        // Publisher for 3D coordinate frame
        frame_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/gazebo_ground_thruth_frame",
            10);

        RCLCPP_INFO(this->get_logger(), "gazebo_ground_thruth node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /odometry");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /gazebo_ground_thruth (position trail)");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /gazebo_ground_thruth_arrow (orientation arrow)");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /gazebo_ground_thruth_frame (coordinate frame)");
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

        // Create purple arrow for orientation
        auto arrow = std::make_unique<visualization_msgs::msg::Marker>();
        arrow->header.frame_id = world_frame_;
        arrow->header.stamp = msg->header.stamp;
        arrow->ns = "orientation_arrow";
        arrow->id = 0; // Only one arrow at a time
        arrow->type = visualization_msgs::msg::Marker::ARROW;
        arrow->action = visualization_msgs::msg::Marker::ADD;

        // Position and orientation from odometry
        arrow->pose.position.x = msg->pose.pose.position.x;
        arrow->pose.position.y = msg->pose.pose.position.y;
        arrow->pose.position.z = msg->pose.pose.position.z;
        arrow->pose.orientation = msg->pose.pose.orientation;

        // Purple color
        arrow->color.r = 0.5;
        arrow->color.g = 0.0;
        arrow->color.b = 0.5;
        arrow->color.a = 1.0;

        // Arrow size (matching ArUco visualization)
        arrow->scale.x = 1.0; // Total length (shaft + head)
        arrow->scale.y = 0.05; // Shaft diameter
        arrow->scale.z = 0.1; // Head diameter

        arrow->lifetime = rclcpp::Duration(0, 0); // Permanent

        arrow_pub_->publish(std::move(arrow));

        // Create RGB coordinate frame
        auto frame = std::make_unique<visualization_msgs::msg::Marker>();
        frame->header.frame_id = world_frame_;
        frame->header.stamp = msg->header.stamp;
        frame->ns = "coordinate_frame";
        frame->id = 0; // Only one frame at a time
        frame->type = visualization_msgs::msg::Marker::LINE_LIST;
        frame->action = visualization_msgs::msg::Marker::ADD;

        // Position and orientation from odometry
        frame->pose.position.x = msg->pose.pose.position.x;
        frame->pose.position.y = msg->pose.pose.position.y;
        frame->pose.position.z = msg->pose.pose.position.z;
        frame->pose.orientation = msg->pose.pose.orientation;

        // Define three axes as line pairs (origin to axis endpoint)
        geometry_msgs::msg::Point origin, x_end, y_end, z_end;
        origin.x = 0.0; origin.y = 0.0; origin.z = 0.0;
        x_end.x = 0.2; x_end.y = 0.0; x_end.z = 0.0; // X-axis (Red)
        y_end.x = 0.0; y_end.y = 0.2; y_end.z = 0.0; // Y-axis (Green)
        z_end.x = 0.0; z_end.y = 0.0; z_end.z = 0.2; // Z-axis (Blue)

        // X-axis line
        frame->points.push_back(origin);
        frame->points.push_back(x_end);
        // Y-axis line
        frame->points.push_back(origin);
        frame->points.push_back(y_end);
        // Z-axis line
        frame->points.push_back(origin);
        frame->points.push_back(z_end);

        // Colors for each vertex (need 6 colors: 2 per line)
        std_msgs::msg::ColorRGBA red, green, blue;
        red.r = 1.0; red.g = 0.0; red.b = 0.0; red.a = 1.0;
        green.r = 0.0; green.g = 1.0; green.b = 0.0; green.a = 1.0;
        blue.r = 0.0; blue.g = 0.0; blue.b = 1.0; blue.a = 1.0;

        // X-axis colors (red)
        frame->colors.push_back(red);
        frame->colors.push_back(red);
        // Y-axis colors (green)
        frame->colors.push_back(green);
        frame->colors.push_back(green);
        // Z-axis colors (blue)
        frame->colors.push_back(blue);
        frame->colors.push_back(blue);

        // Line width
        frame->scale.x = 0.02;

        frame->lifetime = rclcpp::Duration(0, 0); // Permanent

        frame_pub_->publish(std::move(frame));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frame_pub_;
    int marker_id_;
    std::string world_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboGroundThruth>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
