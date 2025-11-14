#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "aruco_pose_estimation/marker_map.hpp"

class ArucoPoseEstimationNode : public rclcpp::Node
{
public:
    ArucoPoseEstimationNode() : Node("aruco_pose_estimation_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("camera_calibration_file", "");
        this->declare_parameter<std::string>("marker_map_file", "");
        this->declare_parameter<std::string>("world_frame", "world");
        this->declare_parameter<std::string>("camera_frame", "camera");
        this->declare_parameter<bool>("publish_tf", true);
        this->declare_parameter<bool>("publish_debug_image", true);

        // Get parameters
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        std::string camera_calib_file = this->get_parameter("camera_calibration_file").as_string();
        std::string marker_map_file = this->get_parameter("marker_map_file").as_string();
        world_frame_ = this->get_parameter("world_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();

        // Load camera calibration
        if (!loadCameraCalibration(camera_calib_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera calibration!");
            rclcpp::shutdown();
            return;
        }

        // Load marker map
        if (!marker_map_.loadFromFile(marker_map_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load marker map!");
            rclcpp::shutdown();
            return;
        }
        // Subscribe to camera
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10,
            std::bind(&ArucoPoseEstimationNode::imageCallback, this, std::placeholders::_1));

        // Initialize ArUco detector
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
        detector_params_ = cv::aruco::DetectorParameters::create();

        // Create publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/aruco/camera_pose", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/aruco/visualization_markers", 10);

        if (publish_debug_image_) {
            debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/aruco/debug_image", 10);
        }

        // Create TF broadcaster
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }

        // Initialize path trail tracking
        last_trail_time_ = this->now();
        trail_marker_id_ = 0;


        RCLCPP_INFO(this->get_logger(), "ArUco Pose Estimation Node initialized");
        RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Marker map: %zu markers loaded", marker_map_.getMarkerCount());
        RCLCPP_INFO(this->get_logger(), "World frame: %s, Camera frame: %s",
                    world_frame_.c_str(), camera_frame_.c_str());
    }

private:
    void publishTrailMarker(const cv::Vec3d& position, const rclcpp::Time& timestamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker trail_marker;

        trail_marker.header.frame_id = world_frame_;
        trail_marker.header.stamp = timestamp;
        trail_marker.ns = "path_trail";
        trail_marker.id = trail_marker_id_++;
        trail_marker.type = visualization_msgs::msg::Marker::SPHERE;
        trail_marker.action = visualization_msgs::msg::Marker::ADD;

        // Position at camera location
        trail_marker.pose.position.x = position[0];
        trail_marker.pose.position.y = position[1];
        trail_marker.pose.position.z = position[2];
        trail_marker.pose.orientation.w = 1.0;

        // Green color
        trail_marker.color.r = 0.0;
        trail_marker.color.g = 1.0;
        trail_marker.color.b = 0.0;
        trail_marker.color.a = 1.0;

        // Size of the sphere
        trail_marker.scale.x = 0.08;
        trail_marker.scale.y = 0.08;
        trail_marker.scale.z = 0.08;

        trail_marker.lifetime = rclcpp::Duration(0, 0); // Permanent marker

        marker_array.markers.push_back(trail_marker);
        marker_pub_->publish(marker_array);
    }

    bool loadCameraCalibration(const std::string& filepath)
    {
        cv::FileStorage fs(filepath, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera calibration file: %s", filepath.c_str());
            return false;
        }

        fs["camera_matrix"] >> camera_matrix_;
        fs["distortion_coefficients"] >> dist_coeffs_;
        fs.release();

        RCLCPP_INFO(this->get_logger(), "Loaded camera calibration from: %s", filepath.c_str());
        return true;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;

        // Detect ArUco markers
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
        cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, detector_params_);

        if (marker_ids.empty()) {
            return; // No markers detected
        }

        // Estimate camera pose using marker map with multi-marker fusion
        cv::Vec3d camera_position, camera_orientation;
        std::vector<int> used_markers;

        bool pose_estimated = marker_map_.estimateCameraPose(
            marker_ids, marker_corners,
            camera_matrix_, dist_coeffs_,
            camera_position, camera_orientation,
            used_markers
        );

        if (!pose_estimated) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Detected markers but could not estimate camera pose");
            return;
        }

        // Convert rotation vector to quaternion
        cv::Mat rotation_matrix;
        cv::Rodrigues(camera_orientation, rotation_matrix);

        tf2::Matrix3x3 tf_rotation(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
        );

        tf2::Quaternion tf_quat;
        tf_rotation.getRotation(tf_quat);

        // Publish pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = world_frame_;
        pose_msg.pose.position.x = camera_position[0];
        pose_msg.pose.position.y = camera_position[1];
        pose_msg.pose.position.z = camera_position[2];
        pose_msg.pose.orientation.x = tf_quat.x();
        pose_msg.pose.orientation.y = tf_quat.y();
        pose_msg.pose.orientation.z = tf_quat.z();
        pose_msg.pose.orientation.w = tf_quat.w();
        pose_pub_->publish(pose_msg);

        // Publish trail marker every 2 seconds
        rclcpp::Time current_time = this->now();
        if ((current_time - last_trail_time_).seconds() >= 2.0) {
            publishTrailMarker(camera_position, msg->header.stamp);
            last_trail_time_ = current_time;
        }

        // Publish TF transform
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = msg->header.stamp;
            transform.header.frame_id = world_frame_;
            transform.child_frame_id = camera_frame_;
            transform.transform.translation.x = camera_position[0];
            transform.transform.translation.y = camera_position[1];
            transform.transform.translation.z = camera_position[2];
            transform.transform.rotation.x = tf_quat.x();
            transform.transform.rotation.y = tf_quat.y();
            transform.transform.rotation.z = tf_quat.z();
            transform.transform.rotation.w = tf_quat.w();
            tf_broadcaster_->sendTransform(transform);
        }

        // Publish debug image
        if (publish_debug_image_) {
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

            // Draw info text
            cv::Vec3d euler = marker_map::rotationVectorToEulerAngles(camera_orientation);
            cv::Vec3d euler_deg(euler[0] * 180.0 / CV_PI, euler[1] * 180.0 / CV_PI, euler[2] * 180.0 / CV_PI);

            std::stringstream ss;
            ss << std::fixed << std::setprecision(3);
            ss << "Pos: [" << camera_position[0] << ", " << camera_position[1] << ", " << camera_position[2] << "] m";
            cv::putText(frame, ss.str(), cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            ss.str("");
            ss << "Markers: " << used_markers.size() << "/" << marker_ids.size();
            cv::putText(frame, ss.str(), cv::Point(10, 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(
                msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
            debug_image_pub_->publish(*debug_msg);
        }

        // Log info (throttled)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Camera pose: [%.3f, %.3f, %.3f] orientation: [%.3f, %.3f, %.3f] (using %zu/%zu markers)",
                            camera_position[0], camera_position[1], camera_position[2],
                            camera_orientation[0], camera_orientation[1], camera_orientation[2]);
    }

    // Member variables
    marker_map::MarkerMap marker_map_;
    cv::Mat camera_matrix_, dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    std::string world_frame_;
    std::string camera_frame_;
    bool publish_tf_;
    bool publish_debug_image_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Trail tracking
    rclcpp::Time last_trail_time_;
    int trail_marker_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoPoseEstimationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
