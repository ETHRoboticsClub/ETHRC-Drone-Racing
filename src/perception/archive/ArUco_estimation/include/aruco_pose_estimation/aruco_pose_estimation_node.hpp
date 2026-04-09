#ifndef ARUCO_POSE_ESTIMATION_NODE_HPP
#define ARUCO_POSE_ESTIMATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <chrono>
#include <sstream>
#include <iomanip>
#include "aruco_pose_estimation/pose_estimation_utils.hpp"
#include "aruco_pose_estimation/visualization_helper.hpp"

namespace aruco_pose_estimation {

class ArucoPoseEstimationNode : public rclcpp::Node
{
public:
    ArucoPoseEstimationNode();

private:
    // Initialization
    bool initializeCalibration();
    void initializeArucoDetector();
    void createPublishersAndSubscribers();

    // Transform
    bool transformCameraToDrone(const cv::Vec3d& camera_position, const tf2::Quaternion& camera_quat,
                                cv::Vec3d& drone_position, tf2::Quaternion& drone_quat);

    // Smoothing methods
    cv::Vec3d smoothPosition(const cv::Vec3d& new_position, const cv::Vec3d& prev_position, double alpha);
    tf2::Quaternion smoothOrientation(const tf2::Quaternion& new_quat, const tf2::Quaternion& prev_quat, double alpha);

    // Publishing
    void publishPoses(const cv::Vec3d& camera_position, const tf2::Quaternion& camera_quat,
                      const rclcpp::Time& timestamp);
    void publishTFTransforms(const cv::Vec3d& camera_position, const tf2::Quaternion& camera_quat,
                             const rclcpp::Time& timestamp);

    // Image processing
    cv::Mat convertRosImageToCV(const sensor_msgs::msg::Image::SharedPtr& msg);
    void detectArucoMarkers(const cv::Mat& frame, std::vector<int>& marker_ids,
                            std::vector<std::vector<cv::Point2f>>& marker_corners);
    bool estimateAndValidatePose(const std::vector<int>& marker_ids,
                                  const std::vector<std::vector<cv::Point2f>>& marker_corners,
                                  cv::Vec3d& camera_position, cv::Vec3d& camera_orientation,
                                  std::vector<int>& used_markers);

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // Member variables
    pose_estimation_utils::MarkerMap marker_map_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    std::string camera_topic_;
    std::string marker_map_file_;
    std::string camera_calibration_file_;
    std::string world_frame_;
    std::string camera_frame_;
    std::string base_link_frame_;
    bool publish_debug_image_;
    bool is_initialized_;

    int min_markers_for_init_;

    // ArUco detector parameter values
    int corner_refinement_win_size_;
    int corner_refinement_max_iterations_;
    double corner_refinement_min_accuracy_;
    int adaptive_thresh_win_size_min_;
    int adaptive_thresh_win_size_max_;
    int adaptive_thresh_win_size_step_;
    double min_marker_perimeter_rate_;
    double max_marker_perimeter_rate_;
    double polygonal_approx_accuracy_rate_;
    double min_marker_distance_rate_;

    //Runtime state
    std::vector<std::vector<cv::Point2f>> rejected_candidates_;
    rclcpp::Time last_trail_time_;

    // Smoothing parameters and state
    bool use_smoothing_;
    double position_smoothing_alpha_;
    double orientation_smoothing_alpha_;
    cv::Vec3d prev_camera_position_;
    tf2::Quaternion prev_camera_quat_;
    bool has_previous_pose_;

    // Lucas-Kanade tracking members
    cv::Mat prev_gray_;
    std::map<int, std::vector<cv::Point2f>> tracked_corners_;
    std::map<int, int> tracking_age_;  // How many frames each marker has been tracked
    bool use_optical_flow_;
    int max_tracking_age_;
    int redetect_interval_;  // Force full detection every N frames
    int frames_since_detection_;
    
    // LK parameters
    cv::Size lk_win_size_;
    int lk_max_level_;
    cv::TermCriteria lk_term_criteria_;

    // Motion blur parameters
    bool motion_blur_enabled_;
    int motion_blur_kernel_size_;
    int motion_blur_major_axis_;
    int motion_blur_minor_axis_;
    double motion_blur_angle_;

    // Visualization helper
    std::shared_ptr<VisualizationHelper> visualization_helper_;
    
    // Methods
    void trackCornersWithLK(const cv::Mat& gray, 
                           std::vector<int>& marker_ids,
                           std::vector<std::vector<cv::Point2f>>& marker_corners);
    void updateTrackedMarkers(const std::vector<int>& detected_ids,
                             const std::vector<std::vector<cv::Point2f>>& detected_corners);
    bool validateTrackedCorners(const std::vector<cv::Point2f>& corners, 
                               const cv::Size& frame_size);

    //publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr drone_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace aruco_pose_estimation

#endif // ARUCO_POSE_ESTIMATION_NODE_HPP
