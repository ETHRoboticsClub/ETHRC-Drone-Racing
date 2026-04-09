#ifndef ARUCO_VISUALIZATION_HELPER_HPP
#define ARUCO_VISUALIZATION_HELPER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <memory>
#include <functional>

namespace aruco_pose_estimation {

struct VisualizationConfig
{
    // Trail marker settings
    double trail_marker_interval_sec = 0.1;
    double trail_marker_size = 0.08;
    
    // Debug text settings
    double debug_text_scale = 0.6;
    int debug_text_thickness = 2;
    int debug_text_line_height = 30;
    
    // Visualization flags
    bool show_rejected_candidates = true;
    bool show_corner_points = true;
    bool show_detector_params = true;
    bool show_marker_ids = true;
};

class VisualizationHelper
{
public:
    explicit VisualizationHelper(const VisualizationConfig& config = VisualizationConfig());

    /**
     * @brief Publish trail marker for trajectory visualization
     */
    void publishTrailMarker(
        const cv::Vec3d& position,
        const rclcpp::Time& timestamp,
        const std::string& world_frame,
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub);

    /**
     * @brief Publish diagnostic information
     */
    void publishDiagnostics(
        const rclcpp::Time& timestamp,
        int markers_detected,
        int markers_used,
        const std::vector<int>& marker_ids,
        bool pose_valid,
        double processing_time_ms,
        const cv::Vec3d& drone_position,
        const tf2::Quaternion& drone_quat,
        size_t total_markers_in_map,
        bool is_initialized,
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub,
        std::shared_ptr<rclcpp::Logger> logger);

    /**
     * @brief Draw debug information on image frame
     */
    void drawDebugImage(
        cv::Mat& frame,
        const std::vector<std::vector<cv::Point2f>>& marker_corners,
        const std::vector<int>& marker_ids,
        const std::vector<int>& used_markers,
        const std::vector<std::vector<cv::Point2f>>& rejected_candidates,
        const cv::Vec3d& camera_position,
        const tf2::Quaternion& camera_quat,
        bool is_initialized,
        const cv::Ptr<cv::aruco::DetectorParameters>& detector_params,
        std::function<bool(const cv::Vec3d&, const tf2::Quaternion&, cv::Vec3d&, tf2::Quaternion&)> transform_func);

    /**
     * @brief Publish debug image
     */
    void publishDebugImage(
        const cv::Mat& frame,
        const rclcpp::Time& timestamp,
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub);

    /**
     * @brief Get current configuration
     */
    const VisualizationConfig& getConfig() const { return config_; }

    /**
     * @brief Update configuration
     */
    void setConfig(const VisualizationConfig& config) { config_ = config; }

private:
    VisualizationConfig config_;
    uint32_t trail_marker_id_ = 0;
    rclcpp::Time last_trail_time_;

    /**
     * @brief Add a key-value pair to diagnostic status
     */
    void addDiagnosticKeyValue(
        diagnostic_msgs::msg::DiagnosticStatus& status,
        const std::string& key,
        const std::string& value);

    /**
     * @brief Draw detected markers and corners
     */
    void drawMarkers(
        cv::Mat& frame,
        const std::vector<std::vector<cv::Point2f>>& marker_corners,
        const std::vector<int>& marker_ids);

    /**
     * @brief Draw rejected candidates
     */
    void drawRejectedCandidates(
        cv::Mat& frame,
        const std::vector<std::vector<cv::Point2f>>& rejected_candidates);

    /**
     * @brief Draw text information
     */
    void drawTextInfo(
        cv::Mat& frame,
        const cv::Vec3d& camera_position,
        const tf2::Quaternion& camera_quat,
        const std::vector<int>& used_markers,
        const std::vector<int>& marker_ids,
        const std::vector<std::vector<cv::Point2f>>& rejected_candidates,
        bool is_initialized,
        const cv::Ptr<cv::aruco::DetectorParameters>& detector_params,
        std::function<bool(const cv::Vec3d&, const tf2::Quaternion&, cv::Vec3d&, tf2::Quaternion&)> transform_func);
};

} // namespace aruco_pose_estimation

#endif // ARUCO_VISUALIZATION_HELPER_HPP
