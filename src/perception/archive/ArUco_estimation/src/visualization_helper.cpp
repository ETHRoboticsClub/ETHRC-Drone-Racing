#include "aruco_pose_estimation/visualization_helper.hpp"
#include "aruco_pose_estimation/pose_estimation_utils.hpp"

namespace aruco_pose_estimation {

VisualizationHelper::VisualizationHelper(const VisualizationConfig& config)
    : config_(config), trail_marker_id_(0)
{
}

void VisualizationHelper::publishTrailMarker(
    const cv::Vec3d& position,
    const rclcpp::Time& timestamp,
    const std::string& world_frame,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub)
{
    auto trail_marker = std::make_unique<visualization_msgs::msg::Marker>();
    trail_marker->header.frame_id = world_frame;
    trail_marker->header.stamp = timestamp;
    trail_marker->ns = "path_trail";
    trail_marker->id = trail_marker_id_++;
    trail_marker->type = visualization_msgs::msg::Marker::SPHERE;
    trail_marker->action = visualization_msgs::msg::Marker::ADD;

    trail_marker->pose.position.x = position[0];
    trail_marker->pose.position.y = position[1];
    trail_marker->pose.position.z = position[2];
    trail_marker->pose.orientation.w = 1.0;

    trail_marker->color.r = 0.0;
    trail_marker->color.g = 1.0;
    trail_marker->color.b = 0.0;
    trail_marker->color.a = 1.0;

    trail_marker->scale.x = config_.trail_marker_size;
    trail_marker->scale.y = config_.trail_marker_size;
    trail_marker->scale.z = config_.trail_marker_size;

    trail_marker->lifetime = rclcpp::Duration(0, 0);

    marker_pub->publish(std::move(trail_marker));
}

void VisualizationHelper::addDiagnosticKeyValue(
    diagnostic_msgs::msg::DiagnosticStatus& status,
    const std::string& key,
    const std::string& value)
{
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
}

void VisualizationHelper::publishDiagnostics(
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
    std::shared_ptr<rclcpp::Logger> logger)
{
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = timestamp;

    diagnostic_msgs::msg::DiagnosticStatus diag_status;
    diag_status.name = "aruco_pose_estimation";
    diag_status.hardware_id = "camera";

    if (!pose_valid || markers_used == 0) {
        diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_status.message = "No valid pose estimate";
    } else {
        diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diag_status.message = "Pose estimation OK";
    }

    addDiagnosticKeyValue(diag_status, "markers_detected", std::to_string(markers_detected));
    addDiagnosticKeyValue(diag_status, "markers_used", std::to_string(markers_used));

    std::string ids_str;
    for (size_t i = 0; i < marker_ids.size(); ++i) {
        ids_str += std::to_string(marker_ids[i]);
        if (i < marker_ids.size() - 1) ids_str += ",";
    }
    addDiagnosticKeyValue(diag_status, "marker_ids", ids_str);
    addDiagnosticKeyValue(diag_status, "processing_time_ms", std::to_string(processing_time_ms));
    addDiagnosticKeyValue(diag_status, "is_initialized", is_initialized ? "true" : "false");

    diag_array.status.push_back(diag_status);
    diagnostics_pub->publish(diag_array);

    // Print nice formatted output to console
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "[" << timestamp.seconds() << "s] ";
    ss << "Pos: [" << drone_position[0] << ", " << drone_position[1] << ", " << drone_position[2] << "] ";
    ss << "Orient: [" << drone_quat.x() << ", " << drone_quat.y() << ", " << drone_quat.z() << ", " << drone_quat.w() << "] ";
    ss << "Markers: " << markers_used << "/" << markers_detected << "/" << total_markers_in_map;

    RCLCPP_INFO(*logger, "%s", ss.str().c_str());
}

void VisualizationHelper::drawMarkers(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& marker_corners,
    const std::vector<int>& marker_ids)
{
    if (!config_.show_marker_ids) return;

    // Draw detected markers with IDs
    cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

    if (!config_.show_corner_points) return;

    // Draw corners for detected markers (individual corner points)
    for (const auto& corners : marker_corners) {
        for (size_t i = 0; i < corners.size(); ++i) {
            // Draw each corner with different colors
            cv::Scalar color;
            switch(i) {
                case 0: color = cv::Scalar(255, 0, 0); break;    // Blue - top-left
                case 1: color = cv::Scalar(0, 255, 0); break;    // Green - top-right
                case 2: color = cv::Scalar(0, 0, 255); break;    // Red - bottom-right
                case 3: color = cv::Scalar(255, 255, 0); break;  // Cyan - bottom-left
                default: color = cv::Scalar(255, 255, 255); break;
            }
            cv::circle(frame, corners[i], 5, color, -1);
        }
    }
}

void VisualizationHelper::drawRejectedCandidates(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& rejected_candidates)
{
    if (!config_.show_rejected_candidates || rejected_candidates.empty()) return;

    // Draw rejected candidates in yellow/orange
    for (const auto& rejected : rejected_candidates) {
        std::vector<cv::Point> polygon;
        for (const auto& point : rejected) {
            polygon.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
        }
        cv::polylines(frame, polygon, true, cv::Scalar(0, 165, 255), 2); // Orange
    }
}

void VisualizationHelper::drawTextInfo(
    cv::Mat& frame,
    const cv::Vec3d& camera_position,
    const tf2::Quaternion& camera_quat,
    const std::vector<int>& used_markers,
    const std::vector<int>& marker_ids,
    const std::vector<std::vector<cv::Point2f>>& rejected_candidates,
    bool is_initialized,
    const cv::Ptr<cv::aruco::DetectorParameters>& detector_params,
    std::function<bool(const cv::Vec3d&, const tf2::Quaternion&, cv::Vec3d&, tf2::Quaternion&)> transform_func)
{
    int line_num = 1;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);

    // Drone position
    cv::Vec3d drone_position;
    tf2::Quaternion drone_quat;
    if (transform_func(camera_position, camera_quat, drone_position, drone_quat)) {
        ss << "Drone Pos: [" << drone_position[0] << ", " << drone_position[1] << ", " << drone_position[2] << "] m";
    } else {
        ss << "Drone Pos: [TF unavailable]";
    }
    cv::putText(frame, ss.str(), cv::Point(10, config_.debug_text_line_height * line_num),
               cv::FONT_HERSHEY_SIMPLEX, config_.debug_text_scale, cv::Scalar(0, 255, 0), config_.debug_text_thickness);

    // Markers info
    line_num++;
    ss.str("");
    ss << "Markers: " << used_markers.size() << "/" << marker_ids.size() << " | Rejected: " << rejected_candidates.size();
    cv::putText(frame, ss.str(), cv::Point(10, config_.debug_text_line_height * line_num),
               cv::FONT_HERSHEY_SIMPLEX, config_.debug_text_scale, cv::Scalar(0, 255, 0), config_.debug_text_thickness);

    // Status
    line_num++;
    ss.str("");
    ss << "Status: " << (is_initialized ? "TRACKING" : "INITIALIZING");
    cv::putText(frame, ss.str(), cv::Point(10, config_.debug_text_line_height * line_num),
               cv::FONT_HERSHEY_SIMPLEX, config_.debug_text_scale, cv::Scalar(0, 255, 0), config_.debug_text_thickness);

    if (!config_.show_detector_params) return;

    // Display detector parameters
    line_num++;
    ss.str("");
    ss << "Params: WinSize=" << detector_params->cornerRefinementWinSize
       << " AdaptThresh=" << detector_params->adaptiveThreshWinSizeMin << "-" << detector_params->adaptiveThreshWinSizeMax;
    cv::putText(frame, ss.str(), cv::Point(10, config_.debug_text_line_height * line_num),
               cv::FONT_HERSHEY_SIMPLEX, config_.debug_text_scale, cv::Scalar(0, 255, 0), config_.debug_text_thickness);

    line_num++;
    ss.str("");
    ss << "       MinPerim=" << detector_params->minMarkerPerimeterRate
       << " MaxPerim=" << detector_params->maxMarkerPerimeterRate
       << " PolyApprox=" << detector_params->polygonalApproxAccuracyRate;
    cv::putText(frame, ss.str(), cv::Point(10, config_.debug_text_line_height * line_num),
               cv::FONT_HERSHEY_SIMPLEX, config_.debug_text_scale, cv::Scalar(0, 255, 0), config_.debug_text_thickness);
}

void VisualizationHelper::drawDebugImage(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& marker_corners,
    const std::vector<int>& marker_ids,
    const std::vector<int>& used_markers,
    const std::vector<std::vector<cv::Point2f>>& rejected_candidates,
    const cv::Vec3d& camera_position,
    const tf2::Quaternion& camera_quat,
    bool is_initialized,
    const cv::Ptr<cv::aruco::DetectorParameters>& detector_params,
    std::function<bool(const cv::Vec3d&, const tf2::Quaternion&, cv::Vec3d&, tf2::Quaternion&)> transform_func)
{
    drawMarkers(frame, marker_corners, marker_ids);
    drawRejectedCandidates(frame, rejected_candidates);
    drawTextInfo(frame, camera_position, camera_quat, used_markers, marker_ids,
                 rejected_candidates, is_initialized, detector_params, transform_func);
}

void VisualizationHelper::publishDebugImage(
    const cv::Mat& frame,
    const rclcpp::Time& timestamp,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub)
{
    sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
    debug_msg->header.stamp = timestamp;
    debug_image_pub->publish(*debug_msg);
}

} // namespace aruco_pose_estimation
