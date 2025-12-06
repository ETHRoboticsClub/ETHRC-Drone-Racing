#include "aruco_pose_estimation/aruco_pose_estimation_node.hpp"

namespace aruco_pose_estimation {

ArucoPoseEstimationNode::ArucoPoseEstimationNode()
    : Node("aruco_pose_estimation_node"), is_initialized_(false), has_previous_pose_(false)
    {
        // Parameters
        camera_topic_ = this->declare_parameter("camera_topic", "/cam0/image_raw");
        marker_map_file_ = this->declare_parameter("marker_map_file", "");
        camera_calibration_file_ = this->declare_parameter("camera_calibration_file", "");
        world_frame_ = this->declare_parameter("world_frame", "world");
        camera_frame_ = this->declare_parameter("camera_frame", "front_camera_link");
        base_link_frame_ = this->declare_parameter("base_link_frame", "base_link");
        publish_debug_image_ = this->declare_parameter("publish_debug_image", true);
        min_markers_for_init_ = this->declare_parameter("min_markers_for_init", 1);

        // ArUco detector parameters (nested under aruco_detector)
        corner_refinement_win_size_ = this->declare_parameter("aruco_detector.corner_refinement_win_size", 0);
        corner_refinement_max_iterations_ = this->declare_parameter("aruco_detector.corner_refinement_max_iterations", 0);
        corner_refinement_min_accuracy_ = this->declare_parameter("aruco_detector.corner_refinement_min_accuracy", 0.0);
        adaptive_thresh_win_size_min_ = this->declare_parameter("aruco_detector.adaptive_thresh_win_size_min", 7);
        adaptive_thresh_win_size_max_ = this->declare_parameter("aruco_detector.adaptive_thresh_win_size_max", 23);
        adaptive_thresh_win_size_step_ = this->declare_parameter("aruco_detector.adaptive_thresh_win_size_step", 10);
        min_marker_perimeter_rate_ = this->declare_parameter("aruco_detector.min_marker_perimeter_rate", 0.04);
        max_marker_perimeter_rate_ = this->declare_parameter("aruco_detector.max_marker_perimeter_rate", 4.0);
        polygonal_approx_accuracy_rate_ = this->declare_parameter("aruco_detector.polygonal_approx_accuracy_rate", 0.09);
        min_marker_distance_rate_ = this->declare_parameter("aruco_detector.min_marker_distance_rate", 0.1);

        // Smoothing parameters (nested under pose_estimation)
        use_smoothing_ = this->declare_parameter("pose_estimation.use_smoothing", true);
        position_smoothing_alpha_ = this->declare_parameter("pose_estimation.position_smoothing_alpha", 0.3);
        orientation_smoothing_alpha_ = this->declare_parameter("pose_estimation.orientation_smoothing_alpha", 0.3);

        // Lucas-Kanade tracking parameters (nested under optical_flow)
        use_optical_flow_ = this->declare_parameter("optical_flow.enabled", true);
        max_tracking_age_ = this->declare_parameter("optical_flow.max_tracking_age", 10);
        redetect_interval_ = this->declare_parameter("optical_flow.redetect_interval", 30);
        frames_since_detection_ = 0;
        
        // LK algorithm parameters (nested under optical_flow)
        int lk_win_size = this->declare_parameter("optical_flow.window_size", 21);
        lk_win_size_ = cv::Size(lk_win_size, lk_win_size);
        lk_max_level_ = this->declare_parameter("optical_flow.max_pyramid_level", 3);
        int max_iterations = this->declare_parameter("optical_flow.max_iterations", 30);
        double epsilon = this->declare_parameter("optical_flow.epsilon", 0.01);
        lk_term_criteria_ = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, max_iterations, epsilon);

        // Initialize TF2 buffer and listener first
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if (!initializeCalibration()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize calibration. Node will not start.");
            return;
        }

        initializeArucoDetector();
        createPublishersAndSubscribers();

        // Visualization configuration (must be after publishers are created, nested under visualization)
        VisualizationConfig vis_config;
        vis_config.trail_marker_interval_sec = this->declare_parameter("visualization.trail_marker_interval_sec", 0.1);
        vis_config.trail_marker_size = this->declare_parameter("visualization.trail_marker_size", 0.08);
        vis_config.debug_text_scale = this->declare_parameter("visualization.debug_text_scale", 0.6);
        vis_config.debug_text_thickness = this->declare_parameter("visualization.debug_text_thickness", 2);
        vis_config.debug_text_line_height = this->declare_parameter("visualization.debug_text_line_height", 30);
        vis_config.show_rejected_candidates = this->declare_parameter("visualization.show_rejected_candidates", true);
        vis_config.show_corner_points = this->declare_parameter("visualization.show_corner_points", true);
        vis_config.show_detector_params = this->declare_parameter("visualization.show_detector_params", true);
        vis_config.show_marker_ids = this->declare_parameter("visualization.show_marker_ids", true);
        
        visualization_helper_ = std::make_shared<VisualizationHelper>(vis_config);

        RCLCPP_INFO(this->get_logger(), "ArUco Pose Estimation Node initialized");
        RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Marker map: %zu markers loaded", marker_map_.getMarkerCount());
        RCLCPP_INFO(this->get_logger(), "World frame: %s, Camera frame: %s, Base link frame: %s",
                    world_frame_.c_str(), camera_frame_.c_str(), base_link_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Using TF2 to look up camera->base_link transform");
        RCLCPP_INFO(this->get_logger(), "Smoothing: %s (pos_alpha=%.2f, orient_alpha=%.2f)",
                    use_smoothing_ ? "enabled" : "disabled", 
                    position_smoothing_alpha_, orientation_smoothing_alpha_);
        RCLCPP_INFO(this->get_logger(), 
            "Optical flow tracking: %s (max_age=%d, redetect_interval=%d)",
            use_optical_flow_ ? "enabled" : "disabled", 
            max_tracking_age_, redetect_interval_);
    }

bool ArucoPoseEstimationNode::initializeCalibration()
{
        // 1. Set safe defaults first (cleaner than repeating code)
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        bool calib_loaded = false;

        // 2. Handle Calibration File
        if (!camera_calibration_file_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Attempting to load calibration: %s", camera_calibration_file_.c_str());

            cv::FileStorage fs(camera_calibration_file_, cv::FileStorage::READ);

            if (fs.isOpened()) {
                fs["camera_matrix"] >> camera_matrix_;
                fs["distortion_coefficients"] >> dist_coeffs_;
                fs.release();

                // 3. Validate Data
                if (camera_matrix_.cols == 3 && camera_matrix_.rows == 3) {
                    RCLCPP_INFO(this->get_logger(), "Camera calibration loaded successfully.");
                    calib_loaded = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Calibration file opened, but 'camera_matrix' data was invalid!");
                    return false;
                }
            } else {
                // 4. Hard Fail if file cannot be opened
                RCLCPP_ERROR(this->get_logger(), "Could not open requested calibration file!");
                return false;
                }
            }

        // 6. Load Marker Map
        if (marker_map_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Marker map file parameter is empty!");
            return false;
        }

        if (!marker_map_.loadFromFile(marker_map_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load marker map from: %s", marker_map_file_.c_str());
            return false;
        }

        return true;
}

void ArucoPoseEstimationNode::initializeArucoDetector()
{
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_->cornerRefinementWinSize = corner_refinement_win_size_;
        detector_params_->cornerRefinementMaxIterations = corner_refinement_max_iterations_;
        detector_params_->cornerRefinementMinAccuracy = corner_refinement_min_accuracy_;

        // Set corner refinement method based on window size
        if (corner_refinement_win_size_ > 0) {
            detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        } else {
            detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
        }
        detector_params_->adaptiveThreshWinSizeMin = adaptive_thresh_win_size_min_;
        detector_params_->adaptiveThreshWinSizeMax = adaptive_thresh_win_size_max_;
        detector_params_->adaptiveThreshWinSizeStep = adaptive_thresh_win_size_step_;
        detector_params_->minMarkerPerimeterRate = min_marker_perimeter_rate_;
        detector_params_->maxMarkerPerimeterRate = max_marker_perimeter_rate_;
    detector_params_->polygonalApproxAccuracyRate = polygonal_approx_accuracy_rate_;
    detector_params_->minMarkerDistanceRate = min_marker_distance_rate_;
}

void ArucoPoseEstimationNode::createPublishersAndSubscribers()
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, 10,
        std::bind(&ArucoPoseEstimationNode::imageCallback, this, std::placeholders::_1));

    camera_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/camera_pose", 10);
    drone_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/aruco/drone_pose", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aruco/visualization_markers", 10);
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/aruco/diagnostics", 10);

    if (publish_debug_image_) {
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco/debug_image", 10);
    }

    // Always create the TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

bool ArucoPoseEstimationNode::transformCameraToDrone(const cv::Vec3d& camera_position, const tf2::Quaternion& camera_quat,
                                cv::Vec3d& drone_position, tf2::Quaternion& drone_quat)
{
    try {
            // This transform defines where the camera is relative to the drone's base.
            geometry_msgs::msg::TransformStamped base_to_camera_transform_msg;
            base_to_camera_transform_msg = tf_buffer_->lookupTransform(
                base_link_frame_, camera_frame_, tf2::TimePointZero);

            // Convert camera pose to tf2::Transform
            tf2::Transform camera_in_world;
            camera_in_world.setOrigin(tf2::Vector3(camera_position[0], camera_position[1], camera_position[2]));
            camera_in_world.setRotation(camera_quat);

            // Convert the TF message to tf2::Transform. This is T_base_camera.
            tf2::Transform base_to_camera_transform;
            tf2::fromMsg(base_to_camera_transform_msg.transform, base_to_camera_transform);

      
            // T_world_base = T_world_camera * T_base_camera^-1
            tf2::Transform base_in_world = camera_in_world * base_to_camera_transform.inverse();

            // Extract position and orientation
            drone_position[0] = base_in_world.getOrigin().x();
            drone_position[1] = base_in_world.getOrigin().y();
            drone_position[2] = base_in_world.getOrigin().z();
            drone_quat = base_in_world.getRotation();

            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not transform %s to %s: %s",
                                 base_link_frame_.c_str(), camera_frame_.c_str(), ex.what());
            return false;
        }
}

// Track corners using Lucas-Kanade optical flow
void ArucoPoseEstimationNode::trackCornersWithLK(const cv::Mat& gray, std::vector<int>& marker_ids, std::vector<std::vector<cv::Point2f>>& marker_corners)
{
    if (prev_gray_.empty() || tracked_corners_.empty()) {
        return;  // Nothing to track yet
    }
    
    marker_ids.clear();
    marker_corners.clear();
    
    std::vector<int> markers_to_remove;
    
    for (auto& [marker_id, prev_corners] : tracked_corners_) {
        std::vector<cv::Point2f> next_corners;
        std::vector<uchar> status;
        std::vector<float> err;
        
        // Track corners from previous frame to current frame
        cv::calcOpticalFlowPyrLK(
            prev_gray_, gray,
            prev_corners, next_corners,
            status, err,
            lk_win_size_, lk_max_level_,
            lk_term_criteria_,
            0);
        
        // Check if all 4 corners were successfully tracked
        int tracked_count = std::count(status.begin(), status.end(), 1);
        
        if (tracked_count == 4 && validateTrackedCorners(next_corners, gray.size())) {
            // Successfully tracked all corners
            marker_ids.push_back(marker_id);
            marker_corners.push_back(next_corners);
            tracked_corners_[marker_id] = next_corners;
            tracking_age_[marker_id]++;
            
            // If tracked too long, mark for redetection
            if (tracking_age_[marker_id] > max_tracking_age_) {
                markers_to_remove.push_back(marker_id);
            }
        } else {
            // Lost tracking for this marker
            markers_to_remove.push_back(marker_id);
        }
    }
    
    // Remove markers that lost tracking or exceeded max age
    for (int marker_id : markers_to_remove) {
        tracked_corners_.erase(marker_id);
        tracking_age_.erase(marker_id);
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "LK Tracking: %zu markers tracked, %zu lost",
                 marker_ids.size(), markers_to_remove.size());
}

bool ArucoPoseEstimationNode::validateTrackedCorners(
    const std::vector<cv::Point2f>& corners,
    const cv::Size& frame_size)
{
    if (corners.size() != 4) return false;
    
    // Check all corners are within frame bounds
    for (const auto& corner : corners) {
        if (corner.x < 0 || corner.x >= frame_size.width ||
            corner.y < 0 || corner.y >= frame_size.height) {
            return false;
        }
    }
    
    // Check corners form a convex quadrilateral (basic sanity check)
    double area = cv::contourArea(corners);
    if (area < 100.0) {  // Minimum marker size in pixels
        return false;
    }
    
    // Check aspect ratio isn't too distorted
    double side1 = cv::norm(corners[0] - corners[1]);
    double side2 = cv::norm(corners[1] - corners[2]);
    double side3 = cv::norm(corners[2] - corners[3]);
    double side4 = cv::norm(corners[3] - corners[0]);
    
    double ratio1 = std::max(side1, side3) / std::min(side1, side3);
    double ratio2 = std::max(side2, side4) / std::min(side2, side4);
    
    if (ratio1 > 3.0 || ratio2 > 3.0) {
        return false;  // Too distorted
    }
    
    return true;
}

void ArucoPoseEstimationNode::updateTrackedMarkers(const std::vector<int>& detected_ids, const std::vector<std::vector<cv::Point2f>>& detected_corners)
{
    for (size_t i = 0; i < detected_ids.size(); ++i) {
        int marker_id = detected_ids[i];
        tracked_corners_[marker_id] = detected_corners[i];
        tracking_age_[marker_id] = 0;  // Reset age for newly detected markers
    }
}


cv::Vec3d ArucoPoseEstimationNode::smoothPosition(const cv::Vec3d& new_position, const cv::Vec3d& prev_position, double alpha)
{
    if (!has_previous_pose_) {
        return new_position;
    }
    
    // Exponential moving average
    cv::Vec3d smoothed;
    smoothed[0] = alpha * new_position[0] + (1.0 - alpha) * prev_position[0];
    smoothed[1] = alpha * new_position[1] + (1.0 - alpha) * prev_position[1];
    smoothed[2] = alpha * new_position[2] + (1.0 - alpha) * prev_position[2];
    
    return smoothed;
}

tf2::Quaternion ArucoPoseEstimationNode::smoothOrientation(const tf2::Quaternion& new_quat, const tf2::Quaternion& prev_quat, double alpha)
{
    if (!has_previous_pose_) {
        return new_quat;
    }
    
    tf2::Quaternion smoothed = prev_quat.slerp(new_quat, alpha);
    smoothed.normalize();
    
    return smoothed;
}

void ArucoPoseEstimationNode::publishPoses(const cv::Vec3d& camera_position, const tf2::Quaternion& camera_quat,
                      const rclcpp::Time& timestamp)
{
    // Apply smoothing to camera pose if enabled
    cv::Vec3d smoothed_camera_position = camera_position;
    tf2::Quaternion smoothed_camera_quat = camera_quat;
    
    if (use_smoothing_ && has_previous_pose_) {
        smoothed_camera_position = smoothPosition(camera_position, prev_camera_position_, position_smoothing_alpha_);
        smoothed_camera_quat = smoothOrientation(camera_quat, prev_camera_quat_, orientation_smoothing_alpha_);
    }
    
    // Store current pose for next iteration
    prev_camera_position_ = smoothed_camera_position;
    prev_camera_quat_ = smoothed_camera_quat;
    has_previous_pose_ = true;

    // Publish smoothed camera pose
    geometry_msgs::msg::PoseStamped camera_pose_msg;
    camera_pose_msg.header.stamp = timestamp;
    camera_pose_msg.header.frame_id = world_frame_;
    camera_pose_msg.pose.position.x = smoothed_camera_position[0];
    camera_pose_msg.pose.position.y = smoothed_camera_position[1];
    camera_pose_msg.pose.position.z = smoothed_camera_position[2];
    camera_pose_msg.pose.orientation.x = smoothed_camera_quat.x();
    camera_pose_msg.pose.orientation.y = smoothed_camera_quat.y();
    camera_pose_msg.pose.orientation.z = smoothed_camera_quat.z();
    camera_pose_msg.pose.orientation.w = smoothed_camera_quat.w();
    camera_pose_pub_->publish(camera_pose_msg);

    cv::Vec3d drone_position;
    tf2::Quaternion drone_quat;
    if (!transformCameraToDrone(smoothed_camera_position, smoothed_camera_quat, drone_position, drone_quat)) {
        return;  // Transform failed, skip publishing drone pose
    }

    geometry_msgs::msg::PoseWithCovarianceStamped drone_pose_msg;
    drone_pose_msg.header.stamp = timestamp;
    drone_pose_msg.header.frame_id = world_frame_;
    drone_pose_msg.pose.pose.position.x = drone_position[0];
    drone_pose_msg.pose.pose.position.y = drone_position[1];
    drone_pose_msg.pose.pose.position.z = drone_position[2];
    drone_pose_msg.pose.pose.orientation.x = drone_quat.x();
    drone_pose_msg.pose.pose.orientation.y = drone_quat.y();
    drone_pose_msg.pose.pose.orientation.z = drone_quat.z();
    drone_pose_msg.pose.pose.orientation.w = drone_quat.w();
    drone_pose_pub_->publish(drone_pose_msg);

    // Publish trail marker periodically
    static rclcpp::Time last_trail_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    double trail_interval = visualization_helper_->getConfig().trail_marker_interval_sec;
    if ((timestamp - last_trail_time).seconds() >= trail_interval) {
        visualization_helper_->publishTrailMarker(drone_position, timestamp, world_frame_, marker_pub_);
        last_trail_time = timestamp;
    }
}

void ArucoPoseEstimationNode::publishTFTransforms(const cv::Vec3d& camera_position, const tf2::Quaternion& camera_quat,
                             const rclcpp::Time& timestamp)
{
    cv::Vec3d drone_position;
    tf2::Quaternion drone_quat;
    if (!transformCameraToDrone(camera_position, camera_quat, drone_position, drone_quat)) {
        return;
    }

    // Only publish world -> base_link transform
    // The base_link -> camera_link transform should come from your URDF or a static transform publisher
    geometry_msgs::msg::TransformStamped drone_transform;
    drone_transform.header.stamp = timestamp;
    drone_transform.header.frame_id = world_frame_;
    drone_transform.child_frame_id = base_link_frame_;
    drone_transform.transform.translation.x = drone_position[0];
    drone_transform.transform.translation.y = drone_position[1];
    drone_transform.transform.translation.z = drone_position[2];
    drone_transform.transform.rotation.x = drone_quat.x();
    drone_transform.transform.rotation.y = drone_quat.y();
    drone_transform.transform.rotation.z = drone_quat.z();
    drone_transform.transform.rotation.w = drone_quat.w();
    tf_broadcaster_->sendTransform(drone_transform);
}

cv::Mat ArucoPoseEstimationNode::convertRosImageToCV(const sensor_msgs::msg::Image::SharedPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        return cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

void ArucoPoseEstimationNode::detectArucoMarkers(const cv::Mat& frame, std::vector<int>& marker_ids,
                            std::vector<std::vector<cv::Point2f>>& marker_corners)
{
    rejected_candidates_.clear();
    cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids,
                             detector_params_, rejected_candidates_,
                             camera_matrix_, dist_coeffs_);
}

bool ArucoPoseEstimationNode::estimateAndValidatePose(const std::vector<int>& marker_ids,
                                  const std::vector<std::vector<cv::Point2f>>& marker_corners,
                                  cv::Vec3d& camera_position, cv::Vec3d& camera_orientation,
                                  std::vector<int>& used_markers)
{
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;

    // Estimate pose using the marker map
    if (!marker_map_.estimateCameraPose(marker_ids, marker_corners, camera_matrix_, dist_coeffs_,
                                        camera_position, camera_orientation, used_markers,
                                        &object_points, &image_points)) {
        return false;
    }

    if (!is_initialized_) {
        if (static_cast<int>(used_markers.size()) >= min_markers_for_init_) {
            is_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Pose initialized! Markers: %zu",
                       used_markers.size());
            return true;
        }
        return false;
    }

    return true;
}

void ArucoPoseEstimationNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    cv::Mat frame = convertRosImageToCV(msg);
    if (frame.empty()) return;

    // Convert to grayscale for optical flow
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    
    bool force_detection = (frames_since_detection_ >= redetect_interval_);
    
    if (use_optical_flow_ && !prev_gray_.empty() && !tracked_corners_.empty() && !force_detection) {
        trackCornersWithLK(gray, marker_ids, marker_corners);
        
        // If we lost too many markers, force a full detection
        if (marker_ids.size() < 2) {
            RCLCPP_DEBUG(this->get_logger(), "Too few tracked markers, forcing full detection");
            marker_ids.clear();
            marker_corners.clear();
            force_detection = true;
        }
    }
    
    // Perform full ArUco detection if needed
    if (marker_ids.empty() || force_detection) {
        detectArucoMarkers(frame, marker_ids, marker_corners);
        updateTrackedMarkers(marker_ids, marker_corners);
        frames_since_detection_ = 0;
        
        RCLCPP_DEBUG(this->get_logger(), "Full detection: %zu markers found", marker_ids.size());
    } else {
        frames_since_detection_++;
        RCLCPP_DEBUG(this->get_logger(), "Using optical flow: %zu markers tracked", marker_ids.size());
    }
    
    // Store current frame for next iteration
    prev_gray_ = gray.clone();

    if (marker_ids.empty()) return;

    cv::Vec3d camera_position, camera_orientation;
    std::vector<int> used_markers;
    if (!estimateAndValidatePose(marker_ids, marker_corners, camera_position, camera_orientation,
                                   used_markers)) {
        return;
    }

    tf2::Quaternion camera_quat = pose_estimation_utils::cvRotationVecToQuaternion(camera_orientation);

    publishPoses(camera_position, camera_quat, msg->header.stamp);
    publishTFTransforms(camera_position, camera_quat, msg->header.stamp);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double processing_time_ms = duration.count() / 1000.0;

    // Get drone pose for diagnostics
    cv::Vec3d drone_position;
    tf2::Quaternion drone_quat;
    if (transformCameraToDrone(camera_position, camera_quat, drone_position, drone_quat)) {
        if (visualization_helper_ && diagnostics_pub_) {
            auto logger_ptr = std::make_shared<rclcpp::Logger>(this->get_logger());
            visualization_helper_->publishDiagnostics(
                msg->header.stamp, marker_ids.size(), used_markers.size(),
                used_markers, true, processing_time_ms, drone_position, drone_quat,
                marker_map_.getMarkerCount(), is_initialized_,
                diagnostics_pub_, logger_ptr);
        }
    }

    // Draw and publish debug image
    if (publish_debug_image_ && visualization_helper_ && debug_image_pub_) {
        visualization_helper_->drawDebugImage(
            frame, marker_corners, marker_ids, used_markers, rejected_candidates_,
            camera_position, camera_quat, is_initialized_, detector_params_,
            [this](const cv::Vec3d& cam_pos, const tf2::Quaternion& cam_quat,
                   cv::Vec3d& drone_pos, tf2::Quaternion& drone_quat) -> bool {
                return this->transformCameraToDrone(cam_pos, cam_quat, drone_pos, drone_quat);
            });
        visualization_helper_->publishDebugImage(frame, msg->header.stamp, debug_image_pub_);
    }
}

} // namespace aruco_pose_estimation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<aruco_pose_estimation::ArucoPoseEstimationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}