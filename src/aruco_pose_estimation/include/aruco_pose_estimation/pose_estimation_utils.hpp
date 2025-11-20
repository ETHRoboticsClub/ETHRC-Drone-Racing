#ifndef POSE_ESTIMATION_UTILS_HPP
#define POSE_ESTIMATION_UTILS_HPP

#include <opencv2/core.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <map>
#include <vector>
#include <string>

namespace pose_estimation_utils {

// Structure to hold marker information
struct MarkerInfo {
    int id;
    float size;  
    cv::Vec3d position;  //(x, y, z)
    cv::Vec3d orientation;  //(roll, pitch, yaw)

    //4 corner points in world frame
    std::vector<cv::Point3f> getWorldCorners() const;

    //(roll, pitch, yaw) to rotation matrix
    cv::Mat getRotationMatrix() const;
};

// Class to manage ArUco marker map
class MarkerMap {
public:
    MarkerMap();
    ~MarkerMap();

    // Load marker map from YAML file
    bool loadFromFile(const std::string& filepath);

    // Get marker info by ID
    bool getMarkerInfo(int marker_id, MarkerInfo& info) const;

    // Get all marker IDs
    std::vector<int> getAllMarkerIds() const;

    // Get marker dictionary name
    std::string getDictionary() const { return dictionary_; }

    // Get reference frame
    std::string getReferenceFrame() const { return reference_frame_; }

    // Get default marker size
    float getDefaultMarkerSize() const { return default_marker_size_; }

    // Check if map is loaded
    bool isLoaded() const { return is_loaded_; }

    // Get number of markers in map
    size_t getMarkerCount() const { return markers_.size(); }

    // Estimate camera pose using multiple markers
    // Returns true if pose estimation successful
    bool estimateCameraPose(
        const std::vector<int>& detected_marker_ids,
        const std::vector<std::vector<cv::Point2f>>& detected_corners,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        cv::Vec3d& camera_position,
        cv::Vec3d& camera_orientation,
        std::vector<int>& used_markers,
        std::vector<cv::Point3f>* all_object_points = nullptr,
        std::vector<cv::Point2f>* all_image_points = nullptr
    ) const;

private:
    std::map<int, MarkerInfo> markers_;
    std::string dictionary_;
    std::string reference_frame_;
    float default_marker_size_;
    bool is_loaded_;
};

// ============================================================================
// Conversion Utilities
// ============================================================================

// Convert OpenCV rotation vector (Rodrigues) to TF2 quaternion
tf2::Quaternion cvRotationVecToQuaternion(const cv::Vec3d& rotation_vector);

} // namespace pose_estimation_utils

#endif // POSE_ESTIMATION_UTILS_HPP
