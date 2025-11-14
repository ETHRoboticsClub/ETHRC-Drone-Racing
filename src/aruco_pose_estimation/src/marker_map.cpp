#include "aruco_pose_estimation/marker_map.hpp"
#include <iostream>
#include <cmath>

namespace marker_map {

// Constructor
MarkerMap::MarkerMap()
    : dictionary_("DICT_5X5_100"), default_marker_size_(0.2f),
      is_loaded_(false), reference_frame_("world") {
}

// Destructor
MarkerMap::~MarkerMap() {
}

// Load marker map from YAML file
bool MarkerMap::loadFromFile(const std::string& filepath) {

    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "ERROR: Could not open marker map file: " << filepath << std::endl;
        return false;
    }

    // Read global parameters
    fs["marker_dict"] >> dictionary_;
    fs["reference_frame"] >> reference_frame_;
    fs["marker_size"] >> default_marker_size_;

    // Read markers array
    cv::FileNode markers_node = fs["markers"];
    if (markers_node.type() != cv::FileNode::SEQ) {
        std::cerr << "ERROR: 'markers' is not a sequence in YAML file" << std::endl;
        fs.release();
        return false;
    }
    //clear existing markers
    markers_.clear();

    for (cv::FileNodeIterator it = markers_node.begin(); it != markers_node.end(); ++it) {
        MarkerInfo info;

        (*it)["id"] >> info.id;
        (*it)["size"] >> info.size;

        cv::FileNode pose_node = (*it)["pose"];
        std::vector<double> pos, orient;
        pose_node["position"] >> pos;
        pose_node["orientation"] >> orient;

        if (pos.size() != 3 || orient.size() != 3) {
            std::cerr << "ERROR: Invalid position or orientation for marker " << info.id << std::endl;
            continue;
        }

        info.position = cv::Vec3d(pos[0], pos[1], pos[2]);
        info.orientation = cv::Vec3d(orient[0], orient[1], orient[2]);

        markers_[info.id] = info;
    }

    fs.release();

    is_loaded_ = true;
    std::cout << "Loaded marker map with " << markers_.size() << " markers" << std::endl;
    std::cout << "Dictionary: " << dictionary_ << std::endl;
    std::cout << "Reference frame: " << reference_frame_ << std::endl;

    return true;
}

// Get marker info by ID
bool MarkerMap::getMarkerInfo(int marker_id, MarkerInfo& info) const {
    auto it = markers_.find(marker_id);
    if (it == markers_.end()) {
        return false;
    }
    info = it->second;
    return true;
}

// Get all marker IDs
std::vector<int> MarkerMap::getAllMarkerIds() const {
    std::vector<int> ids;
    for (const auto& pair : markers_) {
        ids.push_back(pair.first);
    }
    return ids;
}

// Convert Euler angles (roll, pitch, yaw) to rotation matrix
cv::Mat MarkerInfo::getRotationMatrix() const {
    double roll = orientation[0];
    double pitch = orientation[1];
    double yaw = orientation[2];

    // Rotation matrices for each axis
    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll)
    );

    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
        cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch)
    );

    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
        cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1
    );

    // Combined rotation: R = Rz * Ry * Rx
    return R_z * R_y * R_x;
}


// Get marker corners in world frame
std::vector<cv::Point3f> MarkerInfo::getWorldCorners() const {
    // Marker corners in marker's local frame (centered at marker origin)
    float half_size = size / 2.0f;
    std::vector<cv::Point3f> local_corners = {
        cv::Point3f(-half_size,  half_size, 0),  // Top-left
        cv::Point3f( half_size,  half_size, 0),  // Top-right
        cv::Point3f( half_size, -half_size, 0),  // Bottom-right
        cv::Point3f(-half_size, -half_size, 0)   // Bottom-left
    };

    // Get rotation matrix
    cv::Mat R = getRotationMatrix();

    // Transform corners to world frame
    std::vector<cv::Point3f> world_corners;
    for (const auto& corner : local_corners) {
        cv::Mat corner_local = (cv::Mat_<double>(3,1) << corner.x, corner.y, corner.z);
        cv::Mat corner_world = R * corner_local;

        world_corners.push_back(cv::Point3f(
            corner_world.at<double>(0) + position[0],
            corner_world.at<double>(1) + position[1],
            corner_world.at<double>(2) + position[2]
        ));
    }

    return world_corners;
}

// Estimate camera pose using multiple markers (multi-marker fusion)
bool MarkerMap::estimateCameraPose(
    const std::vector<int>& detected_marker_ids,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    cv::Vec3d& camera_position,
    cv::Vec3d& camera_orientation,
    std::vector<int>& used_markers
) const {

    if (detected_marker_ids.size() != detected_corners.size()) {
        std::cerr << "ERROR: Mismatch between marker IDs and corners" << std::endl;
        return false;
    }

    // Collect all 3D-2D correspondences from all detected markers
    std::vector<cv::Point3f> all_object_points;
    std::vector<cv::Point2f> all_image_points;
    used_markers.clear();

    for (size_t i = 0; i < detected_marker_ids.size(); i++) {
        int marker_id = detected_marker_ids[i];
        MarkerInfo info;

        if (!getMarkerInfo(marker_id, info)) {
            std::cerr << "WARNING: Detected marker " << marker_id << " not in map, skipping" << std::endl;
            continue;
        }

        // Get world corners for this marker
        std::vector<cv::Point3f> world_corners = info.getWorldCorners();

        // Add all 4 corners to the correspondence list
        all_object_points.insert(all_object_points.end(), world_corners.begin(), world_corners.end());
        all_image_points.insert(all_image_points.end(), detected_corners[i].begin(), detected_corners[i].end());

        used_markers.push_back(marker_id);
    }

    if (all_object_points.size() < 4) {
        std::cerr << "ERROR: Not enough correspondences for pose estimation" << std::endl;
        return false;
    }

    // Solve PnP with all markers at once (multi-marker fusion)
    cv::Vec3d rvec, tvec;
    bool success = cv::solvePnP(
        all_object_points,
        all_image_points,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_ITERATIVE
    );

    if (!success) {
        std::cerr << "ERROR: solvePnP failed" << std::endl;
        return false;
    }

    // solvePnP returns world-to-camera transform (R_wc, t_wc)
    // We need camera-to-world transform for camera position in world frame

    // Convert rotation vector to matrix
    cv::Mat R_wc;
    cv::Rodrigues(rvec, R_wc);

    // Invert the transformation to get camera-to-world
    // Camera position in world = -R_wc^T * t_wc
    cv::Mat t_wc = (cv::Mat_<double>(3,1) << tvec[0], tvec[1], tvec[2]);
    cv::Mat camera_pos_world = -R_wc.t() * t_wc;

    camera_position = cv::Vec3d(
        camera_pos_world.at<double>(0),
        camera_pos_world.at<double>(1),
        camera_pos_world.at<double>(2)
    );

    // Camera orientation in world frame (R_cw = R_wc^T)
    cv::Mat R_cw = R_wc.t();
    cv::Rodrigues(R_cw, camera_orientation);

    return true;
}

// Helper: Convert rotation vector to Euler angles
cv::Vec3d rotationVectorToEulerAngles(const cv::Vec3d& rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    return rotationMatrixToEulerAngles(R);
}

// Helper: Convert rotation matrix to Euler angles
cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat& R) {
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +
                     R.at<double>(1,0) * R.at<double>(1,0));

    bool singular = sy < 1e-6;

    double roll, pitch, yaw;
    if (!singular) {
        roll = atan2(R.at<double>(2,1), R.at<double>(2,2));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        roll = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = 0;
    }

    return cv::Vec3d(roll, pitch, yaw);
}
} // namespace marker_map
