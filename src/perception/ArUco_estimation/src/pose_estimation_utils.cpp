#include "aruco_pose_estimation/pose_estimation_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <limits>

namespace pose_estimation_utils {

MarkerMap::MarkerMap()
    : dictionary_("DICT_5X5_100"), reference_frame_("world"),
      default_marker_size_(0.2f), is_loaded_(false) {}

MarkerMap::~MarkerMap() {}

bool MarkerMap::loadFromFile(const std::string& filepath)
{
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_ERROR(rclcpp::get_logger("MarkerMap"), "Could not open marker map file: %s", filepath.c_str());
        return false;
    }

    fs["marker_dict"] >> dictionary_;
    fs["reference_frame"] >> reference_frame_;
    fs["marker_size"] >> default_marker_size_;

    cv::FileNode markers_node = fs["markers"];
    if (markers_node.type() != cv::FileNode::SEQ) {
        RCLCPP_ERROR(rclcpp::get_logger("MarkerMap"), "'markers' is not a sequence in YAML file");
        fs.release();
        return false;
    }

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
            RCLCPP_ERROR(rclcpp::get_logger("MarkerMap"), "Invalid position or orientation for marker %d", info.id);
            continue;
        }

        info.position = cv::Vec3d(pos[0], pos[1], pos[2]);
        info.orientation = cv::Vec3d(orient[0], orient[1], orient[2]);
        markers_[info.id] = info;
    }

    fs.release();
    is_loaded_ = true;

    RCLCPP_INFO(rclcpp::get_logger("MarkerMap"), "Loaded marker map with %zu markers", markers_.size());
    RCLCPP_INFO(rclcpp::get_logger("MarkerMap"), "Dictionary: %s, Reference frame: %s", dictionary_.c_str(), reference_frame_.c_str());

    return true;
}

bool MarkerMap::getMarkerInfo(int marker_id, MarkerInfo& info) const
{
    auto it = markers_.find(marker_id);
    if (it == markers_.end()) return false;
    info = it->second;
    return true;
}

std::vector<int> MarkerMap::getAllMarkerIds() const
{
    std::vector<int> ids;
    ids.reserve(markers_.size());
    for (const auto& pair : markers_) {
        ids.push_back(pair.first);
    }
    return ids;
}

cv::Mat MarkerInfo::getRotationMatrix() const
{
    const double roll = orientation[0];
    const double pitch = orientation[1];
    const double yaw = orientation[2];

    const double cr = cos(roll), sr = sin(roll);
    const double cp = cos(pitch), sp = sin(pitch);
    const double cy = cos(yaw), sy = sin(yaw);

    cv::Mat R_x = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, cr, -sr, 0, sr, cr);
    cv::Mat R_y = (cv::Mat_<double>(3,3) << cp, 0, sp, 0, 1, 0, -sp, 0, cp);
    cv::Mat R_z = (cv::Mat_<double>(3,3) << cy, -sy, 0, sy, cy, 0, 0, 0, 1);

    return R_z * R_y * R_x;
}

std::vector<cv::Point3f> MarkerInfo::getWorldCorners() const
{
    const float half_size = size / 2.0f;
    const std::vector<cv::Point3f> local_corners = {
        cv::Point3f(-half_size,  half_size, 0),
        cv::Point3f( half_size,  half_size, 0),
        cv::Point3f( half_size, -half_size, 0),
        cv::Point3f(-half_size, -half_size, 0)
    };

    const cv::Mat R = getRotationMatrix();
    std::vector<cv::Point3f> world_corners;
    world_corners.reserve(4);

    for (const auto& corner : local_corners) {
        cv::Mat corner_local = (cv::Mat_<double>(3,1) << corner.x, corner.y, corner.z);
        cv::Mat corner_world = R * corner_local;
        world_corners.emplace_back(
            corner_world.at<double>(0) + position[0],
            corner_world.at<double>(1) + position[1],
            corner_world.at<double>(2) + position[2]
        );
    }

    return world_corners;
}

bool MarkerMap::estimateCameraPose(
    const std::vector<int>& detected_marker_ids,
    const std::vector<std::vector<cv::Point2f>>& detected_corners,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    cv::Vec3d& camera_position,
    cv::Vec3d& camera_orientation,
    std::vector<int>& used_markers,
    std::vector<cv::Point3f>* all_object_points,
    std::vector<cv::Point2f>* all_image_points
) const
{
    if (detected_marker_ids.size() != detected_corners.size()) {
        return false;
    }

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    used_markers.clear();

    // Collect 3D-2D correspondences
    for (size_t i = 0; i < detected_marker_ids.size(); i++) {
        MarkerInfo info;
        if (!getMarkerInfo(detected_marker_ids[i], info)) {
            continue;
        }
        std::vector<cv::Point3f> world_corners = info.getWorldCorners();
        object_points.insert(object_points.end(), world_corners.begin(), world_corners.end());
        image_points.insert(image_points.end(), detected_corners[i].begin(), detected_corners[i].end());
        used_markers.push_back(detected_marker_ids[i]);
    }

    if (object_points.size() < 4) {
        return false;
    }

    // Solve PnP
    cv::Vec3d rvec, tvec;
    if (!cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec)) {
        return false;
    }

    // Convert to camera position in world frame
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat t = (cv::Mat_<double>(3,1) << tvec[0], tvec[1], tvec[2]);
    cv::Mat camera_pos = -R.t() * t;

    camera_position = cv::Vec3d(camera_pos.at<double>(0), camera_pos.at<double>(1), camera_pos.at<double>(2));
    cv::Rodrigues(R.t(), camera_orientation);

    if (all_object_points) *all_object_points = object_points;
    if (all_image_points) *all_image_points = image_points;

    return true;
}

tf2::Quaternion cvRotationVecToQuaternion(const cv::Vec3d& rotation_vector)
{
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);

    tf2::Matrix3x3 tf_rotation(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
    );

    tf2::Quaternion quat;
    tf_rotation.getRotation(quat);
    return quat;
}

}  // namespace pose_estimation_utils
