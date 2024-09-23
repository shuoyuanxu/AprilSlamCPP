// publishing_utils.cpp

#include "publishing_utils.h"

namespace aprilslam {

void publishLandmarks(ros::Publisher& landmark_pub, const std::map<int, gtsam::Point2>& landmarks, const std::string& frame_id) {
    visualization_msgs::MarkerArray markers;
    int id = 0;
    for (const auto& landmark : landmarks) {
        // Sphere marker for each landmark
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "landmarks";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = landmark.second.x();
        marker.pose.position.y = landmark.second.y();
        marker.pose.position.z = 0;  // Assuming the landmarks are on the ground plane
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);

        // Marker IDs
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "landmark_ids";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = landmark.second.x();
        text_marker.pose.position.y = landmark.second.y();
        text_marker.pose.position.z = 0.5;
        text_marker.scale.z = 0.2;
        text_marker.text = std::to_string(landmark.first);
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;

        markers.markers.push_back(text_marker);
    }

    landmark_pub.publish(markers);
}

void publishPath(ros::Publisher& path_pub, const gtsam::Values& result, int max_index, const std::string& frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();
    path.poses.clear();  // Clear any existing poses

    for (int i = 1; i <= max_index; i++) {
        gtsam::Symbol sym('X', i);
        if (result.exists(sym)) {
            gtsam::Pose2 pose = result.at<gtsam::Pose2>(sym);

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = frame_id;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose.position.x = pose.x();
            pose_msg.pose.position.y = pose.y();
            pose_msg.pose.position.z = 0;  // Assuming 2D

            tf2::Quaternion quat;
            quat.setRPY(0, 0, pose.theta());
            pose_msg.pose.orientation = tf2::toMsg(quat);

            path.poses.push_back(pose_msg);
        }
    }

    path_pub.publish(path);
}

void saveLandmarksToCSV(const std::map<int, gtsam::Point2>& landmarks, const std::string& filename) {
    std::ofstream file;
    file.open(filename, std::ios::out); // Open file in write mode

    if (!file) {
        std::cerr << "Failed to open the file!" << std::endl;
        return;
    }
    // Write the header line
    file << "id,x,y\n";
    
    for (const auto& landmark : landmarks) {
        int id = landmark.first;
        gtsam::Point2 point = landmark.second;
        file << id << "," << point.x() << "," << point.y() << "\n";
    }

    file.close();
}

std::map<int, gtsam::Point2> loadLandmarksFromCSV(const std::string& filename) {
    std::map<int, gtsam::Point2> landmarks;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open the file!" << std::endl;
        return landmarks;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string id_str, x_str, y_str;
        if (std::getline(ss, id_str, ',') && std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            int id = std::stoi(id_str);
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            landmarks[id] = gtsam::Point2(x, y);
        }
    }

    file.close();
    return landmarks;
}

// funtion for computing tag locations from coordinate transformation
void processDetections(const apriltag_ros::AprilTagDetectionArray::ConstPtr& cam_msg, 
                       const Eigen::Vector3d& xyTrans_cam_baselink,
                       std::vector<int>& Ids, 
                       std::vector<Eigen::Vector2d>& tagPoss) {
    if (cam_msg) {
        for (const auto& detection : cam_msg->detections) {
            Ids.push_back(detection.id[0]);
            double rotTheta = xyTrans_cam_baselink(2);
            Eigen::Matrix2d R;
            R << cos(rotTheta), -sin(rotTheta),
                 sin(rotTheta),  cos(rotTheta);
            Eigen::Vector2d rotP = R * Eigen::Vector2d(detection.pose.pose.pose.position.z, -detection.pose.pose.pose.position.x);
            tagPoss.push_back(rotP + xyTrans_cam_baselink.head<2>());
        }
    }
}

// funtion for processing tag detection topics into IDs and TagPoss
std::pair<std::vector<int>, std::vector<Eigen::Vector2d>> getCamDetections(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& mCam_msg,
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& rCam_msg,
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& lCam_msg,
    const Eigen::Vector3d& xyTrans_mcam_baselink,
    const Eigen::Vector3d& xyTrans_rcam_baselink,
    const Eigen::Vector3d& xyTrans_lcam_baselink) {

    std::vector<int> Ids;
    std::vector<Eigen::Vector2d> tagPoss;

    // Process detections from each camera
    processDetections(mCam_msg, xyTrans_mcam_baselink, Ids, tagPoss);
    processDetections(rCam_msg, xyTrans_rcam_baselink, Ids, tagPoss);
    processDetections(lCam_msg, xyTrans_lcam_baselink, Ids, tagPoss);

    return std::make_pair(Ids, tagPoss);
}
} // namespace aprilslamcpp


