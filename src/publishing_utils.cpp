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

} // namespace aprilslamcpp
