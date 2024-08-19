#include "DataAssociation.h"

namespace aprilslam {

DataAssociation::DataAssociation() {
    nh_.getParam("association_threshold", association_threshold_);
    associated_clusters_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("associated_clusters", 1);
}

int DataAssociation::associateCluster(const Eigen::Vector3f& current_center) {
    int associated_id = -1;
    float min_distance = association_threshold_;

    for (const auto& [id, previous_center] : ) {
        // float distance = (current_center - previous_center).norm();
        float distance = std::sqrt(std::pow(current_center.x() - previous_center.x(), 2) +
                     std::pow(current_center.y() - previous_center.y(), 2));

        if (distance < min_distance) {
            min_distance = distance;
            associated_id = id;
        }
    }

    if (associated_id == -1) {
        associated_id = next_id_++;
    }

    previous_cluster_centers_[associated_id] = current_center;
    return associated_id;
}

void DataAssociation::associateClusters(const std::vector<Eigen::Vector3f>& cluster_centers, const std::string& frame_id) {
    std::unordered_map<int, Eigen::Vector3f> current_cluster_centers;
    visualization_msgs::MarkerArray marker_array;

    int j = 0;
    for (const auto& center : cluster_centers) {
        // Associate the global center with an ID
        int cluster_id = associateCluster(center);
        current_cluster_centers[cluster_id] = center;

        // Create a marker for the cluster center
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;  // Global frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "associated_clusters";
        marker.id = cluster_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = center[0];
        marker.pose.position.y = center[1];
        marker.pose.position.z = center[2];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // Add the sphere marker to the array
        marker_array.markers.push_back(marker);

        // Create a marker for the text displaying the ID
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_id;  // Global frame
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "associated_clusters";
        text_marker.id = cluster_id + 1000; // Use a different ID to avoid conflicts with sphere markers
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = center[0];
        text_marker.pose.position.y = center[1];
        text_marker.pose.position.z = center[2] + 0.2;  // Slightly above the cluster center
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.2;  // Text height
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0f;
        text_marker.text = std::to_string(cluster_id); // Display the cluster ID

        // Add the text marker to the array
        marker_array.markers.push_back(text_marker);

        j++;
    }

    // Update the previous cluster centers for the next frame
    previous_cluster_centers_ = current_cluster_centers;

    // Publish the marker array with associated IDs and text markers
    associated_clusters_pub_.publish(marker_array);
}

} // namespace aprilslam
