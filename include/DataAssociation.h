#ifndef DATA_ASSOCIATION_H
#define DATA_ASSOCIATION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>
#include <Eigen/Dense>

namespace aprilslam {

class DataAssociation {
public:
    DataAssociation(); // Constructor
    void associateClusters(const std::vector<Eigen::Vector3f>& cluster_centers, const std::string& frame_id);

private:
    std::unordered_map<int, Eigen::Vector3f> previous_cluster_centers_; // Store previous cluster centers and their IDs
    int next_id_ = 0; // ID counter for new clusters
    float association_threshold_ = 2.0f; // Distance threshold for association (in meters)
    
    ros::NodeHandle nh_;
    ros::Publisher associated_clusters_pub_; // Publisher for associated cluster IDs and centers

    int associateCluster(const Eigen::Vector3f& current_center); // Method to associate clusters with unique IDs
};

} // namespace aprilslam

#endif // DATA_ASSOCIATION_H
