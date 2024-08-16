#ifndef PcClustering
#define PcClustering

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/passthrough.h>
#include <tf/tf.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include "DataAssociation.h"

namespace aprilslam {
    typedef pcl::PointXYZ PointT;

    class TreeTrunkDetector
    {
    public:
        TreeTrunkDetector(); // Constructor
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input); // Callback function for LIDAR data
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); // Callback function for odometry data

    private:
        ros::NodeHandle nh_;           // ROS node handle
        ros::Subscriber sub_;          // ROS subscriber for point cloud data
        ros::Subscriber odom_sub_;     // ROS subscriber for odometry data
        ros::Publisher cluster_centers_pub_; // ROS publisher for cluster centers
        DataAssociation data_association_; // Data association object to track clusters

        nav_msgs::Odometry current_odom_; // Store the latest odometry message
        int marker_id_;  // ID counter for markers

        Eigen::Vector3f transformToGlobal(const Eigen::Vector3f& local_point); // Transform point to global frame

        // Parameters
        std::string input_cloud_topic_;
        std::string input_odom_topic_;
        std::string output_marker_topic_;
        float azimuth_min_;
        float azimuth_max_;
        float z_min_;
        float z_max_;
        float x_min_;
        float x_max_;
        float y_min_;
        float y_max_;
        int mean_k_;
        float stddev_mul_thresh_;
        float cluster_tolerance_;
        int min_cluster_size_;
        int max_cluster_size_;
    };
}

#endif
