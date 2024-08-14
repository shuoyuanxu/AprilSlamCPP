#ifndef DataAssociation
#define DataAssociation

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

namespace aprilslam {
    typedef pcl::PointXYZ PointT;

    class DataAssociation
    {
    public:
        DataAssociation(); // Constructor
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input); // Callback function for LIDAR data
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); // Callback function for odometry data

    private:
        ros::NodeHandle nh_;           // ROS node handle
        ros::Subscriber sub_;          // ROS subscriber for point cloud data
        ros::Subscriber odom_sub_;     // ROS subscriber for odometry data
        ros::Publisher cluster_centers_pub_; // ROS publisher for cluster centers

        nav_msgs::Odometry current_odom_; // Latest odometry data
        int marker_id_;  // Variable to track the marker IDs
    };
}

#endif
