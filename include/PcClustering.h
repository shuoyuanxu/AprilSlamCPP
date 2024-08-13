#ifndef PcClustering
#define PcClustering

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

namespace aprilslam {
    // Define a point type alias for convenience
    typedef pcl::PointXYZ PointT;

    class TreeTrunkDetector
    {
    public:
        TreeTrunkDetector(); // Constructor
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input); // Callback function for LIDAR data

    private:
        ros::NodeHandle nh_;           // ROS node handle
        ros::Subscriber sub_;          // ROS subscriber for point cloud data
    };
}

#endif


