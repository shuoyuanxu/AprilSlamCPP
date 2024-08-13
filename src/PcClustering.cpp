#include "PcClustering.h"
#include <pcl/filters/passthrough.h>

namespace aprilslam {
    TreeTrunkDetector::TreeTrunkDetector() {
        sub_ = nh_.subscribe("/ouster/points", 1, &TreeTrunkDetector::cloudCallback, this);
        odom_sub_ = nh_.subscribe("/odometry/filtered", 1, &TreeTrunkDetector::odomCallback, this);
        cluster_centers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cluster_centers", 1);
        marker_id_ = 0;  // Initialize marker ID counter
    }

    void TreeTrunkDetector::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;  // Store the latest odometry information
    }

    void TreeTrunkDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*input, *cloud);
        ROS_INFO("Received point cloud with %zu points.", cloud->points.size());

        pcl::PointCloud<PointT>::Ptr cloud_filtered_azimuth(new pcl::PointCloud<PointT>);
        for (const auto& point : cloud->points) {
            float azimuth = std::atan2(point.y, point.x);
            if (azimuth >= -3*M_PI / 4 && azimuth <= 3*M_PI / 4) {  // Filter for -135° to 135°
                cloud_filtered_azimuth->points.push_back(point);
            }
        }
        ROS_INFO("Filtered point cloud based on azimuth, remaining %zu points.", cloud_filtered_azimuth->points.size());

        pcl::PointCloud<PointT>::Ptr cloud_filtered_z(new pcl::PointCloud<PointT>);
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud_filtered_azimuth);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.8, -0.5); 
        pass.filter(*cloud_filtered_z);
        ROS_INFO("Filtered point cloud based on height, remaining %zu points.", cloud_filtered_z->points.size());

        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud_filtered_z);
        sor.setMeanK(50);  
        sor.setStddevMulThresh(1.0); 
        sor.filter(*cloud_filtered);
        ROS_INFO("After noise removal, %zu points remain.", cloud_filtered->points.size());

        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.3); 
        ec.setMinClusterSize(5);  
        ec.setMaxClusterSize(10000);  
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        ROS_INFO("Detected %zu clusters.", cluster_indices.size());

        visualization_msgs::MarkerArray marker_array;

        // First, delete old markers
        for (int i = 0; i < marker_id_; i++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";  // Global frame
            marker.header.stamp = ros::Time::now();
            marker.ns = "tree_trunk_centers";
            marker.id = i;
            marker.action = visualization_msgs::Marker::DELETE; // Delete marker
            marker_array.markers.push_back(marker);
        }

        int j = 0;
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            Eigen::Vector4f centroid;
            for (const auto& idx : indices.indices)
                cloud_cluster->points.push_back(cloud_filtered->points[idx]);

            pcl::compute3DCentroid(*cloud_cluster, centroid);

            // Transform centroid to global frame using odometry information
            tf::Vector3 point(centroid[0], centroid[1], centroid[2]);
            tf::Quaternion orientation(
                current_odom_.pose.pose.orientation.x,
                current_odom_.pose.pose.orientation.y,
                current_odom_.pose.pose.orientation.z,
                current_odom_.pose.pose.orientation.w
            );

            tf::Transform transform(orientation, tf::Vector3(
                current_odom_.pose.pose.position.x,
                current_odom_.pose.pose.position.y,
                current_odom_.pose.pose.position.z
            ));

            tf::Vector3 global_point = transform * point;

            // Create a marker for the centroid in the global frame
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";  // Global frame
            marker.header.stamp = ros::Time::now();
            marker.ns = "tree_trunk_centers";
            marker.id = j;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;  // Add marker
            marker.pose.position.x = global_point.x();
            marker.pose.position.y = global_point.y();
            marker.pose.position.z = global_point.z();
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            marker_array.markers.push_back(marker);
            j++;
        }

        marker_id_ = j;  // Update marker count

        if (!cluster_indices.empty()) {
            ROS_INFO("Publishing %zu clusters and their centroids.", cluster_indices.size());
        } else {
            ROS_WARN("No clusters found in the filtered point cloud.");
        }

        cluster_centers_pub_.publish(marker_array);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tree_trunk_detector");

    aprilslam::TreeTrunkDetector detector;

    ros::spin();

    return 0;
}
