#include "PcClustering.h"
#include "DataAssociation.h"

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

Eigen::Vector3f TreeTrunkDetector::transformToGlobal(const Eigen::Vector3f& local_point) {
    // Convert Eigen::Vector3f to tf::Vector3 for transformation
    tf::Vector3 point(local_point[0], local_point[1], local_point[2]);

    // Extract odometry orientation and position
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

    // Apply the transformation
    tf::Vector3 global_point = transform * point;

    // Convert back to Eigen::Vector3f
    return Eigen::Vector3f(global_point.x(), global_point.y(), global_point.z());
}

void TreeTrunkDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *cloud);
    ROS_INFO("Received point cloud with %zu points.", cloud->points.size());

    // Filter based on azimuth
    pcl::PointCloud<PointT>::Ptr cloud_filtered_azimuth(new pcl::PointCloud<PointT>);
    for (const auto& point : cloud->points) {
        float azimuth = std::atan2(point.y, point.x);
        if (azimuth >= -3 * M_PI / 4 && azimuth <= 3 * M_PI / 4) {  // Filter for -135° to 135°
            cloud_filtered_azimuth->points.push_back(point);
        }
    }
    ROS_INFO("Filtered point cloud based on azimuth, remaining %zu points.", cloud_filtered_azimuth->points.size());

    // Filter based on height
    pcl::PointCloud<PointT>::Ptr cloud_filtered_z(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_filtered_azimuth);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.8, -0.5); 
    pass.filter(*cloud_filtered_z);
    ROS_INFO("Filtered point cloud based on height, remaining %zu points.", cloud_filtered_z->points.size());

    // Filter based on X range
    pcl::PointCloud<PointT>::Ptr cloud_filtered_x(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass_x;
    pass_x.setInputCloud(cloud_filtered_z);  // Assuming cloud_filtered_azimuth is your input cloud
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(5.0, 5.0);  // Set limits for X range
    pass_x.filter(*cloud_filtered_x);
    ROS_INFO("Filtered point cloud based on X range, remaining %zu points.", cloud_filtered_x->points.size());

    // Filter based on Y range
    pcl::PointCloud<PointT>::Ptr cloud_filtered_xy(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass_y;
    pass_y.setInputCloud(cloud_filtered_x);  // Input is the output from the previous X filter
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-5.0, 5.0);  // Set limits for Y range
    pass_y.filter(*cloud_filtered_xy);
    ROS_INFO("Filtered point cloud based on Y range, remaining %zu points.", cloud_filtered_xy->points.size());

    // Remove noise
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_filtered_xy);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    ROS_INFO("After noise removal, %zu points remain.", cloud_filtered->points.size());

    // Cluster extraction
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

    // Compute centroids, transform to global frame, and store them
    std::vector<Eigen::Vector3f> cluster_centers;
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        Eigen::Vector4f centroid;

        for (const auto& idx : indices.indices)
            cloud_cluster->points.push_back(cloud_filtered->points[idx]);

        pcl::compute3DCentroid(*cloud_cluster, centroid);
        Eigen::Vector3f local_center(centroid[0], centroid[1], centroid[2]);

        // Transform to global frame using the stored odometry data
        Eigen::Vector3f global_center = transformToGlobal(local_center);
        cluster_centers.push_back(global_center);
    }

    // Associate clusters using the transformed global centers
    data_association_.associateClusters(cluster_centers, "map");

    // The DataAssociation class handles the publication of associated markers
}

}  // namespace aprilslam

int main(int argc, char** argv) {
    ros::init(argc, argv, "tree_trunk_detector");

    aprilslam::TreeTrunkDetector detector;

    ros::spin();

    return 0;
}
