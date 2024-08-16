#include "PcClustering.h"
#include "DataAssociation.h"

namespace aprilslam {

TreeTrunkDetector::TreeTrunkDetector() {
    // Load parameters from the YAML file
    nh_.getParam("input_cloud_topic", input_cloud_topic_);
    nh_.getParam("input_odom_topic", input_odom_topic_);
    nh_.getParam("output_marker_topic", output_marker_topic_);
    nh_.getParam("azimuth_min", azimuth_min_);
    nh_.getParam("azimuth_max", azimuth_max_);
    nh_.getParam("z_min", z_min_);
    nh_.getParam("z_max", z_max_);
    nh_.getParam("x_min", x_min_);
    nh_.getParam("x_max", x_max_);
    nh_.getParam("y_min", y_min_);
    nh_.getParam("y_max", y_max_);
    nh_.getParam("mean_k", mean_k_);
    nh_.getParam("stddev_mul_thresh", stddev_mul_thresh_);
    nh_.getParam("cluster_tolerance", cluster_tolerance_);
    nh_.getParam("min_cluster_size", min_cluster_size_);
    nh_.getParam("max_cluster_size", max_cluster_size_);

    // Set up subscribers and publishers
    sub_ = nh_.subscribe(input_cloud_topic_, 1, &TreeTrunkDetector::cloudCallback, this);
    odom_sub_ = nh_.subscribe(input_odom_topic_, 1, &TreeTrunkDetector::odomCallback, this);
    cluster_centers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(output_marker_topic_, 1);

    marker_id_ = 0;  // Initialize marker ID counter
}

void TreeTrunkDetector::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = *msg;  // Store the latest odometry information
}

Eigen::Vector3f TreeTrunkDetector::transformToGlobal(const Eigen::Vector3f& local_point) {
    tf::Vector3 point(local_point[0], local_point[1], local_point[2]);

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
    return Eigen::Vector3f(global_point.x(), global_point.y(), global_point.z());
}

void TreeTrunkDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *cloud);
    ROS_INFO("Received point cloud with %zu points.", cloud->points.size());

    pcl::PointCloud<PointT>::Ptr cloud_filtered_azimuth(new pcl::PointCloud<PointT>);
    for (const auto& point : cloud->points) {
        float azimuth = std::atan2(point.y, point.x);
        if (azimuth >= azimuth_min_ && azimuth <= azimuth_max_) {
            cloud_filtered_azimuth->points.push_back(point);
        }
    }
    ROS_INFO("Filtered point cloud based on azimuth, remaining %zu points.", cloud_filtered_azimuth->points.size());

    pcl::PointCloud<PointT>::Ptr cloud_filtered_z(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_filtered_azimuth);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*cloud_filtered_z);
    ROS_INFO("Filtered point cloud based on height, remaining %zu points.", cloud_filtered_z->points.size());

    pcl::PointCloud<PointT>::Ptr cloud_filtered_x(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass_x;
    pass_x.setInputCloud(cloud_filtered_z);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min_, x_max_);
    pass_x.filter(*cloud_filtered_x);
    ROS_INFO("Filtered point cloud based on X range, remaining %zu points.", cloud_filtered_x->points.size());

    pcl::PointCloud<PointT>::Ptr cloud_filtered_xy(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass_y;
    pass_y.setInputCloud(cloud_filtered_x);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min_, y_max_);
    pass_y.filter(*cloud_filtered_xy);
    ROS_INFO("Filtered point cloud based on Y range, remaining %zu points.", cloud_filtered_xy->points.size());

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_filtered_xy);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(stddev_mul_thresh_);
    sor.filter(*cloud_filtered);
    ROS_INFO("After noise removal, %zu points remain.", cloud_filtered->points.size());

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    ROS_INFO("Detected %zu clusters.", cluster_indices.size());

    std::vector<Eigen::Vector3f> cluster_centers;
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        Eigen::Vector4f centroid;

        for (const auto& idx : indices.indices)
            cloud_cluster->points.push_back(cloud_filtered->points[idx]);

        pcl::compute3DCentroid(*cloud_cluster, centroid);
        Eigen::Vector3f local_center(centroid[0], centroid[1], centroid[2]);

        Eigen::Vector3f global_center = transformToGlobal(local_center);
        cluster_centers.push_back(global_center);
    }

    data_association_.associateClusters(cluster_centers, "map");
}

}  // namespace aprilslam

int main(int argc, char** argv) {
    ros::init(argc, argv, "tree_trunk_detector");

    aprilslam::TreeTrunkDetector detector;

    ros::spin();

    return 0;
}
