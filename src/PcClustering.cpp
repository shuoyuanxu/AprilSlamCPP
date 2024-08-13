#include "PcClustering.h"

namespace aprilslam {
    TreeTrunkDetector::TreeTrunkDetector() {
        // Subscribe to the LIDAR point cloud topic (replace "/lidar_topic" with your topic)
        sub_ = nh_.subscribe("/lidar_topic", 1, &TreeTrunkDetector::cloudCallback, this);
    }

    void TreeTrunkDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
        // Convert the ROS message to a PCL point cloud
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*input, *cloud);

        // Apply Statistical Outlier Removal to filter out noise
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);  // Number of neighbors to analyze for each point
        sor.setStddevMulThresh(1.0);  // Standard deviation multiplier threshold
        sor.filter(*cloud_filtered);

        // Perform Euclidean Cluster Extraction to find tree trunks
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.5); // Tolerance in meters
        ec.setMinClusterSize(50);  // Minimum points per cluster
        ec.setMaxClusterSize(10000);  // Maximum points per cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        // Visualization of clusters (tree trunks)
        int j = 0;
        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            for (const auto& idx : indices.indices)
                cloud_cluster->points.push_back(cloud_filtered->points[idx]);

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "Tree trunk " << j++ << " has " << cloud_cluster->points.size() << " points." << std::endl;

            // Optional: Save or visualize the clusters
            pcl::visualization::CloudViewer viewer("Cluster Viewer");
            viewer.showCloud(cloud_cluster);
            while (!viewer.wasStopped()) {}
        }
    }

    int main(int argc, char** argv) {
        ros::init(argc, argv, "tree_trunk_detector");

        TreeTrunkDetector detector;

        ros::spin();

        return 0;
    }
}