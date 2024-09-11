#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <gtsam/geometry/Pose2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For TF2 quaternion conversion functions
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Header.h>

// Class to handle landmark plotting based on odometry
class LandmarkPlotter {
public:
    LandmarkPlotter(ros::NodeHandle& nh) {
        // Subscribe to odometry and camera detections
        odom_sub_ = nh.subscribe("/robot/dlo/odom_node/odom", 10, &LandmarkPlotter::odomCallback, this);
        mCam_sub_ = nh.subscribe("/m/tag_detections", 10, &LandmarkPlotter::mCamCallback, this);
        rCam_sub_ = nh.subscribe("/r/tag_detections", 10, &LandmarkPlotter::rCamCallback, this);
        lCam_sub_ = nh.subscribe("/l/tag_detections", 10, &LandmarkPlotter::lCamCallback, this);

        // Publisher for RViz markers
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("landmark_markers", 1);
        
        // Initialize transformations (dummy values, replace with actual ones)
        xyTrans_mcam_baselink_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        xyTrans_rcam_baselink_ = Eigen::Vector3d(0.0, 0.0, -M_PI / 2);
        xyTrans_lcam_baselink_ = Eigen::Vector3d(0.0, 0.0, M_PI / 2);

        lastPose_ = Eigen::Vector3d(0.0, 0.0, 0.0);  // Initial robot pose (x, y, theta)
    }

private:
    ros::Subscriber odom_sub_, mCam_sub_, rCam_sub_, lCam_sub_;
    ros::Publisher marker_pub_;
    apriltag_ros::AprilTagDetectionArray::ConstPtr mCam_msg, rCam_msg, lCam_msg;
    Eigen::Vector3d lastPose_;  // To track the robot's last odometry
    Eigen::Vector3d xyTrans_mcam_baselink_, xyTrans_rcam_baselink_, xyTrans_lcam_baselink_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Update robot pose (x, y, theta)
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;
        double theta = 2.0 * std::atan2(z, w);  // Convert quaternion to yaw

        lastPose_ = Eigen::Vector3d(x, y, theta);
        ROS_INFO("Updated pose: (%f, %f, %f)", x, y, theta);
    }

    // Camera callback functions
    void mCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        mCam_msg = msg;
    }

    void rCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        rCam_msg = msg;
    }

    void lCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        lCam_msg = msg;
    }
    void processDetections(const apriltag_ros::AprilTagDetectionArray::ConstPtr& cam_msg, 
                       const Eigen::Vector3d& xyTrans_cam_baselink,
                       std::vector<int>& Ids, 
                       std::vector<Eigen::Vector2d>& tagPoss) {
        if (cam_msg) {
            for (const auto& detection : cam_msg->detections) {
                Ids.push_back(detection.id[0]);
                double rotTheta = xyTrans_cam_baselink(2);
                Eigen::Matrix2d R;
                R << cos(rotTheta), -sin(rotTheta),
                    sin(rotTheta),  cos(rotTheta);
                Eigen::Vector2d rotP = R * Eigen::Vector2d(detection.pose.pose.pose.position.z, -detection.pose.pose.pose.position.x);
                tagPoss.push_back(rotP + xyTrans_cam_baselink.head<2>());
            }
        }
    }

    // funtion for processing tag detection topics into IDs and TagPoss
    std::pair<std::vector<int>, std::vector<Eigen::Vector2d>> getCamDetections(
        const apriltag_ros::AprilTagDetectionArray::ConstPtr& mCam_msg,
        const apriltag_ros::AprilTagDetectionArray::ConstPtr& rCam_msg,
        const apriltag_ros::AprilTagDetectionArray::ConstPtr& lCam_msg,
        const Eigen::Vector3d& xyTrans_mcam_baselink,
        const Eigen::Vector3d& xyTrans_rcam_baselink,
        const Eigen::Vector3d& xyTrans_lcam_baselink) {

        std::vector<int> Ids;
        std::vector<Eigen::Vector2d> tagPoss;

        // Process detections from each camera
        processDetections(mCam_msg, xyTrans_mcam_baselink, Ids, tagPoss);
        processDetections(rCam_msg, xyTrans_rcam_baselink, Ids, tagPoss);
        processDetections(lCam_msg, xyTrans_lcam_baselink, Ids, tagPoss);

        return std::make_pair(Ids, tagPoss);
    }

    void processDetections() {
        if (!mCam_msg || !rCam_msg || !lCam_msg) return;  // Make sure we have camera messages

        // Call the external function for processing tag detections
        auto detections = getCamDetections(mCam_msg, rCam_msg, lCam_msg, xyTrans_mcam_baselink_, xyTrans_rcam_baselink_, xyTrans_lcam_baselink_);
        
        // Access the elements of the std::pair
        const std::vector<int>& Id = detections.first;
        const std::vector<Eigen::Vector2d>& tagPos = detections.second;

        // Create a MarkerArray for the tag positions
        visualization_msgs::MarkerArray markerArray;

        for (size_t i = 0; i < Id.size(); ++i) {
            int tagId = Id[i];
            Eigen::Vector2d tagPosInMapFrame = transformToMapFrame(tagPos[i]);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";  // Use map frame
            marker.header.stamp = ros::Time::now();
            marker.ns = "tags";
            marker.id = tagId;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            // Position of the tag
            marker.pose.position.x = tagPosInMapFrame.x();
            marker.pose.position.y = tagPosInMapFrame.y();
            marker.pose.position.z = 0.0;  // Assume 2D for now

            // Orientation
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Scale
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            // Color (make the marker blue)
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;

            markerArray.markers.push_back(marker);
        }

        // Publish the markers to RViz
        marker_pub_.publish(markerArray);
    }

    // Transform tag position from the robot frame to the map frame based on odometry
    Eigen::Vector2d transformToMapFrame(const Eigen::Vector2d& tagPos) {
        double theta = lastPose_.z();
        Eigen::Rotation2Dd rotation(theta);
        Eigen::Vector2d rotatedTagPos = rotation * tagPos;
        Eigen::Vector2d tagPosInMapFrame = rotatedTagPos + lastPose_.head<2>();  // Transform to map frame
        return tagPosInMapFrame;
    }

public:
    // Call this method periodically to process and publish detections
    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            processDetections();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "landmark_plotter");
    ros::NodeHandle nh;

    LandmarkPlotter plotter(nh);
    plotter.spin();

    return 0;
}
