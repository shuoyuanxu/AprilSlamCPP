#include "AprilSlamCPP.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/navigation/NonlinearFactorGraph.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>

using namespace aprislamcpp;
using symbol_shorthand::X; // For pose variables
using symbol_shorthand::L; // For landmark variables


AprilSlamCPP::AprilSlamCPP(const ros::NodeHandle& node_handle) :
    nh_(node_handle),
    tf_listener_(tf_buffer_),
    pose_index_(1) { // Initial pose index set to 1

    // Initialize GTSAM components
    initializeGTSAM();

    // Subscribe to odometry topic
    odom_sub_ = nh_.subscribe("/odometry/filtered", 1000, &AprilSlamCPP::odomCallback, this);
}


void AprilSlamCPP::initializeGTSAM() {
    // initial_estimates_.insert(symbol('x', 0), gtsam::Pose2(0.0, 0.0, 0.0));
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam:::Vector(3) << 0.0001, 0.01, 0.0001).finished());  // Odometry measurement noise.
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector3 << 0.1, 3, 0.1).finished());  // Initial pose estimate noise.
    brNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector3 << 0.1, 0.8).finished());  // Bearing and range measurement noise.
    pointNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector3 << 10, 10).finished());  // Landmark position noise.
    
    // Initialize the factor graph
    graph_ = gtsam::NonlinearFactorGraph();
    index_of_pose = 1;

    // Initialize ISAM2 with parameters.
    gtsam::ISAM2Params parameters;
    parameters.setRelinearizeThreshold = 0.1;  // Threshold for re-linearization
    isam_ = gtsam::ISAM2(parameters);
    batchInitialization_ = true;  // Flag to indicate if batch initialization is required.

    // Debugging/Initialization message.
    ROS_INFO("Initialised GTSAM SLAM system.");


    for (int j = 0; j < 89; ++j) {possibleIds.push_back("tag_" + std::to_string(j))};
    ROS_INFO_STREAM("Possible landmark IDs initialised.");



    tfListener_(tf_buffer_), 
    landCount_ = 0; 
    initializeGTSAM()



    
    // Add a prior on the first pose at the origin
    gtsam::Pose2 priorPose(0.0, 0.0, 0.0);
    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(X(1), priorPose, priorNoise));

    initial_estimates_.insert(X(1), priorPose);


}


void AprilSlamNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Convert and process odometry messages
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "april_slam_cpp");
    ros::NodeHandle nh;
    AprilSlamNode node(nh);

    ros::spin();
    return 0;
}

