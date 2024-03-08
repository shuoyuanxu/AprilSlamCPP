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

using namespace my_gtsam_project;

GTSAMNode::GTSAMNode(const ros::NodeHandle& node_handle) :
    nh_(node_handle),
    tf_listener_(tf_buffer_),
    pose_index_(1) { // Initial pose index set to 1

    // Initialize GTSAM components
    initializeGTSAM();

    // Subscribe to odometry topic
    odom_sub_ = nh_.subscribe("/odometry/filtered", 1000, &GTSAMNode::odomCallback, this);
}

void GTSAMNode::initializeGTSAM() {
    // Setup GTSAM ISAM2 parameters
    gtsam::ISAM2Params parameters;
    parameters.setRelinearizeThreshold(0.1);
    isam_ = gtsam::ISAM2(parameters);

    // Initial pose and noise model setup could go here
    // Example:
    // auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1)); // Just as an example
    // initial_estimates_.insert(symbol('x', 0), gtsam::Pose2(0.0, 0.0, 0.0));
    // graph_.add(gtsam::PriorFactor<gtsam::Pose2>(symbol('x', 0), gtsam::Pose2(0.0, 0.0, 0.0), noise));
}

// Additional method implementations will go here...

