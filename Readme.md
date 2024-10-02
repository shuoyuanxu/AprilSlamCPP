 AprilSlam - ROS-based SLAM using AprilTags and GTSAM

## Overview

This project implements a ROS-based Simultaneous Localization and Mapping (SLAM) system using AprilTags for feature detection and GTSAM for graph-based optimization. The system estimates the robot’s trajectory and maps landmarks using multiple cameras.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Core Components](#core-components)
- [Mathematical Foundation](#mathematical-foundation)
- [Key Functions and Code Structure](#key-functions-and-code-structure)
  - [wrapToPi](#1-wraptopi)
  - [relPoseFG](#2-relposefg)
  - [AprilSlam Node Initialization](#3-aprilslam-node-initialization)
  - [GTSAM Optimization](#4-gtsam-optimization)
  - [Visualization](#5-visualization)
  - [Odometry Processing](#6-odometry-processing)
  - [Landmark Processing](#7-landmark-processing)
- [How to Run](#how-to-run)
- [Future Work](#future-work)

---

## Installation

Ensure that the following dependencies are installed:

- **ROS** (Robot Operating System) - Noetic or Melodic
- **GTSAM** (Georgia Tech Smoothing And Mapping library)
- **AprilTag ROS** - For AprilTag detection
- **Eigen** - For matrix computations

### Steps:

1. Clone the repository into your catkin workspace:
   ```bash
   git clone https://github.com/your-repo/april_slam_cpp.git

    Install dependencies:

    bash

rosdep install --from-paths src --ignore-src -r -y

Build the workspace:

bash

catkin_make

Source the workspace:

bash

    source devel/setup.bash

Core Components
1. AprilTag Detection

The system uses AprilTags for robust feature detection. Three camera topics (mCam, rCam, lCam) are subscribed to detect AprilTags in their respective fields of view.
2. GTSAM Optimization

GTSAM performs factor graph-based optimization using:

    Odometry factors for pose estimation
    Bearing-Range factors for landmarks

Mathematical Foundation
Pose Representation

The pose of the robot is represented using gtsam::Pose2, which includes:

    (x, y): Position of the robot in the 2D plane
    θ: Orientation of the robot

Relative Pose Calculation

The function relPoseFG computes the relative pose between two Pose2 objects. It returns the relative distance, adjusted for orientation, assuming that the robot cannot move sideways.
Graph-Based SLAM

    The system uses odometry constraints (between consecutive poses) and bearing-range constraints (between poses and landmarks).
    GTSAM's ISAM2 optimizer is used for real-time, incremental optimization of the factor graph.

Key Functions and Code Structure
1. wrapToPi

This function normalizes any angle to the range [−π,π][−π,π]. This is crucial for ensuring that angular differences are always within a reasonable range for optimization.

cpp

double wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    return angle - M_PI;
}

2. relPoseFG

The function computes the relative pose between two gtsam::Pose2 objects by converting the difference in poses into robot-relative coordinates.

    Input: Two Pose2 objects (lastPoseSE2 and PoseSE2).
    Output: Relative Pose2 that represents the robot's motion from lastPoseSE2 to PoseSE2.

cpp

gtsam::Pose2 relPoseFG(const gtsam::Pose2& lastPoseSE2, const gtsam::Pose2& PoseSE2) {
    double dx = PoseSE2.x() - lastPoseSE2.x();
    double dy = PoseSE2.y() - lastPoseSE2.y();
    double dtheta = wrapToPi(PoseSE2.theta() - lastPoseSE2.theta());

    double theta = lastPoseSE2.theta();
    double dx_body = std::cos(theta) * dx + std::sin(theta) * dy;
    return gtsam::Pose2(dx_body, 0, dtheta);
}

3. AprilSlam Node Initialization

In the constructor of aprilslamcpp, multiple parameters are read from ROS parameters to configure the system, such as:

    Noise models
    Thresholds for stationary detection
    Paths for saving/loading landmarks
    Subscriber and publisher topics

cpp

aprilslamcpp::aprilslamcpp(ros::NodeHandle node_handle)
    : nh_(node_handle), tf_listener_(tf_buffer_) {
    nh_.getParam("odom_topic", odom_topic);
    nh_.getParam("trajectory_topic", trajectory_topic);
    nh_.getParam("frame_id", frame_id);

    // Noise models
    nh_.getParam("noise_models/odometry", odometry_noise);
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << odometry_noise[0], odometry_noise[1], odometry_noise[2]).finished());

    // Subscribers and Publishers
    mCam_subscriber = nh_.subscribe(mCam_topic, 1000, &aprilslamcpp::mCamCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1, true);
}

4. GTSAM Optimization

The SAMOptimise function performs batch optimization of the factor graph using GTSAM’s Levenberg-Marquardt optimizer. After the optimization, the updated estimates are stored for the next iteration.

cpp

void aprilslamcpp::SAMOptimise() {
    gtsam::Levenberg-MarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    keyframeEstimates_ = batchOptimizer.optimize();
}

5. Visualization

This function publishes loop closure visualization markers in RViz. It draws a green line between the current pose and the keyframe pose when a loop closure is detected.

cpp

void visualizeLoopClosure(ros::Publisher& lc_pub, const gtsam::Pose2& currentPose, const gtsam::Pose2& keyframePose, int currentPoseIndex, const std::string& frame_id) {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = ros::Time::now();
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.color.g = 1.0;
    line_marker.scale.x = 0.05;

    geometry_msgs::Point p1, p2;
    p1.x = keyframePose.x(); p1.y = keyframePose.y();
    p2.x = currentPose.x(); p2.y = currentPose.y();

    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);

    lc_pub.publish(line_marker);
}

6. Odometry Processing

This function processes the incoming odometry messages, updates the pose, and adds a factor to the graph for odometry constraints.

cpp

void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);
    if (position_change < stationary_position_threshold) return;

    index_of_pose++;
    gtsam::Pose2 predictedPose = lastPose_.compose(relPoseFG(lastPoseSE2_, poseSE2));
    keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
}

7. Landmark Processing

Landmark detection is handled by AprilTag detections. The detected landmarks are added to the factor graph as bearing-range factors.

cpp

void aprilslamcpp::processLandmarks(const std::vector<int>& Id, const std::vector<Eigen::Vector2d>& tagPos) {
    for (size_t n = 0; n < Id.size(); ++n) {
        gtsam::Symbol landmarkKey('L', Id[n]);
        gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
            gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
        );
        keyframeGraph_.add(factor);
    }
}

How to Run

    Launch the SLAM node:

    bash

    roslaunch aprilslamcpp slam.launch

    Start the cameras and odometry data sources.

    View the output in RViz by subscribing to the /loop_closure_markers topic.

Future Work

    Loop Closure Enhancements: Current loop closure detection is based on re-observing landmarks. We can integrate feature-based methods for more robust detection.
    Dynamic Environments: Adapting the SLAM algorithm for dynamic environments where landmarks move or disappear.

