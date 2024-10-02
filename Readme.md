# Tag aided localisation algorithm for robots operating in polytunnels

## Overview

This project implements a ROS-based Simultaneous Localization and Mapping (SLAM) system that uses AprilTags as landmarks and GTSAM for graph-based optimization. The system estimates the robot’s trajectory and maps landmarks using multiple cameras. To balance computational cost and localization and mapping accuracy, we employ two similar yet distinct strategies: one for landmark mapping and another for localization.

### Calibration

Calibration is performed using SAM (Smoothing and Mapping), where all poses, landmarks, and observations are incorporated into a factor graph. A single optimization process is then carried out to achieve a globally optimal solution.

### Localization

Localization utilizes prior knowledge of relatively accurate landmark positions. Various optimization techniques and strategies can be employed to balance accuracy and efficiency, which will be discussed in detail in the following sections.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Core Components](#core-components)
- [Mathematical Foundation](#mathematical-foundation)
- [Key Functions and Code Structure](#key-functions-and-code-structure)
  - [wrapToPi](#1-wraptopi)
  - [relPoseFG](#2-relposefg)
  - [AprilSlam Node Initialization](#3-aprilslam-node-initialization)
  - [Optimization](#4-gtsam-optimization)
  - [Visualization](#5-visualization)
  - [Odometry Processing](#6-odometry-processing)
  - [Landmark Processing](#7-landmark-processing)
  - [Calibration](#8-calibration)
  - [Localisation](#9-localisation)
- [How to Run](#how-to-run)
- [Tunning](#tunning)
- [Future Work](#future-work)

---

## **1. Installation**

Ensure that the following dependencies are installed:

- **ROS** (Robot Operating System) - Noetic or Melodic
- **GTSAM** (Georgia Tech Smoothing And Mapping library)
- **Eigen** - For matrix computations

### Some Remarks: 

1. error: ‘optional’ in namespace ‘std’ does not name a template type
	std::optional is c++17 only, add this line to your cmake file:

```set(CMAKE_CXX_STANDARD 17)```

2. error: static assertion failed: Error: GTSAM was built against a different version of Eigen

	need to rebuild:
```cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..```

3. error: gtsam_wrapper.mexa64 unable to find libgtsam.so.4

The default search directory of gtsam_wrapper.mexa64 is /usr/lib/ yet all related libs are installed to /usr/local/lib. All corresponding files (the ones mentioned in Matlab error message) needs to be copied to /usr/lib/

```
sudo cp /usr/local/lib/libgtsam.so.4 /usr/lib/
sudo cp /usr/local/lib/libgtsam.so.4 /usr/lib/
```
		
4. Matlab toolbox: cmake -D GTSAM_INSTALL_MATLAB_TOOLBOX=1 ..
	copy the toolbox from usr/local/ to work directory, then add the folder to path in Matlab

5. To hide "Warning: TF_REPEATED_DATA ignoring data with redundant timestamp" error in terminal
```
source devel/setup.bash
rosrun aprilslamcpp aprilslamcpp 2> >(grep -v TF_REPEATED_DATA buffer_core)
rosbag play --pause rerecord_3_HDL.bag
```
6. Compile:
```
catkin_make --pkg AprilSlamCPP
```

---

## **2. Core Components**

### 1. AprilTag Detection

The system uses AprilTags for robust feature detection. Three camera topics (`mCam`, `rCam`, `lCam`) are subscribed to detect AprilTags in their respective fields of view.

### 2. Odometry

The system utilizes odometry data from various sensors, including wheel encoders, IMU, GPS, or LiDAR, to compute relative poses for constructing the factor graph. For optimal results, we recommend using accurate odometry sources, such as LiDAR, during the calibration phase. As for localization, other odometry sources can be effectively, as long as the sensor provides reasonably reliable and consistent data. This flexibility allows the system to accommodate different sensor configurations, making it adaptable to various environments and use cases.

### 3. GTSAM Optimization

GTSAM performs factor graph-based optimization using:

- Between factors between poses (from Odometry) 
- Bearing-Range factors for landmarks
- Priors of initial pose and landmarks

---

## **3. Mathematical Foundation**

### Assumptions

The algorithm operates on a 2D plane, assuming that vertical differences do not impact performance. The robot's pose is represented using `gtsam::Pose2`, which includes:

- `(x, y)`: The robot's position in the 2D plan
- `θ`: The robot's orientation
- The function `relPoseFG` calculates the relative pose between two `Pose2` objects, returning the relative distance and adjusting for orientation. It assumes that **the robot cannot move sideways**.

### Graph-Based SLAM

The system applies odometry constraints (between consecutive poses) and bearing-range constraints (between poses and landmarks). Both SAM and ISAM2 optimizers are utilized. Here’s a simple comparison between these two optimizers:

| Feature                | SAM (Batch Optimization)                     | iSAM2 (Incremental Optimization)        |
|------------------------|----------------------------------------------|-----------------------------------------|
| **Optimization Type**   | Batch (whole graph at once)                  | Incremental (updates relevant portions) |
| **Computation Time**    | High (grows with graph size)                 | Low (optimized for real-time updates)   |
| **Real-time Suitability**| No                                           | Yes                                     |
| **Memory Usage**        | High (stores entire graph)                   | Lower (incremental updates)             |
| **Algorithm**           | Batch least squares (e.g., Levenberg-Marquardt, Gauss-Newton) | Incremental smoothing with selective relinearization |
| **Use Case**            | Best for offline or small-scale optimization | Ideal for real-time applications like SLAM |

**SAM**: Optimizing the whole graph

`min_{x, l} ∑ || z_i - h(x_i, l_i) ||^2_Σ_i`

**ISAM2**: Incremental Update:

`x_{t+1} = x_t + \Delta x`

- Add New Factors: New factors (e.g., new odometry or landmark observations) are added to the factor graph when a new measurement is received.

- Relinearization: Only a subset of variables is relinearized, meaning that only variables affected (determined by ISAM2) by the new measurements are recalculated. This significantly reduces computational complexity compared to recalculating all variables.

- Bayes Tree Update: iSAM2 uses a Bayes Tree to represent the factor graph and efficiently update the system. The Bayes Tree organizes the factor graph into cliques, allowing for fast updates when new measurements are added.

---

## **4. Key Functions and Code Structure**

### 2. relPoseFG

Our odometry does not give relative pose directly, instead it gives pose estimations. Therefore, a function computes the relative pose between two `gtsam::Pose2` objects is required:

- **Input**: Two `Pose2` objects (`lastPoseSE2` and `PoseSE2`)
- **Output**: Relative `Pose2` that represents the robot's motion from `lastPoseSE2` to `PoseSE2`

```cpp
gtsam::Pose2 relPoseFG(const gtsam::Pose2& lastPoseSE2, const gtsam::Pose2& PoseSE2) {
    double dx = PoseSE2.x() - lastPoseSE2.x();
    double dy = PoseSE2.y() - lastPoseSE2.y();
    double dtheta = wrapToPi(PoseSE2.theta() - lastPoseSE2.theta());

    double theta = lastPoseSE2.theta();
    double dx_body = std::cos(theta) * dx + std::sin(theta) * dy;
    return gtsam::Pose2(dx_body, 0, dtheta);
}
```

### 3. AprilSlam Node Initialization

In the constructor of `aprilslamcpp`, multiple parameters are read from ROS parameters to configure the system, such as:

- Noise models
- Thresholds for stationary detection
- Paths for saving/loading landmarks
- Subscriber and publisher topics

```cpp
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
```

### 4. Optimization



The `SAMOptimise` function performs batch optimization of the factor graph using GTSAM’s Levenberg-Marquardt optimizer. After the optimization, the updated estimates are stored for the next iteration.

```cpp
void aprilslamcpp::SAMOptimise() {
    gtsam::Levenberg-MarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    keyframeEstimates_ = batchOptimizer.optimize();
}
```

### 5. Visualization

This function publishes loop closure visualization markers in RViz. It draws a green line between the current pose and the keyframe pose when a loop closure is detected.

```cpp
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
```

### 6. Odometry Processing

This function processes the incoming odometry messages, updates the pose, and adds a factor to the graph for odometry constraints.

```cpp
void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);
    if (position_change < stationary_position_threshold) return;

    index_of_pose++;
    gtsam::Pose2 predictedPose = lastPose_.compose(relPoseFG(lastPoseSE2_, poseSE2));
    keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
}
```

### 7. Landmark Processing

Landmark detection is handled by AprilTag detections. The detected landmarks are added to the factor graph as bearing-range factors.

```cpp
void aprilslamcpp::processLandmarks(const std::vector<int>& Id, const std::vector<Eigen::Vector2d>& tagPos) {
    for (size_t n = 0; n < Id.size(); ++n) {
        gtsam::Symbol landmarkKey('L', Id[n]);
        gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
            gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
        );
        keyframeGraph_.add(factor);
    }
}
```

---

### **8. Calibration**

Code will wait until the bag finished playing and a graph containing all pose, odometry, landmarks, and landmark detections is built. Then the SAMOptimize function will run once to obtain the landmark locations.

####  SAM (Smoothing and Mapping)

    Batch Optimization: SAM refers to the process of performing batch optimization. In this approach, all the factors and measurements are collected, and optimization is performed on the entire graph at once. This is known as batch least squares optimization (often using methods like Levenberg-Marquardt or Gauss-Newton).
    Full Graph Optimization: In batch optimization, the entire factor graph is optimized every time new measurements are added. This results in a globally optimal solution at each optimization step but is computationally expensive.
    Not Real-time: Because SAM performs optimization over the entire graph every time, it can become slow and resource-heavy as the graph grows, making it unsuitable for real-time applications or systems with large datasets.
    Higher Memory Usage: Since batch optimization works on the full dataset, it requires more memory to store and process the entire factor graph.

#### Pre-mapped Landmark Loading

The system can load pre-mapped landmark locations from a CSV file, which can be used as priors for localization.

```cpp
// Load pre-mapped landmarks from a CSV file
std::map<int, gtsam::Point2> loadLandmarksFromCSV(const std::string& filepath) {
    // Implementation for loading the landmarks from CSV
    std::map<int, gtsam::Point2> landmarks;
    // ... CSV parsing and loading into landmarks map
    return landmarks;
}
```

#### Incorporating Pre-mapped Landmarks into the SLAM System

When initializing the SLAM system, the pre-mapped landmarks are loaded and incorporated as priors into the GTSAM factor graph.

```cpp
// During the constructor of aprilslamcpp
if (usepriortagtable) {
    std::map<int, gtsam::Point2> savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);
    for (const auto& landmark : savedLandmarks) {
        gtsam::Symbol landmarkKey('L', landmark.first);
        keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(landmarkKey, landmark.second, pointNoise));
        keyframeEstimates_.insert(landmarkKey, landmark.second);
        landmarkEstimates.insert(landmarkKey, landmark.second);
    }
}
```

#### Localization using Landmark Observations

When the robot observes a known landmark, the system uses a bearing-range factor to update the robot’s pose and ensure alignment with the pre-mapped landmarks.

```cpp
// Processing landmark observations for localization
void aprilslamcpp::processLandmarks(const std::vector<int>& Id, const std::vector<Eigen::Vector2d>& tagPos) {
    for (size_t n = 0; n < Id.size(); ++n) {
        gtsam::Symbol landmarkKey('L', Id[n]);

        // If the landmark exists in the pre-mapped landmarks
        if (landmarkEstimates.exists(landmarkKey)) {
            // Compute bearing and range to the observed landmark
            double bearing = std::atan2(tagPos , tagPos );
            double range = std::sqrt(tagPos  * tagPos  + tagPos  * tagPos );

            // Add a bearing-range factor for this observation
            keyframeGraph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
                gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise));
        }
    }
}
```

#### Updating the Pose Estimate

As the system detects known landmarks and processes them, the robot’s pose is continuously refined based on the pre-mapped landmark locations.

```cpp
// Update the pose using ISAM2 or batch optimization
void aprilslamcpp::ISAM2Optimise() {
    isam_.update(keyframeGraph_, keyframeEstimates_);
    auto result = isam_.calculateEstimate();

    // Update the pose and publish the results
    aprilslam::publishPath(path_pub_, result, index_of_pose, frame_id);
}
```

---
---

### **9. Localization**

The localization feature leverages previously mapped landmark locations to estimate the robot’s pose in real-time. Here's a breakdown of how the localization is implemented in the system:

#### iSAM2 (Incremental Smoothing and Mapping)

    Incremental Optimization: iSAM2 is designed for incremental optimization, meaning it only updates parts of the factor graph as new measurements come in. This makes it more efficient for real-time applications.
    Efficient Updates: iSAM2 performs partial relinearization and reordering of the factor graph. It doesn’t re-optimize the entire graph every time but incrementally updates the parts affected by new measurements. This allows for faster updates with minimal computational cost.
    Real-time Capable: iSAM2 is specifically designed for real-time applications like SLAM. By only updating relevant portions of the graph, it can maintain a good approximation of the solution without needing to recompute everything.
    Lower Memory Usage: Because iSAM2 only processes parts of the graph incrementally, it has lower memory requirements compared to batch SAM.
    ISAM2 Improvements: iSAM2 includes several improvements over its predecessor iSAM, including better handling of relinearization, improved efficiency in graph reordering, and the ability to work on more complex graphs.

#### Pre-mapped Landmark Loading

The system can load pre-mapped landmark locations from a CSV file, which can be used as priors for localization.

```cpp
// Load pre-mapped landmarks from a CSV file
std::map<int, gtsam::Point2> loadLandmarksFromCSV(const std::string& filepath) {
    // Implementation for loading the landmarks from CSV
    std::map<int, gtsam::Point2> landmarks;
    // ... CSV parsing and loading into landmarks map
    return landmarks;
}
```

#### Incorporating Pre-mapped Landmarks into the SLAM System

When initializing the SLAM system, the pre-mapped landmarks are loaded and incorporated as priors into the GTSAM factor graph.

```cpp
// During the constructor of aprilslamcpp
if (usepriortagtable) {
    std::map<int, gtsam::Point2> savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);
    for (const auto& landmark : savedLandmarks) {
        gtsam::Symbol landmarkKey('L', landmark.first);
        keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(landmarkKey, landmark.second, pointNoise));
        keyframeEstimates_.insert(landmarkKey, landmark.second);
        landmarkEstimates.insert(landmarkKey, landmark.second);
    }
}
```

#### Localization using Landmark Observations

When the robot observes a known landmark, the system uses a bearing-range factor to update the robot’s pose and ensure alignment with the pre-mapped landmarks.

```cpp
// Processing landmark observations for localization
void aprilslamcpp::processLandmarks(const std::vector<int>& Id, const std::vector<Eigen::Vector2d>& tagPos) {
    for (size_t n = 0; n < Id.size(); ++n) {
        gtsam::Symbol landmarkKey('L', Id[n]);

        // If the landmark exists in the pre-mapped landmarks
        if (landmarkEstimates.exists(landmarkKey)) {
            // Compute bearing and range to the observed landmark
            double bearing = std::atan2(tagPos , tagPos );
            double range = std::sqrt(tagPos  * tagPos  + tagPos  * tagPos );

            // Add a bearing-range factor for this observation
            keyframeGraph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
                gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise));
        }
    }
}
```

#### Updating the Pose Estimate

As the system detects known landmarks and processes them, the robot’s pose is continuously refined based on the pre-mapped landmark locations.

```cpp
// Update the pose using ISAM2 or batch optimization
void aprilslamcpp::ISAM2Optimise() {
    isam_.update(keyframeGraph_, keyframeEstimates_);
    auto result = isam_.calculateEstimate();

    // Update the pose and publish the results
    aprilslam::publishPath(path_pub_, result, index_of_pose, frame_id);
}
```

---

---

## **5. How to Run**

Launch the SLAM node:

```bash
roslaunch aprilslamcpp slam.launch
```

Start the cameras and odometry data sources.

View the output in RViz by subscribing to the `/loop_closure_markers` topic.

---

## **6. Future Work**

- **Loop Closure Enhancements**: Current loop closure detection is based on re-observing landmarks. We can integrate feature-based methods for more robust detection.
- **Dynamic Environments**: Adapting the SLAM algorithm for dynamic environments where landmarks move or disappear.







