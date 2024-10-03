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

Download the code and put it into your catkin workspace, then run catkin_make to run it.

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

The system applies odometry constraints (between consecutive poses) and bearing-range constraints (between poses and landmarks). Both SAM and ISAM2 optimizers can be utilized. Here’s a simple comparison between these two optimizers:

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

### 1. relPoseFG

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

### 2. AprilSlam Node Initialization

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

### 3. Optimization

The `SAMOptimise` function performs batch optimization of the factor graph using GTSAM’s Levenberg-Marquardt optimizer. After the optimization, the input estimates are updated and will be used for the next iteration. Due to the fact that SAM is normally computationally heavy, we recommand to use our pruning function 'pruneGraphByPoseCount' to manage the total size of the factor graph when trying to run it in real time.

```cpp
void aprilslamcpp::SAMOptimise() {
    gtsam::Levenberg-MarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    keyframeEstimates_ = batchOptimizer.optimize();

    // Prune the graph based on the number of poses
    if (useprunebysize) {
    pruneGraphByPoseCount(maxfactors);
    }
}
```
The `ISAM2Optimise` function performs incremental optimization of the factor graph using GTSAM’s iSAM2 optimizer. Estimates on all poses and landmarks plus the entire graph is stored in a 'gtsam::ISAM2 isam_' variable, which can only be updated but not trimed. Therefore, after every ISAM2 iteration, historic estimates and graph needs to be cleaned to avoid repetitive data appeared in isam_ variable. Since ISAM2 is not computationally heavy, no graph management approach is required

```cpp
void aprilslamcpp::ISAM2Optimise() {    
    // Perform batch optimization once if required
    if (batchOptimisation_) {
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
        keyframeEstimates_ = batchOptimizer.optimize();
        batchOptimisation_ = false; // Only do this once
    }

    // Incrementally update the iSAM2 instance with new measurements
    isam_.update(keyframeGraph_, keyframeEstimates_);

    // Clear the graph and estimates for the next iteration
    keyframeEstimates_.clear();
    keyframeGraph_.resize(0);
}
```

### 6. Odometry Processing

This function is where the factor graph is built and corresponding estimates are inserted whenever a new steam of odometry data comes in:

```cpp
    odom_sub_ = nh_.subscribe(odom_topic, 10, &aprilslamcpp::addOdomFactor, this);
```

Once the odom are received, the soonest tag detections will be used to formulate the factor graph, once this function is finished running with the current odometry, a estimates (`keyframeEstimates_`) containing all the pos and landmarks, as well as a factor graph (`keyframeGraph_`) containing pose prior, landmark prior (`PriorFactor`), lanmark observation (`BearingRangeFactor`), and pos to pos factor (`BetweenFactor`) are formulated

```cpp
void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
	// Convert the incoming odometry message to a simpler (x, y, theta) format using a previously defined method
	gtsam::Pose2 poseSE2 = translateOdomMsg(msg);
	...
	keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
	...
	if (mCam_msg && rCam_msg && lCam_msg) {  // Ensure the messages have been received
        auto detections = getCamDetections(mCam_msg, rCam_msg, lCam_msg, mcam_baselink_transform, rcam_baselink_transform, lcam_baselink_transform);
		// Access the elements of the std::pair
		...
        if (!Id.empty()) {
            for (size_t n = 0; n < Id.size(); ++n) {
                int tag_number = Id[n];        
                ...
                gtsam::Point2 priorLand(rotatedPosition.x() + lastPose_.x(), rotatedPosition.y() + lastPose_.y());
				...
                // Construct the landmark key
                gtsam::Symbol landmarkKey('L', tag_number);  
				...
                // Check if the landmark has been observed before
                if (detectedLandmarksHistoric.find(landmarkKey) != detectedLandmarksHistoric.end()) {
                    // Existing landmark
                    gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                        gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                    );
                    gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                    // Threshold for ||projection - measurement||
                    if (fabs(error[0]) < add2graph_threshold) keyframeGraph_.add(factor);
                } 
                else {
                    // If the current landmark was not detected in the calibration run 
                    // Or it's on calibration mode
                    if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                    // New landmark detected
                    detectedLandmarksHistoric.insert(landmarkKey);
                    // Check if the key already exists in keyframeEstimates_ before inserting
                    if (keyframeEstimates_.exists(landmarkKey)) {
                    } else {
                        keyframeEstimates_.insert(landmarkKey, priorLand); // Simple initial estimate
                    }
					...
                    // Add a prior for the landmark position to help with initial estimation.
                    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(
                        landmarkKey, priorLand, pointNoise)
                    );
                    }
                    // Add a bearing-range observation for this landmark to the graph
                    gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                        gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                    );
                    keyframeGraph_.add(factor);
                }
				...
			}
        }
    }      
}
```
It is worth noting that `gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);` this line is used for identifying whjether a landmark is too much of an outliner to be added to the factor graph.


### **8. Calibration**

Code will wait until the bag finished playing and a graph containing all pose, odometry, landmarks, and landmark detections is built. Then the SAMOptimize function will run once to obtain the landmark locations.
![image](https://github.com/user-attachments/assets/33a27ead-4368-49e7-b587-ae3cf211938c)

The condition for bag finished is trigger by a preset time interval that no new detections are received. 

### **9. Localization**

The localization feature leverages previously mapped landmark locations to estimate the robot’s pose in real-time. Here's a breakdown of how the localization is implemented in the system:
![image](https://github.com/user-attachments/assets/4d746c1a-5932-4712-9f11-a94b6fa2dc4c)

#### Pre-mapped Landmark Loading and Incorporating into the System

The system can load pre-mapped landmark locations from a CSV file, which can be used as priors for localization. When initializing the SLAM system, the pre-mapped landmarks are loaded and incorporated as priors into the GTSAM factor graph.

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

There are various confugurations can be applied in Localization algorithm to balance the efficiency and accuracy, so far the SAM with pruning works the best, heres the architecture: 

---

## **5. How to Run**

Launch the SLAM node:

```bash
roslaunch aprilslamcpp run_localisation.launch 

roslaunch aprilslamcpp run_calibration.launch 
```

Start the cameras and odometry data sources or start replaying the bag file (e.g. rosbag play matt_DLO.bag).

View the output in RViz.

---

## **6. Future Work**

- **Loop Closure Enhancements**: Current loop closure detection is based on re-observing landmarks. We can integrate feature-based methods for more robust detection.
- **Dynamic Environments**: Adapting the SLAM algorithm for dynamic environments where landmarks move or disappear.







