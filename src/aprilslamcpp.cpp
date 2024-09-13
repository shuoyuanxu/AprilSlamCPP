#include "aprilslamheader.h"
#include "publishing_utils.h"

namespace aprilslam {
// Utility function to normalize an angle to the range [-pi, pi]
double wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    return angle - M_PI;
}   

// Computes the relative pose between two Pose2 objects
gtsam::Pose2 relPoseFG(const gtsam::Pose2& lastPoseSE2, const gtsam::Pose2& PoseSE2) {
    double dx = PoseSE2.x() - lastPoseSE2.x();
    double dy = PoseSE2.y() - lastPoseSE2.y();
    double dtheta = wrapToPi(PoseSE2.theta() - lastPoseSE2.theta());

    // Compute the distance moved along the robot's forward direction
    double distance = std::sqrt(dx * dx + dy * dy);
    double direction = std::atan2(dy, dx);
    // return gtsam::Pose2(distance, 0, dtheta);

    // Adjust the distance based on the robot's heading to account for backward movement
    double theta = lastPoseSE2.theta();
    double dx_body = std::cos(theta) * dx + std::sin(theta) * dy;
    double dy_body = -std::sin(theta) * dx + std::cos(theta) * dy;

    // Return the relative pose assuming robot cant move sideways: dy = 0
    return gtsam::Pose2(dx_body, 0, dtheta);
}

// Constructor
aprilslamcpp::aprilslamcpp(ros::NodeHandle node_handle, ros::Duration cache_time)
    : nh_(node_handle), tf_buffer_(cache_time), tf_listener_(tf_buffer_){ 
    
    // Read topics and corresponding frame
    std::string odom_topic, trajectory_topic;
    nh_.getParam("odom_topic", odom_topic);
    nh_.getParam("trajectory_topic", trajectory_topic);
    nh_.getParam("frame_id", frame_id);
    nh_.getParam("robot_frame", robot_frame);

    // Read batch optimization flag
    nh_.getParam("batch_optimisation", batchOptimisation_);

    // Read noise models
    std::vector<double> odometry_noise, prior_noise, bearing_range_noise, point_noise;
    nh_.getParam("noise_models/odometry", odometry_noise);
    nh_.getParam("noise_models/prior", prior_noise);
    nh_.getParam("noise_models/bearing_range", bearing_range_noise);
    nh_.getParam("noise_models/point", point_noise);


    // Read transformation search range (seconds) 
    nh_.getParam("transformation_search_range", transformation_search_range);

    // Read error thershold for a landmark to be added to the graph
    nh_.getParam("add2graph_threshold", add2graph_threshold);

    // Read Prune conditions
    nh_.getParam("timewindow", timeWindow);
    nh_.getParam("maxfactors", maxfactors);
    nh_.getParam("useprunebytime", useprunebytime);
    nh_.getParam("useprunebysize", useprunebysize);
    
    // Read loop closure parameters
    nh_.getParam("loopClosureEnableFlag", loopClosureEnableFlag);
    nh_.getParam("loopClosureFrequency", loopClosureFrequency);
    nh_.getParam("surroundingKeyframeSize", surroundingKeyframeSize);
    nh_.getParam("historyKeyframeSearchRadius", historyKeyframeSearchRadius);
    nh_.getParam("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
    nh_.getParam("historyKeyframeSearchNum", historyKeyframeSearchNum);

    // Keyframe parameters
    nh_.getParam("distanceThreshold", distanceThreshold);
    nh_.getParam("rotationThreshold", rotationThreshold);

        // Stationay conditions
    nh_.getParam("stationary_position_threshold", stationary_position_threshold);
    nh_.getParam("stationary_rotation_threshold", stationary_rotation_threshold);

    // Read calibration and localisation settings
    std::string package_path = ros::package::getPath("aprilslamcpp");
    std::string save_path, load_path;
    nh_.getParam("pathtosavelandmarkcsv", save_path);
    nh_.getParam("pathtoloadlandmarkcsv", load_path);

    // Construct the full paths
    pathtosavelandmarkcsv = package_path + "/" + save_path;
    pathtoloadlandmarkcsv = package_path + "/" + load_path;
    nh_.getParam("savetaglocation", savetaglocation);
    nh_.getParam("usepriortagtable", usepriortagtable);

    // Camera transformation parameters
    nh_.getParam("camera_parameters/xyTrans_lcam_baselink", xyTrans_lcam_baselink);
    nh_.getParam("camera_parameters/xyTrans_rcam_baselink", xyTrans_rcam_baselink);
    nh_.getParam("camera_parameters/xyTrans_mcam_baselink", xyTrans_mcam_baselink);
    // Convert to Eigen::Vector3d
    mcam_baselink_transform = Eigen::Vector3d(xyTrans_mcam_baselink[0], xyTrans_mcam_baselink[1], xyTrans_mcam_baselink[2]);
    rcam_baselink_transform = Eigen::Vector3d(xyTrans_rcam_baselink[0], xyTrans_rcam_baselink[1], xyTrans_rcam_baselink[2]);
    lcam_baselink_transform = Eigen::Vector3d(xyTrans_lcam_baselink[0], xyTrans_lcam_baselink[1], xyTrans_lcam_baselink[2]);

    // Load camera topics
    nh_.getParam("camera_subscribers/lCam_subscriber/topic", lCam_topic);
    nh_.getParam("camera_subscribers/rCam_subscriber/topic", rCam_topic);
    nh_.getParam("camera_subscribers/mCam_subscriber/topic", mCam_topic);

    // Initialize noise models
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << odometry_noise[0], odometry_noise[1], odometry_noise[2]).finished());
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << prior_noise[0], prior_noise[1], prior_noise[2]).finished());
    brNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << bearing_range_noise[0], bearing_range_noise[1]).finished());
    pointNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << point_noise[0], point_noise[1]).finished());

    // Total number of IDs
    int total_tags;
    nh_.getParam("total_tags", total_tags);
    // Predefined tags to search for in the environment.
    for (int j = 0; j < total_tags; ++j) {
        possibleIds_.push_back("tag_" + std::to_string(j));
    }
    
    ROS_INFO("Parameters loaded.");

    // Initialize GTSAM components
    initializeGTSAM();
    // Index to keep track of the sequential pose.
    index_of_pose = 1;
    // Initialize the factor graphs
    keyframeGraph_ = gtsam::NonlinearFactorGraph();
    windowGraph_ = gtsam::NonlinearFactorGraph();

    // Initialize camera subscribers
    mCam_subscriber = nh_.subscribe(mCam_topic, 1000, &aprilslamcpp::mCamCallback, this);
    rCam_subscriber = nh_.subscribe(rCam_topic, 1000, &aprilslamcpp::rCamCallback, this);
    lCam_subscriber = nh_.subscribe(lCam_topic, 1000, &aprilslamcpp::lCamCallback, this);

    // Subscriptions and Publications
    odom_sub_ = nh_.subscribe(odom_topic, 10, &aprilslamcpp::addOdomFactor, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1, true);
    landmark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("landmarks", 1, true);
    path.header.frame_id = frame_id; 
}

// Camera callback functions
void aprilslamcpp::mCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    mCam_msg = msg;
}

void aprilslamcpp::rCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    rCam_msg = msg;
}

void aprilslamcpp::lCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    lCam_msg = msg;
}

// Initialization of GTSAM components
void aprilslamcpp::initializeGTSAM() { 
    // Initialize graph parameters and stores them in isam_.
    gtsam::ISAM2Params parameters;
    isam_ = gtsam::ISAM2(parameters);
}

bool aprilslamcpp::shouldAddKeyframe(const gtsam::Pose2& lastPose, const gtsam::Pose2& currentPose) {
    // Calculate the distance between the current pose and the last keyframe pose
    double distance = lastPose.range(currentPose);

    // Calculate the difference in orientation (theta) between the current pose and the last keyframe pose
    double angleDifference = std::abs(wrapToPi(currentPose.theta() - lastPose.theta()));

    // Check if either the distance moved or the rotation exceeds the threshold
    if (distance > distanceThreshold || angleDifference > rotationThreshold) {
        return true;  // Add a new keyframe
    }
    return false;  // Do not add a keyframe
}

void aprilslamcpp::createNewKeyframe(const gtsam::Pose2& predictedPose, const gtsam::Pose2& previousPose, gtsam::Symbol& previousKeyframeSymbol) {
    gtsam::Symbol currentKeyframeSymbol('X', index_of_pose);

    // Maintain a persistent storage for historic landmarks
    static std::map<gtsam::Key, gtsam::Point2> historicLandmarks;

    // Compute and add the between factor between the current keyframe and the previous keyframe
    if (previousKeyframeSymbol) {
        gtsam::Pose2 relativePose = previousPose.between(predictedPose);
        keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
    }

    // Use ISAM2 to get the latest estimate of poses 'X' and landmarks 'L'
    auto result = isam_.calculateEstimate();

    // Update the keyframe estimates with the latest estimates from ISAM2 and preserve landmark history
    for (const auto& key_value : result) {
        gtsam::Key key = key_value.key;
        if (gtsam::Symbol(key).chr() == 'X') {  
            // Update pose estimates
            if (!keyframeEstimates_.exists(key)) {
                gtsam::Pose2 pose = result.at<gtsam::Pose2>(key);
                keyframeEstimates_.insert(key, pose);
            }
        } else if (gtsam::Symbol(key).chr() == 'L') {  
            // Update landmark estimates
            if (!keyframeEstimates_.exists(key)) {
                gtsam::Point2 landmark = result.at<gtsam::Point2>(key);
                keyframeEstimates_.insert(key, landmark);

                // Add the landmark as a prior in the keyframe graph if it's not already there
                if (!keyframeGraph_.exists(key)) {
                    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(key, landmark, pointNoise));
                }

                // Store the landmark in the persistent storage
                historicLandmarks[key] = landmark;
            }
        }
    }

    // Reinsert all historic landmarks into the keyframe graph
    for (const auto& key_value : historicLandmarks) {
        gtsam::Key key = key_value.first;
        gtsam::Point2 landmark = key_value.second;
        if (!keyframeEstimates_.exists(key)) {
            keyframeEstimates_.insert(key, landmark);
            if (!keyframeGraph_.exists(key)) {
                keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(key, landmark, pointNoise));
            }
        }
    }

    // Add br factors with X-L detection
    if (poseToLandmarkMeasurementsMap.find(currentKeyframeSymbol) != poseToLandmarkMeasurementsMap.end()) {
        for (const auto& landmarkMeasurement : poseToLandmarkMeasurementsMap[currentKeyframeSymbol]) {
            gtsam::Symbol landmarkSymbol = landmarkMeasurement.first;  // Get the landmark (L) symbol
            double bearing = std::get<0>(landmarkMeasurement.second);  // Extract bearing
            double range = std::get<1>(landmarkMeasurement.second);    // Extract range

            // Add the bearing-range factor between the current keyframe pose and the landmark
            if (!keyframeGraph_.exists(landmarkSymbol)) {  // Only add if not already in the graph
                keyframeGraph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>(
                    currentKeyframeSymbol, landmarkSymbol, 
                    gtsam::Rot2::fromAngle(bearing),
                    range, brNoise));

                // Insert the landmark into keyframe estimates if it doesn't already exist
                if (!keyframeEstimates_.exists(landmarkSymbol)) {
                    keyframeEstimates_.insert(landmarkSymbol, result.at<gtsam::Point2>(landmarkSymbol));
                }
            }
        }
    }

    // graphvisulisation(keyframeGraph_);

    // Clear the window graph and reset it with the new keyframe graph as its base
    windowGraph_.resize(0);  // Clear the current window graph
    windowEstimates_.clear();  // Clear current window estimates

    // Add all keyframe graph factors to the window graph to serve as the base, except X147-X148 factor
    for (size_t i = 0; i < keyframeGraph_.size(); ++i) {
        auto factor = keyframeGraph_.at(i);
        windowGraph_.add(factor);
    }

    // Insert updated estimates from keyframeEstimates_ into windowEstimates_
    windowEstimates_.insert(keyframeEstimates_);

    // isam_.update(keyframeGraph_, keyframeEstimates_);

    ROS_INFO("Window graph and estimates reset with keyframe graph.");
}

void aprilslamcpp::pruneOldFactorsByTime(double current_time, double timewindow) {
    // Define a threshold for old factors and variables
    double time_threshold = current_time - timewindow;
    // Identify factors to remove
    gtsam::FastList<size_t> factors_to_remove;
    for (const auto& factor_time : factorTimestamps_) {
        if (factor_time.second < time_threshold) {
            factors_to_remove.push_back(factor_time.first);
        }
    }

    // Remove old factors
    if (!factors_to_remove.empty()) {
        for (const auto& factor_index : factors_to_remove) {
            keyframeGraph_.remove(factor_index);
            factorTimestamps_.erase(factor_index);
        }
    }
}

void aprilslamcpp::pruneOldFactorsBySize(double maxfactors) {
    // Prune factors if the total number of factors exceeds maxFactors_
    while (factorTimestamps_.size() > maxfactors) {
        auto oldest = factorTimestamps_.begin();
        keyframeGraph_.remove(oldest->first);
        factorTimestamps_.erase(oldest);
    }
}

gtsam::Pose2 aprilslamcpp::translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    tf2::Quaternion tfQuat(qx, qy, qz, qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
    return gtsam::Pose2(x, y, yaw);
}

void aprilslamcpp::graphvisulisation(gtsam::NonlinearFactorGraph& Graph_) {
    for (size_t i = 0; i < Graph_.size(); ++i) {
        // Use the correct shared pointer type for NonlinearFactorGraph
        gtsam::NonlinearFactor::shared_ptr factor = Graph_[i];
        
        // Demangle the factor type using abi::__cxa_demangle
        std::string factorType = typeid(*factor).name();
        int status = 0;
        char* demangledName = abi::__cxa_demangle(factorType.c_str(), nullptr, nullptr, &status);
        
        // If demangling was successful, use the demangled nameLandmarkPlotter
        std::string readableType = (status == 0) ? demangledName : factorType;
        std::string simplifiedType = readableType.substr(readableType.find_last_of(':') + 1); // Simplify type name

        // Free the memory allocated by abi::__cxa_demangle
        if (demangledName) {
            free(demangledName);
        }

        // Print the factor type and keys involved in a simplified format
        std::ostringstream oss;
        // oss << "  Factor " << i << " (" << simplifiedType << "):";
        oss << "  Factor " << i;

        for (const gtsam::Key& key : factor->keys()) {
            oss << " " << gtsam::DefaultKeyFormatter(key);
        }

        ROS_INFO("%s", oss.str().c_str());
    }
}

void aprilslamcpp::ISAM2Optimise() {    
    if (batchOptimisation_) {
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(windowGraph_, windowEstimates_);
        windowEstimates_ = batchOptimizer.optimize();
        batchOptimisation_ = false; // Only do this once
    }

    // Update the iSAM2 instance with the new measurements
    isam_.update(windowGraph_, windowEstimates_);

    // Calculate the current best estimate
    auto result = isam_.calculateEstimate();

    // Extract landmark estimates from result
    std::map<int, gtsam::Point2> landmarks;
    for (const auto& key_value : result) {
        gtsam::Key key = key_value.key;  // Get the key
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point2 point = result.at<gtsam::Point2>(key); // Directly access the Point2 value
            landmarks[gtsam::Symbol(key).index()] = point;
        }
    }
    // Publish the pose
    aprilslam::publishLandmarks(landmark_pub_, landmarks, frame_id);
    aprilslam::publishPath(path_pub_, result, index_of_pose, frame_id);

    // Update keyframe estimates with the results from the optimized window graph
    // updateKeyframeGraphWithOptimizedResults(result);
    // ROS_INFO("keyframe update done");

    // Save the landmarks into a csv file 
    if (savetaglocation) {
        saveLandmarksToCSV(landmarks, pathtosavelandmarkcsv);
    }
    // Prune the graph to maintain a predefined time window
    double current_time = ros::Time::now().toSec();
    if (useprunebytime) {
        pruneOldFactorsByTime(current_time, timeWindow);
    }
    else if (useprunebysize) {
        pruneOldFactorsBySize(maxfactors);
    }
     else {
        // Do nothing if no pruning is required
    }
    // Clear estimates for the next iteration (????necessary)
    windowEstimates_.clear();
    // landmarkEstimates.clear();
}

void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    double current_time = ros::Time::now().toSec();
    ros::WallTime start_loop, end_loop; // Declare variables to hold start and end times
    double elapsed;

    // Convert the incoming odometry message to a simpler (x, y, theta) format using a previously defined method
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);

    // Calculate the distance and rotation change from the last pose
    double position_change = std::hypot(poseSE2.x() - lastPoseSE2_.x(), poseSE2.y() - lastPoseSE2_.y());
    double rotation_change = std::abs(wrapToPi(poseSE2.theta() - lastPoseSE2_.theta()));

    // Check if the movement exceeds the thresholds
    if (position_change < stationary_position_threshold && rotation_change < stationary_rotation_threshold) {
        // Robot is stationary; skip factor graph update
        return;
    }

    index_of_pose += 1; // Increment the pose index for each new odometry message

    // Store the initial pose for relative calculations
    if (index_of_pose == 2) {
        lastPoseSE2_ = poseSE2;
        gtsam::Pose2 pose0(0.0, 0.0, 0.0); // Prior at origin
        windowGraph_.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('X', 1), pose0, priorNoise));
        factorTimestamps_[windowGraph_.size() - 1] = current_time;
        windowEstimates_.insert(gtsam::Symbol('X', 1), pose0);
        lastPose_ = pose0; // Keep track of the last pose for odolandmarkKeymetry calculation
        // Load calibrated landmarks as priors if available
        if (usepriortagtable) {
            std::map<int, gtsam::Point2> savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);
            for (const auto& landmark : savedLandmarks) {
                gtsam::Symbol landmarkKey('L', landmark.first);
                windowGraph_.add(gtsam::PriorFactor<gtsam::Point2>(landmarkKey, landmark.second, pointNoise));
                windowEstimates_.insert(landmarkKey, landmark.second);
                landmarkEstimates.insert(landmarkKey, landmark.second);
            }
        }
        Key_previous_pos = pose0;
        keyframeGraph_ = windowGraph_;
        keyframeEstimates_ = windowEstimates_;
        previousKeyframeSymbol = gtsam::Symbol('X', 1);
    }

    // Predict the next pose based on odometry and add it as an initial estimate
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
    gtsam::Pose2 predictedPose = lastPose_.compose(odometry);

    // Determine if this pose should be a keyframe
    windowEstimates_.insert(gtsam::Symbol('X', index_of_pose), predictedPose);

    // Add this relative motion as an odometry factor to the graph
    windowGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose - 1), gtsam::Symbol('X', index_of_pose), odometry, odometryNoise));

    factorTimestamps_[windowGraph_.size() - 1] = current_time;
    
    // Update the last pose and initial estimates for the next iteration
    lastPose_ = predictedPose;
    landmarkEstimates.insert(gtsam::Symbol('X', index_of_pose), predictedPose);

    // Loop closure detection setup
    std::set<gtsam::Symbol> detectedLandmarks;

    // Iterate through all landmark IDs if detected
    start_loop = ros::WallTime::now();
    if (mCam_msg && rCam_msg && lCam_msg) {  // Ensure the messages have been received
        auto detections = getCamDetections(mCam_msg, rCam_msg, lCam_msg, mcam_baselink_transform, rcam_baselink_transform, lcam_baselink_transform);
        // Access the elements of the std::pair
        const std::vector<int>& Id = detections.first;
        const std::vector<Eigen::Vector2d>& tagPos = detections.second;
        if (!Id.empty()) {
            for (size_t n = 0; n < Id.size(); ++n) {
                int tag_number = Id[n];        
                Eigen::Vector2d landSE2 = tagPos[n];

                // Compute prior location of the landmark using the current robot pose
                double theta = lastPose_.theta();
                Eigen::Rotation2Dd rotation(theta);  // Create a 2D rotation matrix
                Eigen::Vector2d rotatedPosition = rotation * landSE2;  // Rotate the position into the robot's frame
                gtsam::Point2 priorLand(rotatedPosition.x() + lastPose_.x(), rotatedPosition.y() + lastPose_.y());

                // Compute bearing and range
                double bearing = std::atan2(landSE2(1), landSE2(0));
                double range = std::sqrt(landSE2(0) * landSE2(0) + landSE2(1) * landSE2(1));

                // Construct the landmark key
                gtsam::Symbol landmarkKey('L', tag_number);  

                // Store the bearing and range measurements in the map
                poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = std::make_tuple(bearing, range);

                // Check if the landmark has been observed before
                if (tagToNodeIDMap_.find(tag_number) != tagToNodeIDMap_.end()) {
                    // Existing landmark
                    gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                        gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                    );
                    gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                    // Threshold for ||projection - measurement||
                    if (fabs(error[0]) < add2graph_threshold) windowGraph_.add(factor);
                    factorTimestamps_[windowGraph_.size() - 1] = current_time;
                    detectedLandmarks.insert(landmarkKey);
                } 
                else {
                    // If the current landmark was not detected in the calibration run 
                    // Or it's on calibration mode
                    if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                        // New landmark detected
                        tagToNodeIDMap_[tag_number] = landmarkKey;
                        windowEstimates_.insert(landmarkKey, priorLand); // Simple initial estimate
                        landmarkEstimates.insert(landmarkKey, priorLand);
                        // Add a prior for the landmark position to help with initial estimation.
                        windowGraph_.add(gtsam::PriorFactor<gtsam::Point2>(
                            landmarkKey, priorLand, pointNoise)
                        );
                    }
                    factorTimestamps_[windowGraph_.size() - 1] = current_time;
                    // Add a bearing-range observation for this landmark to the graph
                    gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                        gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                    );
                    windowGraph_.add(factor);
                    factorTimestamps_[windowGraph_.size() - 1] = current_time;
                    detectedLandmarks.insert(landmarkKey);
                }
                // Store the bearing and range measurements in the map
                poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = std::make_tuple(bearing, range);                
            }
        }
    }

    // Update the pose to landmarks mapping (for recording LC condition)
    poseToLandmarks[gtsam::Symbol('X', index_of_pose)] = detectedLandmarks;
     
    // Loop closure check
    // checkLoopClosure(current_time, detectedLandmarks);

    end_loop = ros::WallTime::now();
    elapsed = (end_loop - start_loop).toSec();
    // ROS_INFO("transform total: %f seconds", elapsed);
    lastPoseSE2_ = poseSE2;
    start_loop = ros::WallTime::now();
    // ISAM2 optimization to update the map and robot pose estimates
    if (index_of_pose % 1 == 0) {
        ISAM2Optimise();
        end_loop = ros::WallTime::now();
        elapsed = (end_loop - start_loop).toSec();
        ROS_INFO("optimisation: %f seconds", elapsed);
    }

    // keygraph build
    if (shouldAddKeyframe(Key_previous_pos, predictedPose)) {
    createNewKeyframe(predictedPose, Key_previous_pos, previousKeyframeSymbol);
    ROS_INFO("keyframe added");
    Key_previous_pos = predictedPose;
    previousKeyframeSymbol = gtsam::Symbol('X', index_of_pose);
    // Re-Initialize the factor graph
    // initializeGTSAM();
    }
}
}

int main(int argc, char **argv) {
    // Initialize the ROS system and specify the name of the node
    ros::init(argc, argv, "april_slam_cpp");

    // Create a handle to this process' node
    ros::NodeHandle nh;

    // Setting buffer cache time
    double transformation_search_range;
    nh.getParam("transformation_search_range", transformation_search_range);
    ros::Duration tsr(transformation_search_range); 

    // Create an instance of the aprilslamcpp class, passing in the node handle
    aprilslam::aprilslamcpp slamNode(nh, tsr);

    // ROS enters a loop, pumping callbacks. Internally, it will call all the callbacks waiting to be called at that point in time.
    ros::spin();

    return 0;
}