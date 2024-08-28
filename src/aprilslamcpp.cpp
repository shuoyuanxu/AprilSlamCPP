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

    // Initialize GTSAM components
    initializeGTSAM();

    // Subscriptions and Publications
    odom_sub_ = nh_.subscribe(odom_topic, 10, &aprilslamcpp::addOdomFactor, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1, true);
    landmark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("landmarks", 1, true);
    path.header.frame_id = frame_id; 
}

//Intilialisation
void aprilslamcpp::initializeGTSAM() { 
    // Index to keep track of the sequential pose.
    index_of_pose = 1;

    // Initialize the factor graphs
    keyframeGraph_ = gtsam::NonlinearFactorGraph();
    windowGraph_ = gtsam::NonlinearFactorGraph();

    // Initialize ISAM2 with parameters.
    gtsam::ISAM2Params parameters;
    isam_ = gtsam::ISAM2(parameters);

    ROS_INFO_STREAM("Initilisation Done");
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

void aprilslamcpp::createNewKeyframe(const gtsam::Pose2& predictedPose, gtsam::Symbol& previousKeyframeSymbol) {
    gtsam::Symbol currentKeyframeSymbol('X', index_of_pose);
    
    // Log the start of keyframe creation
    ROS_INFO("Creating new keyframe with symbol: %s", gtsam::DefaultKeyFormatter(currentKeyframeSymbol).c_str());

    // Add the keyframe pose to the keyframe graph
    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Pose2>(currentKeyframeSymbol, predictedPose, priorNoise));
    keyframeEstimates_.insert(currentKeyframeSymbol, predictedPose);
    ROS_INFO("Keyframe pose inserted into keyframe graph and estimates.");

    // Compute and add the between factor between the current keyframe and the previous keyframe
    if (index_of_pose > 1) {  // Ensure there is a previous keyframe
        ROS_INFO("Computing and adding between factor between %s and %s.",
                 gtsam::DefaultKeyFormatter(previousKeyframeSymbol).c_str(),
                 gtsam::DefaultKeyFormatter(currentKeyframeSymbol).c_str());
        
        gtsam::Pose2 previousPose = keyframeEstimates_.at<gtsam::Pose2>(previousKeyframeSymbol);
        gtsam::Pose2 relativePose = previousPose.between(predictedPose);
        keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
        ROS_INFO("Between factor between %s and %s added to keyframe graph.",
                 gtsam::DefaultKeyFormatter(previousKeyframeSymbol).c_str(),
                 gtsam::DefaultKeyFormatter(currentKeyframeSymbol).c_str());
    }

    // Copy any existing landmark priors from the window graph to the keyframe graph
    for (const auto& key_value : windowEstimates_) {
        gtsam::Key key = key_value.key;

        // Only copy if it's a landmark (denoted by 'L') and it hasn't been added to keyframeEstimates_ yet
        if (gtsam::Symbol(key).chr() == 'L' && !keyframeEstimates_.exists(key)) {
            gtsam::Point2 landmarkPosition = windowEstimates_.at<gtsam::Point2>(key);
            keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(key, landmarkPosition, pointNoise));
            keyframeEstimates_.insert(key, landmarkPosition);
            ROS_INFO("Copied landmark prior %s to keyframe graph.", gtsam::DefaultKeyFormatter(key).c_str());
        }
    }

    // Add associated landmarks to the keyframe graph
    if (poseToLandmarks.find(currentKeyframeSymbol) != poseToLandmarks.end()) {
        ROS_INFO("Adding associated landmarks for keyframe %s.", gtsam::DefaultKeyFormatter(currentKeyframeSymbol).c_str());
        for (const auto& landmarkSymbol : poseToLandmarks[currentKeyframeSymbol]) {
            ROS_INFO("Processing landmark with symbol: %s", gtsam::DefaultKeyFormatter(landmarkSymbol).c_str());
            
            if (!keyframeEstimates_.exists(landmarkSymbol)) {
                keyframeEstimates_.insert(landmarkSymbol, windowEstimates_.at<gtsam::Point2>(landmarkSymbol));
                ROS_INFO("Inserted landmark %s into keyframe estimates.", gtsam::DefaultKeyFormatter(landmarkSymbol).c_str());
            }
            
            keyframeGraph_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>(
                currentKeyframeSymbol, landmarkSymbol, 
                gtsam::Rot2::fromAngle(atan2(
                    windowEstimates_.at<gtsam::Point2>(landmarkSymbol).y() - predictedPose.y(),
                    windowEstimates_.at<gtsam::Point2>(landmarkSymbol).x() - predictedPose.x()
                )),
                predictedPose.range(windowEstimates_.at<gtsam::Point2>(landmarkSymbol)), brNoise));
            
            ROS_INFO("Added bearing-range factor between keyframe %s and landmark %s to keyframe graph.",
                     gtsam::DefaultKeyFormatter(currentKeyframeSymbol).c_str(),
                     gtsam::DefaultKeyFormatter(landmarkSymbol).c_str());
        }
    } else {
        ROS_WARN("No landmarks found for keyframe %s.", gtsam::DefaultKeyFormatter(currentKeyframeSymbol).c_str());
    }

    // Clear the window graph and reset it with the new keyframe graph as its base
    ROS_INFO("Clearing window graph and estimates, and resetting with keyframe graph as base.");
    windowGraph_.resize(0); // Clear the current window graph
    windowEstimates_.clear(); // Clear current window estimates

    // Add all keyframe graph factors to the window graph to serve as the base, except X147-X148 factor
    for (size_t i = 0; i < keyframeGraph_.size(); ++i) {
        auto factor = keyframeGraph_.at(i);
        if (auto betweenFactor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose2>>(factor)) {
            gtsam::Symbol symbol1(betweenFactor->keys()[0]);
            gtsam::Symbol symbol2(betweenFactor->keys()[1]);
            if (!(symbol1 == gtsam::Symbol('X', 147) && symbol2 == currentKeyframeSymbol)) {
                windowGraph_.add(factor);
            }
        } else {
            windowGraph_.add(factor);
        }
    }
    windowEstimates_.insert(keyframeEstimates_); // Add all estimates from the keyframe graph
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

void aprilslamcpp::updateKeyframeGraphWithOptimizedResults(const gtsam::Values& optimizedResults) {
    ROS_INFO("Entered updateKeyframeGraphWithOptimizedResults");

    // Update keyframe estimates with the optimized values
    for (const auto& keyframe : keyframeEstimates_) {
        gtsam::Key key = keyframe.key;
        ROS_INFO("Processing key: %s", gtsam::DefaultKeyFormatter(key).c_str());
        if (optimizedResults.exists(key)) {
            try {
                keyframeEstimates_.update(key, optimizedResults.at<gtsam::Pose2>(key));
                ROS_INFO("Updated keyframe estimate for key: %s", gtsam::DefaultKeyFormatter(key).c_str());
            } catch (const std::exception& e) {
                ROS_ERROR("Exception while updating keyframe estimate: %s", e.what());
            }
        } else {
            ROS_WARN("Key %s does not exist in optimized results", gtsam::DefaultKeyFormatter(key).c_str());
        }
    }
    ROS_INFO("Update keyframe estimates with the optimized values");
    // Update any associated landmarks in the keyframe graph
    for (const auto& key_value : optimizedResults) {
        gtsam::Key key = key_value.key;
        ROS_INFO("Processing key for landmarks: %s", gtsam::DefaultKeyFormatter(key).c_str());
        if (gtsam::Symbol(key).chr() == 'L' && keyframeEstimates_.exists(key)) {
            try {
                keyframeEstimates_.update(key, optimizedResults.at<gtsam::Point2>(key));
                ROS_INFO("Updated landmark estimate for key: %s", gtsam::DefaultKeyFormatter(key).c_str());
            } catch (const std::exception& e) {
                ROS_ERROR("Exception while updating landmark estimate: %s", e.what());
            }
        }
    }

    ROS_INFO("Completed updateKeyframeGraphWithOptimizedResults");
}

void aprilslamcpp::graphvisulisation(gtsam::NonlinearFactorGraph& Graph_) {
    for (size_t i = 0; i < Graph_.size(); ++i) {
        // Use the correct shared pointer type for NonlinearFactorGraph
        gtsam::NonlinearFactor::shared_ptr factor = Graph_[i];
        
        // Demangle the factor type using abi::__cxa_demangle
        std::string factorType = typeid(*factor).name();
        int status = 0;
        char* demangledName = abi::__cxa_demangle(factorType.c_str(), nullptr, nullptr, &status);
        
        // If demangling was successful, use the demangled name
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
    // Debug message: print the factor graph structure before optimization
    ROS_INFO("Factor graph structure before optimization:");
    
    // Graph visulisation
    graphvisulisation(windowGraph_);

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
    ROS_INFO("optimisation done");

    // Update keyframe estimates with the results from the optimized window graph
    // updateKeyframeGraphWithOptimizedResults(result);
    ROS_INFO("keyframe update done");

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

// void aprilslamcpp::checkLoopClosure(double current_time, const std::set<gtsam::Symbol>& detectedLandmarks) {
//     if (loopClosureEnableFlag) {
//         double spatialThreshold = 2.0; // meters
//         double temporalThreshold = 30.0; // seconds
//         int requiredReobservedLandmarks = 3; // Minimum number of re-detected landmarks to trigger loop closure

//         for (const auto& keyframe : keyframes) {
//             gtsam::Symbol keyframeSymbol = keyframe.first;
//             gtsam::Pose2 keyframePose = keyframe.second;
//             double keyframeTime = keyframeTimestamps[keyframeSymbol];

//             double distance = lastPose_.range(keyframePose);
//             double timeDiff = current_time - keyframeTime;

//             if (distance < spatialThreshold && timeDiff > temporalThreshold) {
//                 // Count the number of re-detected landmarks in this keyframe
//                 std::set<gtsam::Symbol> intersection;
//                 std::set_intersection(detectedLandmarks.begin(), detectedLandmarks.end(),
//                                       poseToLandmarks[keyframeSymbol].begin(), poseToLandmarks[keyframeSymbol].end(),
//                                       std::inserter(intersection, intersection.begin()));

//                 int reobservedLandmarks = intersection.size();
//                 if (reobservedLandmarks >= requiredReobservedLandmarks) {
//                     // Add loop closure constraint
//                     graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(keyframeSymbol, gtsam::Symbol('X', index_of_pose), relPoseFG(keyframePose, lastPoseSE2_), odometryNoise));
//                     factorTimestamps_[graph_.size() - 1] = current_time;
//                     break;
//                 }
//             }
//         }
//     }
// }

void aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    double current_time = ros::Time::now().toSec();
    ros::WallTime start_loop, end_loop; // Declare variables to hold start and end times=
    double elapsed;
    index_of_pose += 1; // Increment the pose index for each new odometry message

    // Convert the incoming odometry message to a simpler (x, y, theta) format using a previously defined method
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);

    // Store the initial pose for relative calculations
    if (index_of_pose == 2) {
        lastPoseSE2_ = poseSE2;
        gtsam::Pose2 pose0(0.0, 0.0, 0.0); // Prior at origin
        windowGraph_.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('X', 1), pose0, priorNoise));
        factorTimestamps_[windowGraph_.size() - 1] = current_time;
        windowEstimates_.insert(gtsam::Symbol('X', 1), pose0);
        lastPose_ = pose0; // Keep track of the last pose for odometry calculation
            // Load calibrated landmarks as priors if avaliable
        if (usepriortagtable) {
            std::map<int, gtsam::Point2> savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);
            for (const auto& landmark : savedLandmarks) {
                gtsam::Symbol landmarkKey('L', landmark.first);
                windowGraph_.add(gtsam::PriorFactor<gtsam::Point2>(landmarkKey, landmark.second, pointNoise));
                windowEstimates_.insert(landmarkKey, landmark.second);
                landmarkEstimates.insert(landmarkKey, landmark.second);
                }
        }

        keyframeGraph_ = windowGraph_;
        keyframeEstimates_ = windowEstimates_;
        previousKeyframeSymbol = gtsam::Symbol('X', 1);
    }
    ROS_INFO("Initial window graph:");
    graphvisulisation(windowGraph_);
    ROS_INFO("Initial key graph:");
    graphvisulisation(keyframeGraph_);

    // Predict the next pose based on odometry and add it as an initial estimate
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
    gtsam::Pose2 predictedPose = lastPose_.compose(odometry);

    // Determine if this pose should be a keyframe
    if (shouldAddKeyframe(Key_previous_pos, predictedPose)) {
        createNewKeyframe(predictedPose, previousKeyframeSymbol);
        ROS_INFO("keyframe added");
        Key_previous_pos = poseSE2;
        previousKeyframeSymbol = gtsam::Symbol('X', index_of_pose);
    }
    else{
        windowEstimates_.insert(gtsam::Symbol('X', index_of_pose), poseSE2);
    }

    // Add this relative motion as an odometry factor to the graph
    windowGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose - 1), gtsam::Symbol('X', index_of_pose), odometry, odometryNoise));
    ROS_INFO("factor added");

    factorTimestamps_[windowGraph_.size() - 1] = current_time;
    
    // Update the last pose and initial estimates for the next iteration
    lastPose_ = predictedPose;
    landmarkEstimates.insert(gtsam::Symbol('X', index_of_pose), poseSE2);
    ROS_INFO("estimated added");

    // Loop closure detection setup
    std::set<gtsam::Symbol> detectedLandmarks;

    // Iterate through all landmark IDs to if detected
    start_loop = ros::WallTime::now();
    for (const auto& tag_id : possibleIds_) {    
        try {
            // Find transformation between vehicle and landmarks (see if landmarks are detected)
            geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform(robot_frame, tag_id, ros::Time(0));

            // Extract the transform details
            double trans_x = transformStamped.transform.translation.x;
            double trans_y = transformStamped.transform.translation.y;
            double theta = lastPose_.theta();
            tf2::Matrix3x3 R;
            R.setEulerYPR(theta, 0, 0);  // yaw (theta), pitch, roll
            tf2::Vector3 trans(trans_x, trans_y, 0.0);
            tf2::Vector3 rotP = R * trans;
            gtsam::Point2 priorLand(rotP.x() + lastPose_.x(), rotP.y() + lastPose_.y());

            // Convert to bearing and range
            double range = sqrt(pow(trans_x, 2) + pow(trans_y, 2));
            double bearing = atan2(trans_y, trans_x);
            
            // Split the tag_id to get the tag number. Assume tag IDs are in the format "tag_X"
            auto underscorePos = tag_id.find('_');
            if (underscorePos == std::string::npos) continue; // Skip if the format is unexpected
            int tag_number = std::stoi(tag_id.substr(underscorePos + 1)) +1;
    
            // Construct the landmark key
            gtsam::Symbol landmarkKey('L', tag_number);  
            
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
                    // ROS_INFO("Condition1 (!initial_estimates_.exists(landmarkL21Key)): %s", !windowEstimates_.exists(landmarkKey) ? "true" : "false");
                    // ROS_INFO("Condition2 (usepriortagtable): %s", usepriortagtable ? "true" : "false");
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
        }
        catch (tf2::TransformException &ex) {
                continue;
        }
    } 
    // Update the pose to landmarks mapping (for recording LC condition)
    poseToLandmarks[gtsam::Symbol('X', index_of_pose)] = detectedLandmarks;
     
    // Loop closure check
    // checkLoopClosure(current_time, detectedLandmarks);

    end_loop = ros::WallTime::now();
    elapsed = (end_loop - start_loop).toSec();
    ROS_INFO("transform total: %f seconds", elapsed);
    lastPoseSE2_ = poseSE2;
    start_loop = ros::WallTime::now();
    // ISAM2 optimization to update the map and robot pose estimates
    if (index_of_pose % 1 == 0) {
        ISAM2Optimise();
    }
    end_loop = ros::WallTime::now();
    elapsed = (end_loop - start_loop).toSec();
    ROS_INFO("optimisation: %f seconds", elapsed);
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