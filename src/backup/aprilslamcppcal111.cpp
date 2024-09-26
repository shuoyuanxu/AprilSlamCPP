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

bool aprilslamcpp::shouldAddKeyframe(
    const gtsam::Pose2& lastPose, 
    const gtsam::Pose2& currentPose, 
    std::set<gtsam::Symbol> oldlandmarks, 
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos) {
    // Calculate the distance between the current pose and the last keyframe pose
    double distance = lastPose.range(currentPose);
    // Iterate over detectedLandmarksCurrentPos, add key if new tag is detected
    for (const auto& landmark : detectedLandmarksCurrentPos) {
        // If the landmark is not found in oldLandmarks, return true
        if (oldlandmarks.find(landmark) == oldlandmarks.end()) {
            return true;
        }
    }
    // Calculate the difference in orientation (theta) between the current pose and the last keyframe pose
    double angleDifference = std::abs(wrapToPi(currentPose.theta() - lastPose.theta()));

    // Check if either the distance moved or the rotation exceeds the threshold
    if (distance > distanceThreshold || angleDifference > rotationThreshold) {
        return true;  // Add a new keyframe
    }
    return false;  // Do not add a keyframe
}

void aprilslamcpp::marginalizenonKeyframes(
    gtsam::ISAM2& isam, 
    const std::set<gtsam::Symbol>& nonKeyframesToMarginalize, 
    const gtsam::NonlinearFactorGraph& originalGraph) {

    // Step 1: Get the current factor graph and values from iSAM2
    const gtsam::Values& originalValues = isam.calculateEstimate();

    // Step 2: Compute the marginal covariance using the Marginals class
    gtsam::Marginals marginals(originalGraph, originalValues);
    
    gtsam::NonlinearFactorGraph newGraph;
    gtsam::Values newValues;

    for (const auto& keyframe : nonKeyframesToMarginalize) {
        try {
            // Compute the marginal information matrix for the variable to be marginalized
            gtsam::Matrix marginalCov = marginals.marginalCovariance(keyframe);
            
            // Convert the marginal covariance into a factor (dense factor)
            // NOTE: This step depends on how you want to approximate the dense factor.
            // gtsam does not provide a direct "marginal factor" creation API,
            // you can add a factor that relates the remaining variables based on the covariance.
            
            // Add the new factor to the graph (this is an example; modify to suit your problem)
            // newGraph.add(...);

        } catch (const std::exception& e) {
            std::cerr << "Error while computing marginal for keyframe " << keyframe << ": " << e.what() << std::endl;
            continue;
        }
    }

    // Step 3: Remove the factors involving non-keyframes from the original graph
    for (const auto& factor : originalGraph) {
        bool shouldKeepFactor = true;

        // Check if the factor involves any of the non-keyframe poses to be marginalized
        for (const gtsam::Key& key : factor->keys()) {
            gtsam::Symbol symbol(key);
            if (nonKeyframesToMarginalize.find(symbol) != nonKeyframesToMarginalize.end()) {
                shouldKeepFactor = false;  // Mark the factor for removal
                break;
            }
        }

        // Add factors that don't involve marginalized variables
        if (shouldKeepFactor) {
            newGraph.add(factor);
        }
    }

    // Step 4: Add remaining keyframes and landmarks to the new graph and estimates
    for (const auto& key_value : originalValues) {
        gtsam::Symbol symbol(key_value.key);
        if (nonKeyframesToMarginalize.find(symbol) == nonKeyframesToMarginalize.end()) {
            newValues.insert(symbol, originalValues.at(symbol));  // Add poses and landmarks not marginalized
        }
    }

    // Step 5: Update iSAM2 with the new graph and values
    isam.update(newGraph, newValues);

    // Optionally, log the marginalized non-keyframes
    for (const auto& keyframe : nonKeyframesToMarginalize) {
        std::cout << "Marginalized non-keyframe " << keyframe << std::endl;
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

    // Save the landmarks into a csv file 
    if (savetaglocation) {
        saveLandmarksToCSV(landmarks, pathtosavelandmarkcsv);
    }
    // Clear estimates for the next iteration (????necessary)
    windowEstimates_.clear();
}

std::set<gtsam::Symbol> aprilslamcpp::getPoseKeysBetweenKeyframes(const gtsam::Symbol& previousKeyframeSymbol, const gtsam::Symbol& currentKeyframeSymbol) {
    std::set<gtsam::Symbol> poseKeys;

    // Extract the indices from the keyframe symbols
    int previousIndex = previousKeyframeSymbol.index();
    int currentIndex = currentKeyframeSymbol.index();

    // Ensure the previous index is smaller than the current index
    if (previousIndex >= currentIndex) {
        ROS_WARN("Previous keyframe index is greater than or equal to the current keyframe index.");
        return poseKeys;
    }

    // Iterate through the indices between previous and current keyframes
    for (int i = previousIndex + 1; i < currentIndex; ++i) {
        gtsam::Symbol poseKey('X', i);  // Create a pose key 'X' with index i
        poseKeys.insert(poseKey);  // Add it to the set
    }

    return poseKeys;
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
        keyframePosIds.insert(gtsam::Symbol('X', 1));  // Add fist keyframeID to the array
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
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos;
    std::set<gtsam::Symbol> oldlandmarks;
    oldlandmarks = detectedLandmarksHistoric; 
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
                if (detectedLandmarksHistoric.find(landmarkKey) != detectedLandmarksHistoric.end()) {
                    // Existing landmark
                    gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                        gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                    );
                    gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                    // Threshold for ||projection - measurement||
                    if (fabs(error[0]) < add2graph_threshold) windowGraph_.add(factor);
                    factorTimestamps_[windowGraph_.size() - 1] = current_time;
                    detectedLandmarksCurrentPos.insert(landmarkKey);
                } 
                else {
                    // If the current landmark was not detected in the calibration run 
                    // Or it's on calibration mode
                    if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                    // New landmark detected
                    detectedLandmarksHistoric.insert(landmarkKey);
                    // Check if the key already exists in windowEstimates_ before inserting
                    if (windowEstimates_.exists(landmarkKey)) {
                    } else {
                        windowEstimates_.insert(landmarkKey, priorLand); // Simple initial estimate
                    }

                    // Check if the key already exists in landmarkEstimates before inserting
                    if (landmarkEstimates.exists(landmarkKey)) {
                    } else {
                        landmarkEstimates.insert(landmarkKey, priorLand);
                    }

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
                    detectedLandmarksCurrentPos.insert(landmarkKey);
                }
                // Store the bearing and range measurements in the map
                poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = std::make_tuple(bearing, range);                
            }
        }
    }

    // Update the pose to landmarks mapping (for recording LC condition)
    poseToLandmarks[gtsam::Symbol('X', index_of_pose)] = detectedLandmarksCurrentPos;
     
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
    if (shouldAddKeyframe(Key_previous_pos, predictedPose, oldlandmarks, detectedLandmarksCurrentPos)) {
    std::set<gtsam::Symbol> keystobemarginalised = getPoseKeysBetweenKeyframes(previousKeyframeSymbol, gtsam::Symbol('X', index_of_pose));
    marginalizenonKeyframes(isam_, keystobemarginalised, windowGraph_);
    Key_previous_pos = predictedPose;
    previousKeyframeSymbol = gtsam::Symbol('X', index_of_pose);
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