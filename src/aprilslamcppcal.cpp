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
    return gtsam::Pose2(dx_body, dy_body, dtheta);
}

// Constructor
aprilslamcpp::aprilslamcpp(ros::NodeHandle node_handle)
    : nh_(node_handle), tf_listener_(tf_buffer_), mCam_data_received_(false), rCam_data_received_(false), lCam_data_received_(false) { 
    
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

    // Read error threshold for a landmark to be added to the graph
    nh_.getParam("add2graph_threshold", add2graph_threshold);    

    // Stationary conditions
    nh_.getParam("stationary_position_threshold", stationary_position_threshold);
    nh_.getParam("stationary_rotation_threshold", stationary_rotation_threshold);

    // Read calibration and localization settings
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
    
    // Predefined tags to search for in the environment
    for (int j = 0; j < total_tags; ++j) {
        possibleIds_.push_back("tag_" + std::to_string(j));
    }

    // Bag stop flag
    double inactivity_threshold;
    nh_.getParam("inactivity_threshold", inactivity_threshold);

    // Initialize GTSAM components
    initializeGTSAM();
    // Index to keep track of the sequential pose.
    index_of_pose = 1;
    // Initialize the factor graphs
    keyframeGraph_ = gtsam::NonlinearFactorGraph();
    
    // Initialize camera subscribers
    mCam_subscriber = nh_.subscribe(mCam_topic, 1000, &aprilslamcpp::mCamCallback, this);
    rCam_subscriber = nh_.subscribe(rCam_topic, 1000, &aprilslamcpp::rCamCallback, this);
    lCam_subscriber = nh_.subscribe(lCam_topic, 1000, &aprilslamcpp::lCamCallback, this);

    // Subscriptions and Publications
    odom_sub_ = nh_.subscribe(odom_topic, 10, &aprilslamcpp::addOdomFactor, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1, true);
    landmark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("landmarks", 1, true);
    path.header.frame_id = frame_id; 

    // Timer to periodically check if valid data has been received by any camera
    check_data_timer_ = nh_.createTimer(ros::Duration(2.0), [this, inactivity_threshold](const ros::TimerEvent&) {
        if (mCam_data_received_ || rCam_data_received_ || lCam_data_received_) {
            accumulated_time_ = 0.0;  // Reset accumulated time on valid data
            mCam_data_received_ = false;  // Reset flags to check for new data next time
            rCam_data_received_ = false;
            lCam_data_received_ = false;
        } else {
            accumulated_time_ += 2.0;  // Accumulate time when no valid data is received
            ROS_WARN("No new valid data received from any camera. Accumulated time: %.1f seconds", accumulated_time_);

            // If no data received for inactivity_threshold from any camera, shut down+
            if (accumulated_time_ >= inactivity_threshold) {
                ROS_ERROR("No valid data from mCam, rCam, or lCam for 15 seconds. Shutting down.");
                this->~aprilslamcpp();  // Trigger the destructor
            }
        }
    });
}

// Destructor implementation
aprilslamcpp::~aprilslamcpp() {
    ROS_INFO("Node is shutting down. Executing SAMOptimise().");
    SAMOptimise();
        // Extract landmark estimates from the result
    std::map<int, gtsam::Point2> landmarks;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;  // Get the key
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point2 point = keyframeEstimates_.at<gtsam::Point2>(key);  // Access the Point2 value
            landmarks[gtsam::Symbol(key).index()] = point;
        }
    }

    // Publish the pose and landmarks
    aprilslam::publishLandmarks(landmark_pub_, landmarks, frame_id);
    aprilslam::publishPath(path_pub_, keyframeEstimates_, index_of_pose, frame_id);

    // Save the landmarks into a CSV file if required
    if (savetaglocation) {
        saveLandmarksToCSV(landmarks, pathtosavelandmarkcsv);
    }
    optimizationExecuted_ = true;
    ROS_INFO("SAMOptimise() executed successfully.");
}

// Callback function for mCam topic
void aprilslamcpp::mCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    if (msg->detections.empty()) {
        // No detections in the message, so we consider the data as "empty"
        mCam_data_received_ = false;
    } else {
        // Valid data received (detections are present)
        mCam_msg = msg;
        mCam_data_received_ = true;
    }
}

// Callback function for rCam topic
void aprilslamcpp::rCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    if (msg->detections.empty()) {
        // No detections in the message, so we consider the data as "empty"
        rCam_data_received_ = false;
    } else {
        // Valid data received (detections are present)
        rCam_msg = msg;
        rCam_data_received_ = true;
    }
}

// Callback function for lCam topic
void aprilslamcpp::lCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    if (msg->detections.empty()) {
        // No detections in the message, so we consider the data as "empty"
        lCam_data_received_ = false;
    } else {
        // Valid data received (detections are present)
        lCam_msg = msg;
        lCam_data_received_ = true;
    }
}


// Initialization of GTSAM components
void aprilslamcpp::initializeGTSAM() { 
    // Initialize graph parameters and stores them in isam_.
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = gtsam::ISAM2(parameters);
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

void aprilslamcpp::SAMOptimise() {    
    // Perform batch optimization using Levenberg-Marquardt optimizer
    gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    gtsam::Values result = batchOptimizer.optimize();

    // Update keyframeEstimates_ with the optimized values for the next iteration
    keyframeEstimates_ = result;
}

void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
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
        keyframeGraph_.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('X', 1), pose0, priorNoise));
        keyframeEstimates_.insert(gtsam::Symbol('X', 1), pose0);
        lastPose_ = pose0; // Keep track of the last pose for odolandmarkKeymetry calculation
        // Load calibrated landmarks as priors if available
        if (usepriortagtable) {
            std::map<int, gtsam::Point2> savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);
            for (const auto& landmark : savedLandmarks) {
                gtsam::Symbol landmarkKey('L', landmark.first);
                keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(landmarkKey, landmark.second, pointNoise));
                keyframeEstimates_.insert(landmarkKey, landmark.second);
                landmarkEstimates.insert(landmarkKey, landmark.second);
            }
        }
        Key_previous_pos = pose0;
        previousKeyframeSymbol = gtsam::Symbol('X', 1);
    }

    // Predict the next pose based on odometry and add it as an initial estimate
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
    gtsam::Pose2 predictedPose = lastPose_.compose(odometry);

    // Determine if this pose should be a keyframe
    gtsam::Symbol currentKeyframeSymbol('X', index_of_pose);

    // Add odometry factor
    keyframeEstimates_.insert(gtsam::Symbol('X', index_of_pose), predictedPose);
    if (previousKeyframeSymbol) {
        gtsam::Pose2 relativePose = Key_previous_pos.between(predictedPose);
        keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
    }
        
    // Update the last pose and initial estimates for the next iteration
    lastPose_ = predictedPose;
    landmarkEstimates.insert(gtsam::Symbol('X', index_of_pose), predictedPose);

    // Iterate through all landmark detected IDs
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

                    // Check if the key already exists in landmarkEstimates before inserting
                    if (landmarkEstimates.exists(landmarkKey)) {
                    } else {
                        landmarkEstimates.insert(landmarkKey, priorLand);
                    }

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
                // Store the bearing and range measurements in the map
                poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = std::make_tuple(bearing, range);                
            }
        }
    }      
    lastPoseSE2_ = poseSE2;
    Key_previous_pos = predictedPose;

    // Visulisation
    previousKeyframeSymbol = gtsam::Symbol('X', index_of_pose);
     // Extract landmark estimates from the result
    std::map<int, gtsam::Point2> landmarks;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;  // Get the key
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point2 point = keyframeEstimates_.at<gtsam::Point2>(key);  // Access the Point2 value
            landmarks[gtsam::Symbol(key).index()] = point;
        }
    }
    // Publish the pose and landmarks
    aprilslam::publishLandmarks(landmark_pub_, landmarks, frame_id);
    aprilslam::publishPath(path_pub_, keyframeEstimates_, index_of_pose, frame_id);

    // Save the landmarks into a CSV file if required
    if (savetaglocation) {
        saveLandmarksToCSV(landmarks, pathtosavelandmarkcsv);
    }
}
}

int main(int argc, char **argv) {
    // Initialize the ROS system and specify the name of the node
    ros::init(argc, argv, "april_slam_cpp");

    // Create a handle to this process' node
    ros::NodeHandle nh;

    // Create an instance of the aprilslamcpp class, passing in the node handle
    aprilslam::aprilslamcpp slamNode(nh);

    // ROS enters a loop, pumping callbacks. Internally, it will call all the callbacks waiting to be called at that point in time.
    ros::spin();

    return 0;
}