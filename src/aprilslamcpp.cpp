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

    // Assuming the robot is always moving forward
    double Dx = std::sqrt(dx * dx + dy * dy);
    double dtheta = wrapToPi(PoseSE2.theta() - lastPoseSE2.theta());

    // Create a Pose2 object for the relative pose
    // Note: Since Dy is assumed to be zero, it's omitted in constructing the Pose2 object
    return gtsam::Pose2(Dx, 0, dtheta);
}

// Constructor
aprilslamcpp::aprilslamcpp(ros::NodeHandle node_handle, ros::Duration cache_time)
    : nh_(node_handle), tf_buffer_(cache_time), tf_listener_(tf_buffer_){ 
    
    // Read topics
    std::string odom_topic, trajectory_topic;
    nh_.getParam("odom_topic", odom_topic);
    nh_.getParam("trajectory_topic", trajectory_topic);
    nh_.getParam("frame_id", frame_id);
    
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

    // Initialize the factor graph
    graph_ = gtsam::NonlinearFactorGraph();

    // Initialize ISAM2 with parameters.
    gtsam::ISAM2Params parameters;
    isam_ = gtsam::ISAM2(parameters);

    ROS_INFO_STREAM("Initilisation Done");
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
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(graph_, initial_estimates_);
        initial_estimates_ = batchOptimizer.optimize();
        batchOptimisation_ = false; // Only do this once
    }

    // Update the iSAM2 instance with the new measurements
    isam_.update(graph_, initial_estimates_);

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
    // Clear the graph and initial estimates for the next iteration
    // graph_.resize(0);
    initial_estimates_.clear();
}   

void aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    ros::WallTime start_loop, end_loop; // Declare variables to hold start and end times=
    double elapsed;
    index_of_pose += 1; // Increment the pose index for each new odometry message

    // Convert the incoming odometry message to a simpler (x, y, theta) format using a previously defined method
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);

    // Store the initial pose for relative calculations
    if (index_of_pose == 2) {
        lastPoseSE2_ = poseSE2;
        gtsam::Pose2 pose0(0.0, 0.0, 0.0); // Prior at origin
        graph_.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('X', 1), pose0, priorNoise));
        initial_estimates_.insert(gtsam::Symbol('X', 1), pose0);
        lastPose_ = pose0; // Keep track of the last pose for odometry calculation
    }

    // Predict the next pose based on odometry and add it as an initial estimate
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
    gtsam::Pose2 predictedPose = lastPose_.compose(odometry);

    // Add this relative motion as an odometry factor to the graph
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose - 1), gtsam::Symbol('X', index_of_pose), odometry, odometryNoise));

    // Update the last pose and initial estimates for the next iteration
    lastPose_ = predictedPose;
    initial_estimates_.insert(gtsam::Symbol('X', index_of_pose), poseSE2);
    landmarkEstimates.insert(gtsam::Symbol('X', index_of_pose), poseSE2);
    
    // Iterate through possible landmark IDs to check for observations
    start_loop = ros::WallTime::now();
    for (const auto& tag_id : possibleIds_) {    
        try {
            // Find transformation between vehicle and landmarks (see if landmarks are detected)
            geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform("base_link", tag_id, ros::Time(0));

            // Extract the transform details
            double trans_x = transformStamped.transform.translation.x;
            double trans_y = transformStamped.transform.translation.y;
            
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
                if (fabs(error[0]) < add2graph_threshold) graph_.push_back(factor);
            } 
            else {
                 // New landmark detected
                tagToNodeIDMap_[tag_number] = landmarkKey;
                initial_estimates_.insert(landmarkKey, gtsam::Point2(trans_x, trans_y)); // Simple initial estimate
                landmarkEstimates.insert(landmarkKey, gtsam::Point2(trans_x, trans_y));

                // Add a prior for the landmark position to help with initial estimation.
                graph_.add(gtsam::PriorFactor<gtsam::Point2>(
                    landmarkKey, gtsam::Point2(trans_x, trans_y), pointNoise)
                );
                // Add a bearing-range observation for this landmark to the graph
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
                graph_.add(factor);

            }
        }
        catch (tf2::TransformException &ex) {
                continue;
        }
}    
    end_loop = ros::WallTime::now();
    elapsed = (end_loop - start_loop).toSec();
    ROS_INFO("transform total: %f seconds", elapsed);
    lastPoseSE2_ = poseSE2;
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

    ros::Duration cache_time(0.1);

    // Create an instance of the aprilslamcpp class, passing in the node handle
    aprilslam::aprilslamcpp slamNode(nh, cache_time);

    // ROS enters a loop, pumping callbacks. Internally, it will call all the callbacks waiting to be called at that point in time.
    ros::spin();

    return 0;
}