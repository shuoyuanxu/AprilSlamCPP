#include "AprilSlam_Header.h"

namespace aprislamcpp {
// Utility function to normalize an angle to the range [-pi, pi]
double wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0)
        angle += 2 * M_PI;
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
AprilSlamCPP::AprilSlamCPP(ros::NodeHandle node_handle)
    : nh_(node_handle), tf_listener_(tf_buffer_){ 

    // Initialize GTSAM components
    initializeGTSAM();

    // Subscribe to odometry topic
    odom_sub_ = nh_.subscribe("/odometry/filtered", 1000, &AprilSlamCPP::odomCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("slam_pose", 10);
}


void AprilSlamCPP::initializeGTSAM() {
    // Initialize noise models
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.0001, 0.01, 0.0001).finished());
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.1, 0.3, 0.1).finished());
    brNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 0.1, 0.8).finished()); // Note: Vector size matches bearing-range model
    pointNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 10, 10).finished()); // Note: Vector size matches point landmark model
    
    // Index to keep track of the sequential pose.
    index_of_pose = 1;

    // Initialize the factor graph
    graph_ = gtsam::NonlinearFactorGraph();

    // Initialize ISAM2 with parameters.
    gtsam::ISAM2Params parameters;
    parameters.setRelinearizeThreshold(0.1);  // Threshold for re-linearization
    isam_ = gtsam::ISAM2(parameters);
    batchInitialization_ = true;  // Flag to indicate if batch initialization is required.

    // Debugging/Initialization message.
    ROS_INFO("Initialised GTSAM SLAM system.");

    // Predefined tags to search for in the environment.
    for (int j = 0; j < 89; ++j) {
        possibleIds_.push_back("tag_" + std::to_string(j));
    }
    ROS_INFO_STREAM("Possible landmark IDs initialised.");

    // Transform listener for real-time pose estimation.
    // tf.TransformListener
    // Determine the robot's relative position to observed landmarks.
    // Update the SLAM graph with new measurements
    // that refine the robot's estimated trajectory and the map of the environment.
    landCount_ = 0; 
}


gtsam::Pose2 AprilSlamCPP::translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    tf2::Quaternion tfQuat(qx, qy, qz, qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
    ROS_INFO("Translated_Pose: x=%f, y=%f, yaw=%f", x, y, yaw);
    return gtsam::Pose2(x, y, yaw);
}

void AprilSlamCPP::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Convert and process odometry messages
    addOdomFactor(msg);
}

void AprilSlamCPP::ISAM2Optimise() {
    // Check if we need to perform a batch initialization
    if (batchInitialization_) {
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(graph_, initial_estimates_);
        initial_estimates_ = batchOptimizer.optimize();
        batchInitialization_ = false; // Only do this once
    }

    // Update the iSAM2 instance with the new measurements
    isam_.update(graph_, initial_estimates_);

    // Calculate the current best estimate
    auto result = isam_.calculateEstimate();

    // Update the last pose based on the latest estimates
    // Assuming 'i' is properly maintained elsewhere in your class
    gtsam::Pose2 lastPose_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', index_of_pose));

    // Convert lastPose_ to PoseStamped message
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "map"; // Change according to your frame setup
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.pose.position.x = lastPose_.x();
    poseMsg.pose.position.y = lastPose_.y();
    poseMsg.pose.position.z = 0; // Assuming 2D
    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, lastPose_.theta());
    poseMsg.pose.orientation = tf2::toMsg(quat);

    // Publish the pose
    pose_pub_.publish(poseMsg);
    
    // Clear the graph and initial estimates for the next iteration
    graph_.resize(0);
    initial_estimates_.clear();
}

void AprilSlamCPP::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    index_of_pose += 1; // Increment the pose index for each new odometry message

    // Convert the incoming odometry message to a simpler (x, y, theta) format using a previously defined method
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);

    // Store the initial pose for relative calculations
    if (index_of_pose == 2) {
        lastPoseSE2_ = poseSE2;
        gtsam::Pose2 pose0(0.0, 0.0, 0.0); // Prior at origin
        graph_.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('x', 1), pose0, priorNoise));
        ROS_INFO("added first pose prior");
        initial_estimates_.insert(gtsam::Symbol('x', 1), pose0);
        lastPose_ = pose0; // Keep track of the last pose for odometry calculation
    }

    // Predict the next pose based on odometry and add it as an initial estimate
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
    gtsam::Pose2 predictedPose = lastPose_.compose(odometry);
    ROS_INFO("Odometry: x=%f, y=%f, yaw=%f", odometry.x(), odometry.y(), odometry.theta());

    // Add this relative motion as an odometry factor to the graph
    ROS_INFO("adding between");
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('x', index_of_pose - 1), gtsam::Symbol('x', index_of_pose), odometry, odometryNoise));

    // Update the last pose and initial estimates for the next iteration
    lastPose_ = predictedPose;
    initial_estimates_.insert(gtsam::Symbol('x', index_of_pose), poseSE2);
    landmarkEstimates.insert(gtsam::Symbol('x', index_of_pose), poseSE2);
    
    // Iterate through possible landmark IDs to check for observations
    for (const auto& tag_id : possibleIds_) {    
        // Check if a transform is available from base_link to the landmark
        if (tf_buffer_.canTransform("base_link", tag_id, ros::Time(0), ros::Duration(0.1))) {
            geometry_msgs::TransformStamped transformStamped;
            try {
                transformStamped = tf_buffer_.lookupTransform("base_link", tag_id, ros::Time(0), ros::Duration(0.1));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                continue;
            }

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
                ROS_INFO("Index of pose wrong: %d", index_of_pose);
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('x', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
                gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                // Threshold for ||projection - measurement||
                if (fabs(error[0]) < 0.2) graph_.add(factor);
            } 
            else {
                 // New landmark detected
                tagToNodeIDMap_[tag_number] = landmarkKey;
                initial_estimates_.insert(landmarkKey, gtsam::Point2(trans_x, trans_y)); // Simple initial estimate
                landmarkEstimates.insert(landmarkKey, gtsam::Point2(trans_x, trans_y));

                // Add a prior for the landmark position to help with initial estimation.
                ROS_INFO("adding landmark priors");
                graph_.add(gtsam::PriorFactor(landmarkKey, gtsam::Point2(trans_x, trans_y), pointNoise));

                // Add a bearing-range observation for this landmark to the graph
                ROS_INFO("adding BR factor");
                ROS_INFO("Index of pose: %d", index_of_pose);
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('x', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
            }
        }
    }

    lastPoseSE2_ = poseSE2;
    // If needed, perform an ISAM2 optimization to update the map and robot pose estimates
    if (index_of_pose % 1 == 0) { // Adjust this condition as necessary
        ISAM2Optimise();
    }
}
}

int main(int argc, char **argv) {
    // Initialize the ROS system and specify the name of the node
    ros::init(argc, argv, "april_slam_cpp");

    // Create a handle to this process' node
    ros::NodeHandle nh;

    // Create an instance of the AprilSlamCPP class, passing in the node handle
    aprislamcpp::AprilSlamCPP slamNode(nh);

    // ROS enters a loop, pumping callbacks. Internally, it will call all the callbacks waiting to be called at that point in time.
    ros::spin();

    return 0;
}
