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

void saveGraphToDot(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& estimates, const std::string& filename) {
    std::ofstream os(filename);
    if (!os) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

    // Start the DOT file
    os << "digraph G {" << std::endl;

    // Write nodes for the values
    for(const auto& key_value: estimates) {
        auto key = key_value.key;
        auto symbol = gtsam::Symbol(key);
        os << "  " << symbol << " [label=\"" << symbol.chr() << symbol.index() << "\"];" << std::endl;
    }

    // Iterate over the factors (this is a simplistic representation)
    for(size_t i = 0; i < graph.size(); ++i) {
        auto factor = graph.at(i);
        if (factor) {
            // Example: write the factor as a node
            os << "  Factor" << i << " [label=\"Factor " << i << "\", shape=box];" << std::endl;
            // Connect the factor to each of its keys (variables)
            for(const auto& key: factor->keys()) {
                auto symbol = gtsam::Symbol(key);
                os << "  Factor" << i << " -> " << symbol << ";" << std::endl;
            }
        }
    }

    // Optionally, add nodes or other attributes for the variables in `estimates`

    // Finish the DOT file
    os << "}" << std::endl;

    os.close();
}



// Constructor
AprilSlamCPP::AprilSlamCPP(ros::NodeHandle node_handle)
    : nh_(node_handle), tf_listener_(tf_buffer_){ 

    // Initialize GTSAM components
    initializeGTSAM();

    // Subscribe to odometry topic
    odom_sub_ = nh_.subscribe("/odometry/filtered", 1000, &AprilSlamCPP::addOdomFactor, this);
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


void AprilSlamCPP::ISAM2Optimise() {
    ros::WallTime start, end; // Declare variables to hold start and end times
    double elapsed;
    
    start = ros::WallTime::now(); // Start timing
    ROS_INFO("Start Optimisation:");
    if (batchInitialization_) {
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(graph_, initial_estimates_);
        initial_estimates_ = batchOptimizer.optimize();
        batchInitialization_ = false; // Only do this once
    }
    ROS_INFO("Optimisation time: %f seconds", elapsed);

    // Update the iSAM2 instance with the new measurements
    isam_.update(graph_, initial_estimates_);

    // Calculate the current best estimate
    auto result = isam_.calculateEstimate();

    // Update the last pose based on the latest estimates
    gtsam::Pose2 lastPose_ = result.at<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose));
    end = ros::WallTime::now(); // End timing
    elapsed = (end - start).toSec();
    ROS_INFO("Optimisation time: %f seconds", elapsed);

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
    ros::WallTime start, end; // Declare variables to hold start and end times
    double elapsed;
    
    index_of_pose += 1; // Increment the pose index for each new odometry message

    // Convert the incoming odometry message to a simpler (x, y, theta) format using a previously defined method
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);
    end = ros::WallTime::now();
    elapsed = (end - start).toSec();
    ROS_INFO("Odometry message translation time: %f seconds", elapsed);

    // Store the initial pose for relative calculations
    if (index_of_pose == 2) {
        lastPoseSE2_ = poseSE2;
        gtsam::Pose2 pose0(0.0, 0.0, 0.0); // Prior at origin
        graph_.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('X', 1), pose0, priorNoise));
        initial_estimates_.insert(gtsam::Symbol('X', 1), pose0);
        lastPose_ = pose0; // Keep track of the last pose for odometry calculation
    }

    // Predict the next pose based on odometry and add it as an initial estimate
    start = ros::WallTime::now();
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
    gtsam::Pose2 predictedPose = lastPose_.compose(odometry);
    ROS_INFO("Odometry: x=%f, y=%f, yaw=%f", odometry.x(), odometry.y(), odometry.theta());

    // Add this relative motion as an odometry factor to the graph
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose - 1), gtsam::Symbol('X', index_of_pose), odometry, odometryNoise));

    // Update the last pose and initial estimates for the next iteration
    lastPose_ = predictedPose;
    initial_estimates_.insert(gtsam::Symbol('X', index_of_pose), poseSE2);
    landmarkEstimates.insert(gtsam::Symbol('X', index_of_pose), poseSE2);

    end = ros::WallTime::now();
    elapsed = (end - start).toSec();
    ROS_INFO("Pose prediction and graph update time: %f seconds", elapsed);

    // Iterate through possible landmark IDs to check for observations
    start = ros::WallTime::now();
    for (const auto& tag_id : possibleIds_) {    
        try {
            geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform("base_link", tag_id, ros::Time(0), ros::Duration(0.01));
            // Use the transform as needed
            end = ros::WallTime::now();
            elapsed = (end - start).toSec();
            ROS_INFO("tramnsform time: %f seconds", elapsed);

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
            end = ros::WallTime::now();
            elapsed = (end - start).toSec();
            ROS_INFO("Odometry total: %f seconds", elapsed);

            // Check if the landmark has been observed before
            if (tagToNodeIDMap_.find(tag_number) != tagToNodeIDMap_.end()) {
                // Existing landmark
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
                // gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                // Threshold for ||projection - measurement||
                // if (fabs(error[0]) < 0.2) graph_.push_back(factor);
                ROS_INFO("landmark br factor added");
                graph_.add(factor);
            } 
            else {
                 // New landmark detected
                tagToNodeIDMap_[tag_number] = landmarkKey;
                initial_estimates_.insert(landmarkKey, gtsam::Point2(trans_x, trans_y)); // Simple initial estimate
                landmarkEstimates.insert(landmarkKey, gtsam::Point2(trans_x, trans_y));

                // Add a prior for the landmark position to help with initial estimation.
                // graph_.add(gtsam::PriorFactor<gtsam::Point2>(
                //     landmarkKey, gtsam::Point2(trans_x, trans_y), pointNoise)
                // );
                // Add a bearing-range observation for this landmark to the graph
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
                graph_.add(factor);

            }
        //     double trans_x = transformStamped.transform.translation.x;
        //     double trans_y = transformStamped.transform.translation.y;
        //     double qx = transformStamped.transform.rotation.x;
        //     double qy = transformStamped.transform.rotation.y;
        //     double qz = transformStamped.transform.rotation.z;
        //     double qw = transformStamped.transform.rotation.w;
        //     tf2::Quaternion tfQuatRel(qx, qy, qz, qw);
        //     double rollRel, pitchRel, yawRel;
        //     tf2::Matrix3x3(tfQuatRel).getRPY(rollRel, pitchRel, yawRel);

        //     gtsam::Pose2 tagPoseInBaseLink(trans_x, trans_y, yawRel);
        //     ROS_INFO("tagPoseInBaseLink: x=%f, y=%f, yaw=%f", tagPoseInBaseLink.x(), tagPoseInBaseLink.y(), tagPoseInBaseLink.theta());
        //     // Split the tag_id to get the tag number. Assume tag IDs are in the format "tag_X"
        //     auto underscorePos = tag_id.find('_');
        //     if (underscorePos == std::string::npos) continue; // Skip if the format is unexpected
        //     int tag_number = std::stoi(tag_id.substr(underscorePos + 1)) +1;
    
        //     // Construct the landmark key
        //     gtsam::Symbol landmarkKey('L', tag_number);

        //     if (tagToNodeIDMap_.find(tag_number) == tagToNodeIDMap_.end()) {
        //         // New landmark detected
        //         tagToNodeIDMap_[tag_number] = landmarkKey;
        //         initial_estimates_.insert(landmarkKey, gtsam::Point2(trans_x, trans_y));
        //         landmarkEstimates.insert(landmarkKey, gtsam::Point2(trans_x, trans_y));
        //         // Add a prior factor for the new landmark's position
        //         // graph_.add(gtsam::PriorFactor<gtsam::Point2>(landmarkKey, gtsam::Point2(), pointNoise));

        //         // Add a BetweenFactor for the landmark and current pose
        //         graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose), landmarkKey, tagPoseInBaseLink, pointNoise));

        //         ROS_INFO("New tag Detected");

        //     } else {
        //         // Existing landmark, only add a BetweenFactor
        //         graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose), landmarkKey, tagPoseInBaseLink, pointNoise));
        //     }
        //     ROS_INFO("Landmark ID: %d", tag_number);
        //     end = ros::WallTime::now();
        //     elapsed = (end - start).toSec();
        //     ROS_INFO("Landmark processing time: %f seconds", elapsed);
        
        }
        catch (tf2::TransformException &ex) {
                continue;
        }
    
    lastPoseSE2_ = poseSE2;
    saveGraphToDot(graph_, initial_estimates_, "/home/shuoyuan/factorGraph.dot");
    // If needed, perform an ISAM2 optimization to update the map and robot pose estimates
    if (index_of_pose % 1 == 0) { // Adjust this condition as necessary
        ISAM2Optimise();
    }
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
