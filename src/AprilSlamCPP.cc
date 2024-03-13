#include "AprilSlamCPP.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/geometry/Pose2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For TF2 quaternion conversion functions
#include <gtsam/navigation/NonlinearFactorGraph.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>

using namespace aprislamcpp;
using symbol_shorthand::X; // For pose variables
using symbol_shorthand::L; // For landmark variables


// Constructor
AprilSlamCPP::AprilSlamCPP(const ros::NodeHandle& node_handle) :
    nh_(node_handle), tf_listener_(tf_buffer_){ 

    // Initialize GTSAM components
    initializeGTSAM();

    // Subscribe to odometry topic
    odom_sub_ = nh_.subscribe("/odometry/filtered", 1000, &AprilSlamCPP::odomCallback, this);
}


void AprilSlamCPP::initializeGTSAM() {
    // Initialize noise models
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.0001, 0.01, 0.0001).finished());
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.1, 0.3, 0.1).finished());
    brNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 0.1, 0.8).finished()); // Note: Vector size matches bearing-range model
    pointNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 10, 10).finished()); // Note: Vector size matches point landmark model
   
    // Initialize the factor graph
    graph_ = gtsam::NonlinearFactorGraph();

    // Initialize ISAM2 with parameters.
    gtsam::ISAM2Params parameters;
    parameters.setRelinearizeThreshold = 0.1;  // Threshold for re-linearization
    isam_ = gtsam::ISAM2(parameters);
    batchInitialization_ = true;  // Flag to indicate if batch initialization is required.

    // Add a prior on the first pose at the origin
    gtsam::Pose2 priorPose(0.0, 0.0, 0.0);
    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(X(1), priorPose, priorNoise));

    // Insert the initial pose estimate
    index_of_pose = 1
    initial_estimates_.insert(X(index_of_pose), priorPose);

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

    return gtsam::Pose2(x, y, yaw);
}

void AprilSlamCPP::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Convert and process odometry messages
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
    gtsam::Pose2 lastPose_ = result.at<gtsam::Pose2>(X(index_of_pose));

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
        graph_.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('x', 1), pose0, priorNoise_));
        initial_estimates_.insert(gtsam::Symbol('x', 1), pose0);
        lastPose_ = pose0; // Keep track of the last pose for odometry calculation
    }

    // Predict the next pose based on odometry and add it as an initial estimate
    gtsam::Pose2 odometry = poseSE2.compose(lastPoseSE2_.inverse());
    gtsam::Pose2 predictedPose = lastPose_.compose(odometry)

    // Add this relative motion as an odometry factor to the graph
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('x', index_of_pose - 1), gtsam::Symbol('x', index_of_pose), odometry, odometryNoise_));

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
            int tag_number = std::stoi(tag_id.substr(underscorePos + 1));
    
            // Construct the landmark key
            gtsam::Symbol landmarkKey('L', tag_number);

            // Check if the landmark has been observed before
            if (tagToNodeIDMap_.find(tag_number) != tagToNodeIDMap_.end()) {
                // Existing landmark
                auto factor = gtsam::BearingRangeFactor2D(gtsam::Symbol('X', i_), landmarkKey, gtsam::Rot2::fromAngle(brng), rng, brNoise_);
                gtsam::Vector(2) error = factor.unwhitenedError(landmarkEstimates);
                graph_.add(factor);
                
                if np.linalg.norm(error) < 100:
                        self.graph.add(
                            gtsam.BearingRangeFactor2D(self.i, landKey, gtsam.Rot2(brng), rng, self.BR_NOISE))
                else:
                        print('Error too high, not adding factor for landmark {}'.format(ii))


            } else {
                 // New landmark detected
                tagToNodeIDMap_[tag_number] = landmarkKey;
                initial_estimates_.insert(landmarkKey, gtsam::Point2(trans_x, trans_y)); // Simple initial estimate
                landmarkEstimates.insert(landmarkKey, gtsam::Point2(trans_x, trans_y));
            }

            // Add a bearing-range observation for this landmark to the graph
            graph_.add(gtsam::BearingRangeFactor2D(gtsam::Symbol('X', i_), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise_));
        }
    }

    lastPoseSE2_ = poseSE2;
    // If needed, perform an ISAM2 optimization to update the map and robot pose estimates
    if (index_of_pose % 1 == 0) { // Adjust this condition as necessary
        ISAM2Optimise();
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "april_slam_cpp");
    ros::NodeHandle nh;

    aprislamcpp::AprilSlamCPP aprilSlamNode(nh);
    
    ros::spin(); // Keep the node running and listening to callbacks.
    return 0;
}

