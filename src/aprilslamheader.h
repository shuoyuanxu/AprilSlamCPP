#ifndef aprilslamheader
#define aprilslamheader
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For TF2 quaternion conversion functions
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <nav_msgs/Path.h>

namespace aprislamcpp{

class aprilslamcpp {
public:
    explicit aprilslamcpp(ros::NodeHandle node_handle); // Constructor declaration
    void initializeGTSAM(); // Method to initialize GTSAM components
    gtsam::Pose2 translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg); // Removed redundant class scope
    void ISAM2Optimise();
    void addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg);
    ros::Publisher path_pub_;
    nav_msgs::Path path;

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimates_;
    gtsam::Values landmarkEstimates;
    gtsam::ISAM2 isam_;
    gtsam::Pose2 lastPoseSE2_;
    gtsam::Pose2 lastPose_;
    std::vector<std::string> possibleIds_; // Predefined tags in the environment
    std::map<int, gtsam::Symbol> tagToNodeIDMap_; // Map from tag IDs to node IDs
    int index_of_pose;
    bool batchOptimisation_;
    // Noise Models
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr brNoise;
    gtsam::noiseModel::Diagonal::shared_ptr pointNoise;

    // ROS odometry message callback
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // Helper methods
    gtsam::Pose2 convertOdometryToGTSAMPose(const nav_msgs::Odometry::ConstPtr& msg);
    void addOdometryFactor(const gtsam::Pose2& odometry);
    void updateAndOptimize();
};

} 

#endif // aprilslamcpp
