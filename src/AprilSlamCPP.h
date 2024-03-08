#ifndef AprilSlamCPP
#define AprilSlamCPP

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>

namespace AprilSlamCPP {

class AprilSlamCPP {
public:
    AprilSlamCPP(const ros::NodeHandle& node_handle);
    virtual ~AprilSlamCPP();

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimates_;
    gtsam::ISAM2 isam_;
    int pose_index_ = 1;

    // Method to initialize GTSAM components
    void initializeGTSAM();

    // ROS odometry message callback
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // Helper methods
    gtsam::Pose2 convertOdometryToGTSAMPose(const nav_msgs::Odometry::ConstPtr& msg);
    void addOdometryFactor(const gtsam::Pose2& odometry);
    void updateAndOptimize();
};

} // namespace my_gtsam_project

#endif // AprilSlamCPP

