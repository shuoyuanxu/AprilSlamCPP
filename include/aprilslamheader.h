#ifndef APRIL_SLAM_H
#define APRIL_SLAM_H
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <gtsam/geometry/Pose2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For TF2 quaternion conversion functions
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <cxxabi.h>
#include <boost/pointer_cast.hpp>

namespace aprilslam {

class aprilslamcpp {
public:
    explicit aprilslamcpp(ros::NodeHandle node_handle, ros::Duration cache_time); // Constructor declaration
    void initializeGTSAM(); // Method to initialize GTSAM components
    gtsam::Pose2 translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg); // Removed redundant class scope
    void ISAM2Optimise();
    void addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg);
    void pruneOldFactorsByTime(double current_time, double timewindow);
    void pruneOldFactorsBySize(double maxfactors);
    void checkLoopClosure(double current_time, const std::set<gtsam::Symbol>& detectedLandmarks);
    bool shouldAddKeyframe(const gtsam::Pose2& lastPose, const gtsam::Pose2& currentPose);
    void updateKeyframeGraphWithOptimizedResults(const gtsam::Values& optimizedResults);
    void createNewKeyframe(const gtsam::Pose2& predictedPose, gtsam::Symbol& previousKeyframeSymbol);
    void graphvisulisation(gtsam::NonlinearFactorGraph& Graph_);
    void printWindowEstimates(const gtsam::Values& windowEstimates);
private:
    ros::Publisher path_pub_;
    ros::Publisher landmark_pub_;
    nav_msgs::Path path;
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::map<gtsam::Symbol, std::map<gtsam::Symbol, std::tuple<double, double>>> poseToLandmarkMeasurementsMap;  //storing X-L pair
    std::map<gtsam::Key, gtsam::Point2> historicLandmarks;     // Maintain a persistent storage for historic landmarks
    gtsam::Values landmarkEstimates;  // for unwhitten error computing 

    gtsam::NonlinearFactorGraph keyframeGraph_;  // Keyframe graph: All keyframes and associated landmarks
    gtsam::Values keyframeEstimates_;            // Estimates for keyframes

    gtsam::NonlinearFactorGraph windowGraph_;    // Window graph: Current time window, built on top of keyframeGraph
    gtsam::Values windowEstimates_;              // Estimates for current time window

    gtsam::Pose2 Key_previous_pos;
    gtsam::Symbol previousKeyframeSymbol;
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
    double transformation_search_range;
    double add2graph_threshold;
    std::string frame_id;
    std::string robot_frame;
    std::string pathtosavelandmarkcsv;
    std::string pathtoloadlandmarkcsv;
    std::string calibrationrun;
    std::map<size_t, double> factorTimestamps_; // Track timestamps of factors
    double timeWindow; // The time window for maintaining the graph
    double maxfactors; // Allowed total number of factors in the graph before pruning
    bool useprunebytime;
    bool useprunebysize;
    bool loopClosureEnableFlag;
    int loopClosureFrequency;
    int surroundingKeyframeSize;
    double historyKeyframeSearchRadius;
    int historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    bool savetaglocation;
    bool usepriortagtable;
    std::map<int, gtsam::Point2> savedLandmarks;
    std::map<gtsam::Symbol, std::set<gtsam::Symbol>> poseToLandmarks; // Maps pose index to a set of detected landmark IDs, e.g. X1: L1,L2,L3.
    double distanceThreshold;
    double rotationThreshold;
};

} 

#endif // aprilslamcpp
