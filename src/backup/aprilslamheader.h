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
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <cmath>
#include <gtsam/nonlinear/Marginals.h>

namespace aprilslam {

class aprilslamcpp {
public:
    explicit aprilslamcpp(ros::NodeHandle node_handle); // Constructor
    ~aprilslamcpp(); // Deconstructor
    void initializeGTSAM(); // Method to initialize GTSAM components
    gtsam::Pose2 translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg); // Removed redundant class scope
    void ISAM2Optimise(); //ISAM optimiser
    void SAMOptimise(); //SAM optimiser
    void addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg);
    void checkLoopClosure(const std::set<gtsam::Symbol>& detectedLandmarks);
    bool shouldAddKeyframe(const gtsam::Pose2& lastPose, const gtsam::Pose2& currentPose, std::set<gtsam::Symbol> oldlandmarks, std::set<gtsam::Symbol> detectedLandmarksCurrentPos);
    void rebuildFactorGraphWithPosindex(const gtsam::ISAM2 &isam, const std::set<gtsam::Symbol> &poseKeys, const std::map<gtsam::Symbol, std::map<gtsam::Symbol, std::tuple<double, double>>> &poseToLandmarkMeasurementsMap);
    std::set<gtsam::Symbol> generatePosArray(const gtsam::Symbol& previousKeyframe, const gtsam::Symbol& currentKeyframe, const std::set<gtsam::Symbol>& keyframes);    
    void createNewKeyframe(const gtsam::Pose2& predictedPose, const gtsam::Pose2& previousPose, gtsam::Symbol& previousKeyframeSymbol);
    void printWindowEstimates(const gtsam::Values& windowEstimates);
    void mCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void rCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void lCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void marginalizenonKeyframes(gtsam::ISAM2& isam, const std::set<gtsam::Symbol>& keyframesToMarginalize, const gtsam::NonlinearFactorGraph& originalGraph);
    std::set<gtsam::Symbol> getPoseKeysBetweenKeyframes(const gtsam::Symbol& previousKeyframeSymbol, const gtsam::Symbol& currentKeyframeSymbol);
    void pruneGraphByPoseCount(int maxPoses);
private:
    ros::Timer check_data_timer_;  // Declare the timer here
    ros::Publisher path_pub_;
    ros::Publisher landmark_pub_;
    nav_msgs::Path path;
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber mCam_subscriber;
    ros::Subscriber rCam_subscriber;
    ros::Subscriber lCam_subscriber;
    apriltag_ros::AprilTagDetectionArray::ConstPtr mCam_msg;
    apriltag_ros::AprilTagDetectionArray::ConstPtr rCam_msg;
    apriltag_ros::AprilTagDetectionArray::ConstPtr lCam_msg;
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
    gtsam::ISAM2 isam_key;
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
    double maxfactors; // Allowed total number of factors in the graph before pruning
    bool useprunebysize;
    bool useloopclosure;
    int loopClosureFrequency;
    int surroundingKeyframeSize;
    double historyKeyframeSearchRadius;
    int historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    double stationary_position_threshold;
    double stationary_rotation_threshold;
    bool savetaglocation;
    bool usepriortagtable;
    std::map<int, gtsam::Point2> savedLandmarks;
    std::map<gtsam::Symbol, std::set<gtsam::Symbol>> poseToLandmarks; // Maps pose index to a set of detected landmark IDs, e.g. X1: L1,L2,L3.
    double distanceThreshold;
    double rotationThreshold;
    std::vector<double> xyTrans_lcam_baselink;
    std::vector<double> xyTrans_rcam_baselink;
    std::vector<double> xyTrans_mcam_baselink;
    std::string lCam_topic;
    std::string rCam_topic;
    std::string mCam_topic;
    Eigen::Vector3d lcam_baselink_transform;
    Eigen::Vector3d rcam_baselink_transform;
    Eigen::Vector3d mcam_baselink_transform;
    std::set<gtsam::Symbol> keyframePosIds;
    std::set<gtsam::Symbol> detectedLandmarksHistoric;

    // Keeps track of pose symbols in the order they were added
    std::deque<gtsam::Symbol> poseSymbols_;

    // Maps variables (poses and landmarks) to the indices of factors that involve them
    std::map<gtsam::Key, std::vector<size_t>> variableToFactorIndices_;

    // Collects indices of factors to remove
    std::vector<size_t> factorsToRemove;

    // Collects keys of variables to remove
    std::set<gtsam::Key> variablesToRemove;

    // Symbol of the pose that currently has a prior
    gtsam::Symbol priorPoseSymbol;
    gtsam::FastMap<gtsam::Symbol, bool> priorAddedToPose;

    // Flag to ensure SAMOptimise is called only once
    bool mCam_data_received_, rCam_data_received_, lCam_data_received_;
    bool optimizationExecuted_;
    double accumulated_time_;  // Accumulated time since last valid data

    // Optimisation type: true for ISAM2, false for SAM
    bool useisam2;

    //
    };
} 

#endif // aprilslamcpp