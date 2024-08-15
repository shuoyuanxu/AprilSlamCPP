#ifndef PUBLISHING_UTILS_H
#define PUBLISHING_UTILS_H

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <gtsam/geometry/Pose2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For TF2 quaternion conversion functions
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <fstream>
#include <iostream>

namespace aprilslam {
    void publishLandmarks(ros::Publisher& landmark_pub, const std::map<int, gtsam::Point2>& landmarks, const std::string& frame_id);
    void publishPath(ros::Publisher& path_pub, const gtsam::Values& result, int max_index, const std::string& frame_id);
    void saveLandmarksToCSV(const std::map<int, gtsam::Point2>& landmarks, const std::string& filename);
    std::map<int, gtsam::Point2> loadLandmarksFromCSV(const std::string& filename);
}

#endif

