// publishing_utils.cpp

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

void publishLandmarks(ros::Publisher& landmark_pub, const std::map<int, gtsam::Point2>& landmarks, const std::string& frame_id) {
    visualization_msgs::MarkerArray markers;
    int id = 0;
    for (const auto& landmark : landmarks) {
        // Sphere marker for each landmark
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "landmarks";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = landmark.second.x();
        marker.pose.position.y = landmark.second.y();
        marker.pose.position.z = 0;  // Assuming the landmarks are on the ground plane
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);

        // Marker IDs
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "landmark_ids";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = landmark.second.x();
        text_marker.pose.position.y = landmark.second.y();
        text_marker.pose.position.z = 0.5;
        text_marker.scale.z = 0.2;
        text_marker.text = std::to_string(landmark.first);
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;

        markers.markers.push_back(text_marker);
    }

    landmark_pub.publish(markers);
}

void publishPath(ros::Publisher& path_pub, const gtsam::Values& result, int max_index, const std::string& frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();
    path.poses.clear();  // Clear any existing poses

    for (int i = 1; i <= max_index; i++) {
        gtsam::Symbol sym('X', i);
        if (result.exists(sym)) {
            gtsam::Pose2 pose = result.at<gtsam::Pose2>(sym);

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = frame_id;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose.position.x = pose.x();
            pose_msg.pose.position.y = pose.y();
            pose_msg.pose.position.z = 0;  // Assuming 2D

            tf2::Quaternion quat;
            quat.setRPY(0, 0, pose.theta());
            pose_msg.pose.orientation = tf2::toMsg(quat);

            path.poses.push_back(pose_msg);
        }
    }

    path_pub.publish(path);
}

void publishOdometryTrajectory(ros::Publisher& odom_pub, tf2_ros::TransformBroadcaster& tf_broadcaster, 
                                        const gtsam::Values& result, int latest_index, 
                                        const std::string& frame_id, const std::string& child_frame_id) {
    // Define the symbol for the latest pose
    gtsam::Symbol sym('X', latest_index);

    // Check if the latest pose exists in the result
    if (result.exists(sym)) {
        gtsam::Pose2 pose = result.at<gtsam::Pose2>(sym);

        // 1. Publish the Odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = frame_id;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.child_frame_id = child_frame_id;

        // Set position
        odom_msg.pose.pose.position.x = pose.x();
        odom_msg.pose.pose.position.y = pose.y();
        odom_msg.pose.pose.position.z = 0;  // Assuming 2D

        // Set orientation
        tf2::Quaternion quat;
        quat.setRPY(0, 0, pose.theta());
        odom_msg.pose.pose.orientation = tf2::toMsg(quat);

        // Publish the odometry message
        odom_pub.publish(odom_msg);

        // 2. Broadcast the transform
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = frame_id;      // Typically "map"
        transform_stamped.child_frame_id = child_frame_id; // Typically "base_link"

        // Set translation
        transform_stamped.transform.translation.x = pose.x();
        transform_stamped.transform.translation.y = pose.y();
        transform_stamped.transform.translation.z = 0;

        // Set rotation
        transform_stamped.transform.rotation = tf2::toMsg(quat);

        // Broadcast the transform
        tf_broadcaster.sendTransform(transform_stamped);
    }
}

void saveLandmarksToCSV(const std::map<int, gtsam::Point2>& landmarks, const std::string& filename) {
    std::ofstream file;
    file.open(filename, std::ios::out); // Open file in write mode

    if (!file) {
        std::cerr << "Failed to open the file!" << std::endl;
        return;
    }
    // Write the header line
    file << "id,x,y\n";
    
    for (const auto& landmark : landmarks) {
        int id = landmark.first;
        gtsam::Point2 point = landmark.second;
        file << id << "," << point.x() << "," << point.y() << "\n";
    }

    file.close();
}

std::map<int, gtsam::Point2> loadLandmarksFromCSV(const std::string& filename) {
    std::map<int, gtsam::Point2> landmarks;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open the file!" << std::endl;
        return landmarks;
    }

    std::string line;

    // Skip the header line
    if (std::getline(file, line)) {
        // You can print or log the header if needed
        // std::cout << "Header: " << line << std::endl;
    }

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string id_str, x_str, y_str;
        if (std::getline(ss, id_str, ',') && std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            try {
                int id = std::stoi(id_str);
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                landmarks[id] = gtsam::Point2(x, y);
            } catch (const std::exception& e) {
                std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
            }
        }
    }

    file.close();
    return landmarks;
}

// funtion for computing tag locations from coordinate transformation
void processDetections(const apriltag_ros::AprilTagDetectionArray::ConstPtr& cam_msg, 
                       const Eigen::Vector3d& xyTrans_cam_baselink,
                       std::vector<int>& Ids, 
                       std::vector<Eigen::Vector2d>& tagPoss) {
    if (cam_msg) {
        for (const auto& detection : cam_msg->detections) {
            Ids.push_back(detection.id[0]);
            double rotTheta = xyTrans_cam_baselink(2);
            Eigen::Matrix2d R;
            R << cos(rotTheta), -sin(rotTheta),
                 sin(rotTheta),  cos(rotTheta);
            Eigen::Vector2d rotP = R * Eigen::Vector2d(detection.pose.pose.pose.position.z, -detection.pose.pose.pose.position.x);
            tagPoss.push_back(rotP + xyTrans_cam_baselink.head<2>());
        }
    }
}

// funtion for processing tag detection topics into IDs and TagPoss
std::pair<std::vector<int>, std::vector<Eigen::Vector2d>> getCamDetections(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& mCam_msg,
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& rCam_msg,
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& lCam_msg,
    const Eigen::Vector3d& xyTrans_mcam_baselink,
    const Eigen::Vector3d& xyTrans_rcam_baselink,
    const Eigen::Vector3d& xyTrans_lcam_baselink) {

    std::vector<int> Ids;
    std::vector<Eigen::Vector2d> tagPoss;

    // Process detections from each camera
    processDetections(mCam_msg, xyTrans_mcam_baselink, Ids, tagPoss);
    processDetections(rCam_msg, xyTrans_rcam_baselink, Ids, tagPoss);
    processDetections(lCam_msg, xyTrans_lcam_baselink, Ids, tagPoss);

    return std::make_pair(Ids, tagPoss);
}

void visualizeLoopClosure(ros::Publisher& lc_pub, const gtsam::Pose2& currentPose, const gtsam::Pose2& keyframePose, int currentPoseIndex, const std::string& frame_id) {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "loop_closure_line";
    line_marker.id = currentPoseIndex;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;

    // Set the color to green
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    // Set line thickness
    line_marker.scale.x = 0.05;

    // Add points to the marker for the keyframe and current pose
    geometry_msgs::Point p1, p2;
    p1.x = keyframePose.x();
    p1.y = keyframePose.y();
    p1.z = 0;

    p2.x = currentPose.x();
    p2.y = currentPose.y();
    p2.z = 0;

    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);

    // Publish the line marker
    lc_pub.publish(line_marker);
}

void particleFilter(
    const std::vector<int>& Id,
    const Eigen::MatrixXd& tagPos,
    const std::map<int, gtsam::Point2>& savedLandmarks,
    Eigen::MatrixXd& x_P,
    int N,
    double rngVar,
    double brngVar,
    Eigen::VectorXd& x_est
) {
    int numLandmarks = (int)Id.size();
    Eigen::MatrixXd z(2, numLandmarks);

    // Preprocess measurements
    for (int n = 0; n < numLandmarks; ++n) {
        Eigen::Vector2d lp = tagPos.row(n).transpose();
        double range = lp.norm();
        double brng = std::atan2(lp(1), lp(0));
        z(0, n) = range;
        z(1, n) = brng;
    }

    // Extracting landmark positions from savedLandmarks map
    Eigen::MatrixXd egoPos(2, numLandmarks);
    for (int n = 0; n < numLandmarks; ++n) {
        int landmarkID = Id[n];
        auto it = savedLandmarks.find(landmarkID);
        if (it != savedLandmarks.end()) {
            const gtsam::Point2& Lpos = it->second;
            egoPos(0, n) = Lpos.x();
            egoPos(1, n) = Lpos.y();
        } else {
            // If the landmark is not found, set to NaN or handle as needed
            egoPos(0, n) = std::numeric_limits<double>::quiet_NaN();
            egoPos(1, n) = std::numeric_limits<double>::quiet_NaN();
        }
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> nd(0.0, 1.0);
    std::uniform_real_distribution<double> ud(0.0, 1.0);

    Eigen::MatrixXd x_P_update(3, N);
    Eigen::VectorXd P_w(N);

    // Particle Filter Update
    for (int i = 0; i < N; ++i) {
        // State prediction
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
        Q(0,0) = 1.0; 
        Q(1,1) = 1.0; 
        Q(2,2) = 0.4; // std dev for theta

        Eigen::Vector3d noise(nd(gen), nd(gen), nd(gen));
        Eigen::Vector3d xplus = A * x_P.col(i) + Q * noise;
        xplus(2) = wrapToPi(xplus(2));
        x_P_update.col(i) = xplus;

        // Measurement update
        double weight = 0.0;
        for (int l = 0; l < numLandmarks; ++l) {
            // Handle invalid landmarks (e.g., NaN)
            if (std::isnan(egoPos(0,l)) || std::isnan(egoPos(1,l))) {
                // If landmark position unknown, skip or handle differently
                continue;
            }

            Eigen::Vector2d targetPos = xplus.head(2);
            Eigen::Vector2d error = egoPos.col(l) - targetPos;
            double range_pred = error.norm();
            double bearing_pred = std::atan2(error(1), error(0)) - xplus(2);
            bearing_pred = std::atan2(std::sin(bearing_pred), std::cos(bearing_pred));

            double bearingError = z(1,l) - bearing_pred;
            if (bearingError < -M_PI) bearingError += 2*M_PI;
            else if (bearingError > M_PI) bearingError -= 2*M_PI;

            double rangeError = range_pred - z(0,l);

            double sr = std::sqrt(rngVar);
            double sb = std::sqrt(brngVar);

            double rl = std::exp(-0.5 * std::pow(rangeError/sr,2)) / (sr * std::sqrt(2*M_PI));
            double bl = std::exp(-0.5 * std::pow(bearingError/sb,2)) / (sb * std::sqrt(2*M_PI));

            weight += (rl * bl);
        }
        P_w(i) = weight;
    }

    // Normalize weights
    double sumW = P_w.sum();
    if (sumW == 0) {
        P_w = Eigen::VectorXd::Ones(N) / N;
    } else {
        P_w /= sumW;
    }

    // Resampling
    std::vector<double> cumsum(N);
    cumsum[0] = P_w(0);
    for (int i = 1; i < N; ++i) 
        cumsum[i] = cumsum[i-1] + P_w(i);

    Eigen::MatrixXd x_P_new(3, N);
    for (int i = 0; i < N; ++i) {
        double r = ud(gen);
        auto it = std::lower_bound(cumsum.begin(), cumsum.end(), r);
        int idx = (int)std::distance(cumsum.begin(), it);
        if (idx >= N) idx = N-1;
        x_P_new.col(i) = x_P_update.col(idx);
    }

    x_P = x_P_new;
    x_est = x_P.rowwise().mean();
}

std::map<double, Eigen::Vector3d> initParticles(int Ninit) {
    // Initial position thru a detection

    // Define grid boundaries
    double xmin = -3.0, xmax = 6.0;
    double ymin = -25.0, ymax = 145.0;

    // Compute grid dimensions
    double ratio = (xmax - xmin) / (ymax - ymin);
    int Nx = static_cast<int>(std::round(ratio * std::sqrt(Ninit)));
    int Ny = static_cast<int>(std::round(double(Ninit) / Nx));
    int N = Nx * Ny;

    // Generate linear spacing
    Eigen::VectorXd X_lin = Eigen::VectorXd::LinSpaced(Nx, xmin, xmax);
    Eigen::VectorXd Y_lin = Eigen::VectorXd::LinSpaced(Ny, ymin, ymax);

    // Random orientation generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> d_theta(0.0, M_PI);

    std::map<double, Eigen::Vector3d> x_P;

    int count = 0;
    for (int i = 0; i < Ny; ++i) {
        for (int j = 0; j < Nx; ++j) {
            Eigen::Vector3d particle;
            particle(0) = X_lin(j);
            particle(1) = Y_lin(i);
            double new_theta = d_theta(gen);
            particle(2) = wrapToPi(new_theta);

            // Assign a unique weight (simple increasing sequence)
            double w = double(count + 1) / double(N);
            x_P.emplace(w, particle);
            count++;
        }
    }
    return x_P;
}

} // namespace aprilslamcpp


