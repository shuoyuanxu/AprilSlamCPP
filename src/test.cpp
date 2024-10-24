void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    double current_time = ros::Time::now().toSec();

    // Convert the incoming odometry message to a simpler (x, y, theta) format
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);

    // Calculate distance and rotation change
    if (!movementExceedsThreshold(poseSE2)) return;

    // Increment pose index
    index_of_pose++;

    // Handle first pose initialization
    if (index_of_pose == 2) initializeFirstPose(poseSE2, current_time);

    // Predict next pose based on odometry
    gtsam::Pose2 predictedPose = predictNextPose(poseSE2);

    // Get camera detections and store them in a variable for future adjustments
    auto detections = getCamDetections(mCam_msg, rCam_msg, lCam_msg, mcam_baselink_transform, rcam_baselink_transform, lcam_baselink_transform);

    // Check if this pose should be a keyframe and update graph
    if (shouldAddKeyframe(Key_previous_pos, predictedPose, detectedLandmarksHistoric)) {
        updateGraphWithLandmarks(poseSE2, predictedPose, current_time, detections);  // Update graph with landmarks
        optimizeGraph();  // Optimize the graph after keyframe and landmark updates
    } else {
        updateOdometryPose(poseSE2);  // Update pose without adding a keyframe
    }

    // Publish updated results
    publishResults();
}

// Check if movement exceeds the stationary thresholds
bool aprilslam::aprilslamcpp::movementExceedsThreshold(const gtsam::Pose2& poseSE2) {
    double position_change = std::hypot(poseSE2.x() - lastPoseSE2_.x(), poseSE2.y() - lastPoseSE2_.y());
    double rotation_change = std::abs(wrapToPi(poseSE2.theta() - lastPoseSE2_.theta()));
    return position_change >= stationary_position_threshold || rotation_change >= stationary_rotation_threshold;
}

// Handle initialization of the first pose
void aprilslam::aprilslamcpp::initializeFirstPose(const gtsam::Pose2& poseSE2, double current_time) {
    gtsam::Pose2 pose0(0.0, 0.0, 0.0);  // Initial pose at origin
    lastPoseSE2_ = poseSE2;
    lastPose_ = pose0;
    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('X', 1), pose0, priorNoise));
    factorTimestamps_[keyframeGraph_.size() - 1] = current_time;
    keyframeEstimates_.insert(gtsam::Symbol('X', 1), pose0);
    previousKeyframeSymbol = gtsam::Symbol('X', 1);
    // Load calibrated landmarks
    if (usepriortagtable) loadLandmarkPriors();
}

// Predict the next pose based on odometry
gtsam::Pose2 aprilslam::aprilslamcpp::predictNextPose(const gtsam::Pose2& poseSE2) {
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
    return lastPose_.compose(odometry);
}

// Update the graph with keyframes and landmarks
void aprilslam::aprilslamcpp::updateGraphWithLandmarks(const gtsam::Pose2& poseSE2, const gtsam::Pose2& predictedPose, double current_time, const std::pair<std::vector<int>, std::vector<Eigen::Vector2d>>& detections) {
    gtsam::Symbol currentKeyframeSymbol('X', index_of_pose);
    keyframeEstimates_.insert(currentKeyframeSymbol, predictedPose);

    // Add odometry factor between keyframes
    if (previousKeyframeSymbol) {
        gtsam::Pose2 relativePose = Key_previous_pos.between(predictedPose);
        keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
    }
    factorTimestamps_[keyframeGraph_.size() - 1] = current_time;

    // Process landmark detections from cameras
    const std::vector<int>& Id = detections.first;
    const std::vector<Eigen::Vector2d>& tagPos = detections.second;

    if (!Id.empty()) {
        for (size_t n = 0; n < Id.size(); ++n) {
            int tag_number = Id[n];
            Eigen::Vector2d landSE2 = tagPos[n];

            // Compute prior location of the landmark using the current robot pose
            double theta = lastPose_.theta();
            Eigen::Rotation2Dd rotation(theta);
            Eigen::Vector2d rotatedPosition = rotation * landSE2;
            gtsam::Point2 priorLand(rotatedPosition.x() + lastPose_.x(), rotatedPosition.y() + lastPose_.y());

            // Compute bearing and range
            double bearing = std::atan2(landSE2(1), landSE2(0));
            double range = std::sqrt(landSE2(0) * landSE2(0) + landSE2(1) * landSE2(1));

            // Construct the landmark key
            gtsam::Symbol landmarkKey('L', tag_number);

            // Store the bearing and range measurements in the map
            poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = std::make_tuple(bearing, range);

            // Check if the landmark has been observed before
            if (detectedLandmarksHistoric.find(landmarkKey) != detectedLandmarksHistoric.end()) {
                // Existing landmark
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
                gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                // Add to graph if the error is below the threshold
                if (fabs(error[0]) < add2graph_threshold) {
                    keyframeGraph_.add(factor);
                    factorTimestamps_[keyframeGraph_.size() - 1] = current_time;
                    detectedLandmarksCurrentPos.insert(landmarkKey);
                }
            } else {
                // New landmark detected
                if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                    detectedLandmarksHistoric.insert(landmarkKey);

                    if (!keyframeEstimates_.exists(landmarkKey)) {
                        keyframeEstimates_.insert(landmarkKey, priorLand);  // Simple initial estimate
                    }

                    if (!landmarkEstimates.exists(landmarkKey)) {
                        landmarkEstimates.insert(landmarkKey, priorLand);
                    }

                    // Add a prior for the landmark position to help with initial estimation
                    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(landmarkKey, priorLand, pointNoise));
                    factorTimestamps_[keyframeGraph_.size() - 1] = current_time;
                }

                // Add a bearing-range observation for this landmark to the graph
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
                keyframeGraph_.add(factor);
                factorTimestamps_[keyframeGraph_.size() - 1] = current_time;
                detectedLandmarksCurrentPos.insert(landmarkKey);
            }
        }
    }

    // Update the last keyframe pose and symbol
    Key_previous_pos = predictedPose;
    previousKeyframeSymbol = currentKeyframeSymbol;
}

// Update odometry without adding a keyframe
void aprilslam::aprilslamcpp::updateOdometryPose(const gtsam::Pose2& poseSE2) {
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_vis, poseSE2);
    gtsam::Pose2 newPose = Estimates_visulisation.at<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose - 1)).compose(odometry);
    Estimates_visulisation.insert(gtsam::Symbol('X', index_of_pose), newPose);
    lastPoseSE2_vis = poseSE2;
}

// Optimize the factor graph
void aprilslam::aprilslamcpp::optimizeGraph() {
    if (useisam2) {
        ISAM2Optimise();
        auto result = isam_.calculateEstimate();
        updateLandmarkEstimates(result);
    } else {
        SAMOptimise();
        checkLoopClosure();
    }
}

// Publish the results
void aprilslam::aprilslamcpp::publishResults() {
    aprilslam::publishPath(path_pub_, Estimates_visulisation, index_of_pose, frame_id);
    aprilslam::publishOdometryTrajectory(odom_traj_pub_, tf_broadcaster, Estimates_visulisation, index_of_pose, frame_id, ud_frame);
}
