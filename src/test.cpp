// Update the graph with landmarks detections
std::set<gtsam::Symbol> aprilslam::aprilslamcpp::updateGraphWithLandmarks(std::set<gtsam::Symbol> detectedLandmarksCurrentPos, const std::pair<std::vector<int>, std::vector<Eigen::Vector2d>>& detections) {

    // Access the elements of the std::pair   
    const std::vector<int>& Id = detections.first;
    const std::vector<Eigen::Vector2d>& tagPos = detections.second;

    if (!Id.empty()) {
        for (size_t n = 0; n < Id.size(); ++n) {
            int tag_number = Id[n];        
            Eigen::Vector2d landSE2 = tagPos[n];

            // Compute prior location of the landmark using the current robot pose
            double theta = lastPose_.theta();
            Eigen::Rotation2Dd rotation(theta);  // Create a 2D rotation matrix
            Eigen::Vector2d rotatedPosition = rotation * landSE2;  // Rotate the position into the robot's frame
            gtsam::Point2 priorLand(rotatedPosition.x() + lastPose_.x(), rotatedPosition.y() + lastPose_.y());

            // Compute bearing and range
            double bearing = std::atan2(landSE2(1), landSE2(0));
            double range = std::sqrt(landSE2(0) * landSE2(0) + landSE2(1) * landSE2(1));

            // Construct the landmark key
            gtsam::Symbol landmarkKey('L', tag_number);  

            // Check if the landmark has been observed before
            if (detectedLandmarksHistoric.find(landmarkKey) != detectedLandmarksHistoric.end()) {
                // Existing landmark
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
                gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                // Threshold for ||projection - measurement||
                if (fabs(error[0]) < add2graph_threshold) keyframeGraph_.add(factor);
                detectedLandmarksCurrentPos.insert(landmarkKey);
            } 
            else {
                // If the current landmark was not detected in the calibration run 
                // Or it's on calibration mode
                if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                // New landmark detected
                detectedLandmarksHistoric.insert(landmarkKey);
                // Check if the key already exists in keyframeEstimates_ before inserting
                if (keyframeEstimates_.exists(landmarkKey)) {
                } else {
                    keyframeEstimates_.insert(landmarkKey, priorLand); // Simple initial estimate
                }

                // Check if the key already exists in landmarkEstimates before inserting
                if (landmarkEstimates.exists(landmarkKey)) {
                } else {
                    landmarkEstimates.insert(landmarkKey, priorLand);
                }

                // Add a prior for the landmark position to help with initial estimation.
                keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(
                    landmarkKey, priorLand, pointNoise)
                );
                }
                // Add a bearing-range observation for this landmark to the graph
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
                keyframeGraph_.add(factor);
                detectedLandmarksCurrentPos.insert(landmarkKey);
            }
        }
    }
    return detectedLandmarksCurrentPos;
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



void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    double current_time = ros::Time::now().toSec();

    // Convert the incoming odometry message to a simpler (x, y, theta) format
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);

    // Calculate distance and rotation change
    if (!movementExceedsThreshold(poseSE2)) return;

    // Increment pose index
    index_of_pose++;

    // Handle first pose initialization
    if (index_of_pose == 2) initializeFirstPose(poseSE2);

    // Predict next pose based on odometry
    gtsam::Pose2 predictedPose = predictNextPose(poseSE2);
    lastPose_ = predictedPose;

    // Check if this pose should be a keyframe and update graph
    if (shouldAddKeyframe(Key_previous_pos, predictedPose, oldlandmarks, detectedLandmarksCurrentPos) || !usekeyframe) {
        // Get camera detections and store them in a variable for future adjustments
        if (mCam_msg && rCam_msg && lCam_msg) {
            auto detections = getCamDetections(mCam_msg, rCam_msg, lCam_msg, mcam_baselink_transform, rcam_baselink_transform, lcam_baselink_transform);
            updateGraphWithLandmarks(poseSE2, predictedPose, detections);  // Update graph with landmarks
        }
        // Update the pose to landmarks mapping (for LC conditions)
        poseToLandmarks[gtsam::Symbol('X', index_of_pose)] = detectedLandmarksCurrentPos;

        // Update lastpose for next time step
        lastPoseSE2_ = poseSE2;
        
        // Optimize the graph after keyframe and landmark updates   
        optimizeGraph();  
    } else {
        updateOdometryPose(poseSE2);  // Update pose without adding a keyframe
    }

    // Update the last keyframe pose and symbol
    Key_previous_pos = predictedPose;
    previousKeyframeSymbol = currentKeyframeSymbol;

    // Publish updated results
    generate2bePublished()
    publishResults();
}