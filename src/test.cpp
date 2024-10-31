// Update the graph with landmarks detections
std::set<gtsam::Symbol> aprilslam::aprilslamcpp::updateGraphWithLandmarks(
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos, 
    const std::pair<std::vector<int>, std::vector<Eigen::Vector2d>>& detections) {

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
            }
            // Store the bearing and range measurements in the map
            poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = std::make_tuple(bearing, range); 
        }
    }
    return detectedLandmarksCurrentPos;
}