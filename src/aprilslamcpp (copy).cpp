void aprilslamcpp::checkLoopClosure(double current_time, const std::set<gtsam::Symbol>& detectedLandmarks) {
    if (loopClosureEnableFlag) {
        int reobservedLandmarks = 0;
        int count = 0;

        // Compare current pose's detected landmarks with previous poses' detected landmarks
        for (const auto& poseLandmarks : poseToLandmarks) {
            std::set<gtsam::Symbol> intersection;
            std::set_intersection(detectedLandmarks.begin(), detectedLandmarks.end(),
                                  poseLandmarks.second.begin(), poseLandmarks.second.end(),
                                  std::inserter(intersection, intersection.begin()));

            reobservedLandmarks = intersection.size();
            if (reobservedLandmarks >= 3) {
                break;
            }
        }

        // Proceed with loop closure only if at least three landmarks have been re-detected
        if (reobservedLandmarks >= 3) {
            for (int i = 1; i < index_of_pose && count < historyKeyframeSearchNum; ++i) {
                gtsam::Symbol poseSymbol('X', i);
                if (landmarkEstimates.exists(poseSymbol)) {
                    gtsam::Pose2 previousPose = landmarkEstimates.at<gtsam::Pose2>(poseSymbol);
                    double distance = lastPose_.range(previousPose);
                    double timeDiff = current_time - factorTimestamps_[i];

                    // Check if the previous pose is within the search radius and time difference
                    if (distance <= historyKeyframeSearchRadius && timeDiff >= historyKeyframeSearchTimeDiff) {
                        // Add loop closure constraint
                        graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(poseSymbol, gtsam::Symbol('X', index_of_pose), relPoseFG(previousPose, lastPoseSE2_), odometryNoise));
                        factorTimestamps_[graph_.size() - 1] = current_time;
                        count++;
                    }
                }
            }
        }
    }
}

void aprilslamcpp::checkLoopClosure(double current_time, const std::set<gtsam::Symbol>& detectedLandmarks) {
    if (loopClosureEnableFlag) {
        double spatialThreshold = 2.0; // meters
        double temporalThreshold = 30.0; // seconds
        int requiredReobservedLandmarks = 3; // Minimum number of re-detected landmarks to trigger loop closure

        for (const auto& keyframe : keyframes) {
            gtsam::Symbol keyframeSymbol = keyframe.first;
            gtsam::Pose2 keyframePose = keyframe.second;
            double keyframeTime = keyframeTimestamps[keyframeSymbol];

            double distance = lastPose_.range(keyframePose);
            double timeDiff = current_time - keyframeTime;

            if (distance < spatialThreshold && timeDiff > temporalThreshold) {
                // Count the number of re-detected landmarks in this keyframe
                std::set<gtsam::Symbol> intersection;
                std::set_intersection(detectedLandmarks.begin(), detectedLandmarks.end(),
                                      poseToLandmarks[keyframeSymbol].begin(), poseToLandmarks[keyframeSymbol].end(),
                                      std::inserter(intersection, intersection.begin()));

                int reobservedLandmarks = intersection.size();
                if (reobservedLandmarks >= requiredReobservedLandmarks) {
                    // Add loop closure constraint
                    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(keyframeSymbol, gtsam::Symbol('X', index_of_pose), relPoseFG(keyframePose, lastPoseSE2_), odometryNoise));
                    factorTimestamps_[graph_.size() - 1] = current_time;
                    break;
                }
            }
        }
    }
}



void aprilslamcpp::checkLoopClosure(const std::map<int, int>& landmarkCount, double current_time) {
    if (loopClosureEnableFlag) {
        int reobservedLandmarks = 0;
        int count = 0;

        // Check how many landmarks have been re-detected
        for (const auto& landmark : landmarkCount) {
            if (landmark.second > 1) {
                reobservedLandmarks++;
            }
        }

        // Proceed with loop closure only if at least three landmarks have been re-detected
        if (reobservedLandmarks >= 3) {
            for (int i = 1; i < index_of_pose && count < historyKeyframeSearchNum; ++i) {
                if (landmarkEstimates.exists(gtsam::Symbol('X', i))) {
                    gtsam::Pose2 previousPose = landmarkEstimates.at<gtsam::Pose2>(gtsam::Symbol('X', i));
                    double distance = lastPose_.range(previousPose);
                    double timeDiff = current_time - factorTimestamps_[i];

                    // Check if the previous pose is within the search radius and time difference
                    if (distance <= historyKeyframeSearchRadius && timeDiff >= historyKeyframeSearchTimeDiff) {
                        // Add loop closure constraint
                        graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', i), gtsam::Symbol('X', index_of_pose), relPoseFG(previousPose, lastPoseSE2_), odometryNoise));
                        factorTimestamps_[graph_.size() - 1] = current_time;
                        count++;
                    }
                }
            }
        }
    }
}