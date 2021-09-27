#ifndef VISUAL_SLAM_VIEWER_H
#define VISUAL_SLAM_VIEWER_H

#include <memory>
#include <fstream>
#include "sophus/se3.hpp"
#include "sensor/frame.h"
#include "context.h"

class Viewer {
public:
    Viewer();
    void init();
    void loadGroundTruthPoses();
    void displayFeatures(const Context &context);
    void displayTracking(const Context &context);
    void displayTrajectory(const Context &context);

private:
    std::vector<cv::Mat> ground_truth_poses_;
    cv::Mat trajectory_ = cv::Mat::zeros(600, 1200, CV_8UC3);
    std::shared_ptr<Frame> frame_previous{nullptr};
};
#endif //VISUAL_SLAM_VIEWER_H