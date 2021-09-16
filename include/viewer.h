#ifndef VISUAL_SLAM_VIEWER_H
#define VISUAL_SLAM_VIEWER_H

#include <memory>
#include <fstream>
#include "sophus/se3.hpp"
#include "frame.h"

class Viewer {
public:
    Viewer();
    void init();
    void load_poses();
    void update(const std::shared_ptr<Frame> frame_previous,
                const std::shared_ptr<Frame> frame_current);
    void display_features(const std::shared_ptr<Frame> frame_current);
    void display_tracking(const std::shared_ptr<Frame> frame_previous,
                          const std::shared_ptr<Frame> frame_current);
    void display_trajectory(const std::shared_ptr<Frame> frame_current);

private:
    std::vector<cv::Mat> ground_truth_poses_;
    cv::Mat trajectory_ = cv::Mat::zeros(600, 1200, CV_8UC3);
};
#endif //VISUAL_SLAM_VIEWER_H
