#ifndef VISUAL_SLAM_POSE_ESTIMATION_H
#define VISUAL_SLAM_POSE_ESTIMATION_H

#include <memory>
#include <opencv2/opencv.hpp>

struct Frame;
class Estimation {
public:
    Estimation() {}
    void estimate(std::shared_ptr<Frame> frame_previous,
                       std::shared_ptr<Frame> frame_current,
                       const cv::Mat K);
    void removeOutliers(std::shared_ptr<Frame> frame, cv::Mat inliers);
};

#endif //VISUAL_SLAM_POSE_ESTIMATION_H
