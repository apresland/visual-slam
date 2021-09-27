#ifndef VISUAL_SLAM_POSE_ESTIMATION_H
#define VISUAL_SLAM_POSE_ESTIMATION_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "context.h"

struct Frame;
class Estimation {
public:
    Estimation() {}
    void estimate(Context &context, const cv::Mat K);
    void removeOutliers(Context &context, cv::Mat inliers);
};

#endif //VISUAL_SLAM_POSE_ESTIMATION_H
