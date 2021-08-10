#ifndef VISUAL_SLAM_FRAME_H
#define VISUAL_SLAM_FRAME_H

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include "feature.h"

struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame(){};
    cv::Mat image_left_;
    cv::Mat image_right_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    std::vector<cv::DMatch> matches_;
    double timestamp_;
};


#endif //VISUAL_SLAM_FRAME_H
