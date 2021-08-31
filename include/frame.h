#ifndef VISUAL_SLAM_FRAME_H
#define VISUAL_SLAM_FRAME_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "sophus/se3.hpp"

struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame() {};
public:
    int id_;
    cv::Mat image_left_;
    cv::Mat image_right_;
    std::vector<cv::Point2f> points_left_;
    std::vector<cv::Point2f> points_right_;
    cv::Mat points3D_;

    Sophus::SE3d T_c_w_ {Sophus::SE3d()};
    bool is_keyframe_ {false};
};

#endif //VISUAL_SLAM_FRAME_H
