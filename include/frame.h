#ifndef VISUAL_SLAM_FRAME_H
#define VISUAL_SLAM_FRAME_H

#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "sophus/se3.hpp"

struct Feature;
struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame() {};

    Sophus::SE3d Pose() {
        std::unique_lock<std::mutex> lck(mutex_);
        return pose_;
    }

    void SetPose(const Sophus::SE3d &pose) {
        std::unique_lock<std::mutex> lck(mutex_);
        pose_ = pose;
    }

public:
    int id_;
    cv::Mat image_left_;
    cv::Mat image_right_;
    std::vector<cv::Point2f> points_left_;
    std::vector<cv::Point2f> points_right_;
    std::vector<cv::Point3f> points_3d_;
    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;

    std::mutex mutex_;
    Sophus::SE3d pose_ {Sophus::SE3d()};
    bool is_keyframe_ {false};
};

#endif //VISUAL_SLAM_FRAME_H
