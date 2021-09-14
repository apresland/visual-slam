#ifndef VISUAL_SLAM_FRAME_H
#define VISUAL_SLAM_FRAME_H

#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "sophus/se3.hpp"
#include "feature.h"

struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame() {};

    Sophus::SE3d get_pose() {
        std::unique_lock<std::mutex> lck(mutex_);
        return pose_;
    }

    void set_pose(const Sophus::SE3d &pose) {
        std::unique_lock<std::mutex> lck(mutex_);
        pose_ = pose;
    }

    std::vector<cv::Point2f> get_points_left() {
        return features2points(features_left_);
    }

    std::vector<cv::Point2f> get_points_right() {
        return features2points(features_right_);
    }

    std::vector<cv::Point3f> get_points_3d() {
        return features2points3d(features_right_);
    }

private:

    std::vector<cv::Point2f> features2points(std::vector<std::shared_ptr<Feature>> features) {
        std::vector<cv::Point2f> points;
        for (auto &feature : features) {
            points.push_back(feature->point_2d_);
        }
        return points;
    }

    std::vector<cv::Point3f> features2points3d(std::vector<std::shared_ptr<Feature>> features) {
        std::vector<cv::Point3f> points;
        for (auto &feature : features) {
            points.push_back(feature->point_3d_);
        }
        return points;
    }

public:
    int id_;
    cv::Mat image_left_;
    cv::Mat image_right_;
    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;

    std::mutex mutex_;
    Sophus::SE3d pose_ {Sophus::SE3d()};
    bool is_keyframe_ {false};
};

#endif //VISUAL_SLAM_FRAME_H
