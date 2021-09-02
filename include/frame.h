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

    void set_features_left(const std::vector<cv::Point2f> points) {
        points2features(points, features_left_);
    }

    void set_features_right(const std::vector<cv::Point2f> points) {
        points2features(points, features_right_);
    }

private:

    void points2features(const std::vector<cv::Point2f> points, std::vector<std::shared_ptr<Feature>> &features) {
        features.clear();
        for(auto &p : points) {
            features.push_back(std::make_shared<Feature>(p));
        }
    }

    std::vector<cv::Point2f> features2points(std::vector<std::shared_ptr<Feature>> features) {
        std::vector<cv::Point2f> points;
        for (auto &feature : features) {
            points.push_back(feature->point_2d_);
        }
        return points;
    }

public:
    int id_;
    cv::Mat image_left_;
    cv::Mat image_right_;
    //std::vector<cv::Point2f> points_left_;
    //std::vector<cv::Point2f> points_right_;
    std::vector<cv::Point3f> points_3d_;
    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;

    std::mutex mutex_;
    Sophus::SE3d pose_ {Sophus::SE3d()};
    bool is_keyframe_ {false};
};

#endif //VISUAL_SLAM_FRAME_H
