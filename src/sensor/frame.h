#ifndef VISUAL_SLAM_FRAME_H
#define VISUAL_SLAM_FRAME_H

#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "sophus/se3.hpp"
#include "feature.h"

class Frame {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame(){};

    int getID() {
        return id_;
    }

    void setID(int id) {
        id_ = id;
    }

    bool getIsKeyframe() {
        return is_keyframe_;
    }

    void setIsKeyframe(bool isKeyframe) {
        is_keyframe_ = isKeyframe;
    }

    Sophus::SE3d getPose() {
        std::unique_lock<std::mutex> lck(mutex_);
        return pose_;
    }

    void setPose(const Sophus::SE3d &pose) {
        std::unique_lock<std::mutex> lck(mutex_);
        pose_ = pose;
    }

    std::vector<std::shared_ptr<MapPoint>> & getLandmarks(){
        std::unique_lock<std::mutex> lck(mutex_);
        return landmarks_;
    }

    std::vector<cv::Point2f> getPointsLeft() {
        return features2points(features_left_);
    }

    std::vector<cv::Point2f> getPointsRight() {
        return features2points(features_right_);
    }

    std::vector<cv::Point3f> getPoints3D() {
        return features2points3d(features_left_);
    }

private:

    std::vector<cv::Point2f> features2points(std::vector<std::shared_ptr<Feature>> features) {
        std::vector<cv::Point2f> points;
        for (auto &feature : features) {
            points.push_back(feature->getPoint2D());
        }
        return points;
    }

    std::vector<cv::Point3f> features2points3d(std::vector<std::shared_ptr<Feature>> features) {
        std::vector<cv::Point3f> points;
        for (auto &feature : features) {
            if(feature->getLandmark()) {
                auto mappoint = feature->getLandmark();
                points.push_back(mappoint->getPoint3D());
            } else {
                std::cout << "expired landmark" << std::endl;
            }
        }
        return points;
    }

public:
    cv::Mat image_left_;
    cv::Mat image_right_;
    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;
    std::vector<std::shared_ptr<MapPoint>> landmarks_;

private:
    int id_;
    bool is_keyframe_ {false};

    std::mutex mutex_;
    Sophus::SE3d pose_ {Sophus::SE3d()};

public:
    static std::shared_ptr<Frame> create(){
        return std::make_shared<Frame>();
    }
};

#endif //VISUAL_SLAM_FRAME_H
