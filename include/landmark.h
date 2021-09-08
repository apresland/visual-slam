#ifndef VISUAL_SLAM_LANDMARK_H
#define VISUAL_SLAM_LANDMARK_H

#include <memory>
#include <mutex>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "feature.h"
#include "observation.h"

struct Landmark {

public:
    Landmark(){}
    Landmark(long id, cv::Point3f point_3d)
        : id_(id), point_3d_(point_3d) {}

    void set_position(cv::Point3f point_3d) {
        //std::unique_lock<std::mutex> lck(mutex_);
        point_3d_ = point_3d;
    }

    void add_observation(Observation observation) {
        //std::unique_lock<std::mutex> lck(mutex_);
        observations_.push_back(observation);
        observed_times_++;
    }

public:
    unsigned long id_;
    cv::Point3f point_3d_;
    //std::mutex mutex_;
    std::vector<Observation> observations_;
    int observed_times_ {0};
};

#endif //VISUAL_SLAM_LANDMARK_H
