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
    Landmark() {};
    Landmark(Eigen::Matrix<double, 3, 1> position)
        : position_(position) {};

    void set_position(Eigen::Matrix<double, 3, 1> position) {
        std::unique_lock<std::mutex> lck(mutex_);
        position_ = position;
    };

    void add_observation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lck(mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

public:
    int id_;
    Eigen::Matrix<double, 3, 1> position_;
    std::mutex mutex_;
    std::vector<std::weak_ptr<Feature>> observations_;
    int observed_times_ {0};
};

#endif //VISUAL_SLAM_LANDMARK_H
