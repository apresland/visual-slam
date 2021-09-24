#ifndef VISUAL_SLAM_MAPPOINT_H
#define VISUAL_SLAM_MAPPOINT_H

#include <memory>
#include <mutex>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "feature.h"

struct Feature;
struct MapPoint {

public:
    MapPoint(){}
    MapPoint(long id, cv::Point3f point_3d)
        : id_(id), point_3d_(point_3d) {}

    void set_position(cv::Point3f point_3d) {
        std::unique_lock<std::mutex> lck(mutex_);
        point_3d_ = point_3d;
    }

    void add_observation(std::shared_ptr<Feature> observation) {
        std::unique_lock<std::mutex> lck(mutex_);
        observations_.push_back(observation);
        observed_times_++;
    }

    std::vector<std::weak_ptr<Feature>> observations() {
        std::unique_lock<std::mutex> lck(mutex_);
        return observations_;
    }

    void remove_observation(std::shared_ptr<Feature> feat);

public:
    unsigned long id_;
    cv::Point3f point_3d_;
    std::mutex mutex_;
    std::vector<std::weak_ptr<Feature>> observations_;
    int observed_times_ {0};
};

#endif //VISUAL_SLAM_MAPPOINT_H
