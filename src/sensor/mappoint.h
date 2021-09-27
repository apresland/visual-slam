#ifndef VISUAL_SLAM_MAPPOINT_H
#define VISUAL_SLAM_MAPPOINT_H

#include <memory>
#include <mutex>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "feature.h"

struct Feature;
class MapPoint {
public:
    MapPoint(){}
    MapPoint(long id, cv::Point3f point_3d)
        : id_(id), point_3d_(point_3d) {

    }

    unsigned long getID() {
        return id_;
    }

    void setID(unsigned long id) {
        id_ = id;
    }

    int getObservedTimes() {
        return observed_times_;
    }

    void setPoint3D(cv::Point3f point_3d) {
        std::unique_lock<std::mutex> lck(mutex_);
        point_3d_ = point_3d;
    }

    cv::Point3f& getPoint3D() {
        std::unique_lock<std::mutex> lck(mutex_);
        return point_3d_;
    }

    void addObservation(std::shared_ptr<Feature> observation) {
        std::unique_lock<std::mutex> lck(mutex_);
        observations_.push_back(observation);
        observed_times_++;
    }

    void removeObservation(std::shared_ptr<Feature> feat);

    std::vector<std::shared_ptr<Feature>> getObservations() {
        std::unique_lock<std::mutex> lck(mutex_);
        return observations_;
    }

private:
    unsigned long id_;
    cv::Point3f point_3d_;
    std::mutex mutex_;
    std::vector<std::shared_ptr<Feature>> observations_;
    int observed_times_ {0};
};

#endif //VISUAL_SLAM_MAPPOINT_H
