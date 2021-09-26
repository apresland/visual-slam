#ifndef VISUAL_SLAM_FEATURE_H
#define VISUAL_SLAM_FEATURE_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "mappoint.h"

struct Frame;
struct MapPoint;
class Feature {
public:
    Feature() = delete;
    Feature(std::shared_ptr<Frame> frame, cv::Point2f point2d)
            : frame_(frame), point_2d_(point2d) { }

    cv::Point2f getPoint2D() {
        return point_2d_;
    }

    void setLandmark(std::shared_ptr<MapPoint> landmark) {
        landmark_ = landmark;
    }

    std::shared_ptr<MapPoint> getLandmark() {
        return landmark_;
    }

    std::shared_ptr<Frame> getFrame() {
        return frame_;
    }

    void isInlier(bool  is_inlier) {
        is_inlier_ = is_inlier;
    }

    bool getIsInlier() {
        return is_inlier_;
    }

private:

    int id_{-1};
    std::shared_ptr<Frame> frame_;
    std::shared_ptr<MapPoint> landmark_{nullptr};
    cv::Point2f point_2d_;
    bool is_inlier_{false};

public:
    static std::shared_ptr<Feature> create(std::shared_ptr<Frame> frame, cv::Point2f point2d) {
        return std::make_shared<Feature>(frame, point2d);
    }
};

#endif //VISUAL_SLAM_FEATURE_H
