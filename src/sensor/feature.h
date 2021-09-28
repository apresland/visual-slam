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
    Feature(int frame_id, cv::Point2f point2d)
            : frame_id_(frame_id), point_2d_(point2d) { }

    cv::Point2f getPoint2D() {
        return point_2d_;
    }

    void setLandmark(std::shared_ptr<MapPoint> landmark) {
        landmark_ = landmark;
    }

    std::shared_ptr<MapPoint> getLandmark() {
        return landmark_;
    }

    int getFrameID() {
        return frame_id_;
    }

    void isInlier(bool  is_inlier) {
        is_inlier_ = is_inlier;
    }

    bool getIsInlier() {
        return is_inlier_;
    }

private:

    int id_{-1};
    int frame_id_{-1};
    std::shared_ptr<MapPoint> landmark_{nullptr};
    cv::Point2f point_2d_;
    bool is_inlier_{false};

public:
    static std::shared_ptr<Feature> create(int frame_id, cv::Point2f point2d) {
        return std::make_shared<Feature>(frame_id, point2d);
    }
};

#endif //VISUAL_SLAM_FEATURE_H
