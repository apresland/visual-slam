#ifndef VISUAL_SLAM_FEATURE_H
#define VISUAL_SLAM_FEATURE_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "mappoint.h"

struct Frame;
struct MapPoint;
struct Feature {
public:
    Feature(std::shared_ptr<Frame> frame, cv::Point2f point_2d)
        : frame_(frame), point_2d_(point_2d) {}
    Feature(std::shared_ptr<Frame> frame, cv::Point2f point_2d, cv::Point3f point_3d)
            : frame_(frame), point_2d_(point_2d) {}
public:
    int id_{-1};
    int frame_id_{-1};
    int landmark_id_{-1};
    int age_{0};
    std::shared_ptr<Frame> frame_;
    std::shared_ptr<MapPoint> landmark_;
    cv::Point2f point_2d_;
    //cv::Point3f point_3d_;
    bool is_inlier_{false};
};

#endif //VISUAL_SLAM_FEATURE_H
