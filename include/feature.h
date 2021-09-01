#ifndef VISUAL_SLAM_FEATURE_H
#define VISUAL_SLAM_FEATURE_H

#include <memory>
#include <opencv2/opencv.hpp>

struct Frame;
struct Landmark;
struct Feature {
public:
    Feature(cv::Point2f point_2d) : point_2d_(point_2d) {}
public:
    cv::Point2f point_2d_;
    std::weak_ptr<Frame> frame_;
    std::weak_ptr<Landmark> landmark_;
    bool is_inlier_{false};
};

#endif //VISUAL_SLAM_FEATURE_H
