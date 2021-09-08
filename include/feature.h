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
    int landmark_id_{-1};
    cv::Point2f point_2d_;
    cv::Point3f point_3d_;
    std::shared_ptr<Frame> frame_;
    std::shared_ptr<Landmark> landmark_;
    bool is_inlier_{false};
};

#endif //VISUAL_SLAM_FEATURE_H
