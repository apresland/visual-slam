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
    int id_{-1};
    int frame_id_{-1};
    int landmark_id_{-1};
    std::shared_ptr<Frame> frame_{nullptr};
    std::shared_ptr<Landmark> landmark_{nullptr};
    cv::Point2f point_2d_;
    cv::Point3f point_3d_;
    bool is_inlier_{false};
};

#endif //VISUAL_SLAM_FEATURE_H
