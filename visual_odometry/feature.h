#ifndef VISUAL_SLAM_FEATURE_H
#define VISUAL_SLAM_FEATURE_H

#include <memory>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

struct Feature {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Feature(){}
    Feature(const cv::Point2f &point)
            : point_(point) {}

    cv::Point2f point_;
};

#endif //VISUAL_SLAM_FEATURE_H
