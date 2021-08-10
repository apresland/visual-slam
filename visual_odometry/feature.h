//
// Created by andy on 8/9/21.
//

#ifndef VISUAL_SLAM_FEATURE_H
#define VISUAL_SLAM_FEATURE_H

#include <memory>

struct Frame;

struct Feature {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Feature(){}
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;
    cv::KeyPoint position_;
};

#endif //VISUAL_SLAM_FEATURE_H
