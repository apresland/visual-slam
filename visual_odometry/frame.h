//
// Created by andy on 8/8/21.
//

#ifndef VISUAL_SLAM_FRAME_H
#define VISUAL_SLAM_FRAME_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame();
    std::string image_left_;
    std::string image_right_;
    double timestamp_;
};


#endif //VISUAL_SLAM_FRAME_H
