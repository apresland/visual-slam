//
// Created by andy on 8/8/21.
//

#ifndef VISUAL_SLAM_SEQUENCE_H
#define VISUAL_SLAM_SEQUENCE_H

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "frame.h"

class Sequence {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Sequence(std::string sequence_path);
    bool Init();
    std::vector<std::shared_ptr<Frame>> frames_;
private:
    void LoadFrames(std::string filepath);
    std::string loadImageLeft(int frame_index, std::string image_filepath);
    std::string loadImageRight(int frame_index, std::string image_filepath);
    std::string sequence_path_;
};

#endif //VISUAL_SLAM_SEQUENCE_H
