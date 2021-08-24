//
// Created by andy on 8/9/21.
//

#ifndef VISUAL_SLAM_ODOMETRY_H
#define VISUAL_SLAM_ODOMETRY_H

#include <memory>
#include <Eigen/Core>
#include "sequence.h"
#include "camera.h"

class Odometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Odometry(std::string path_to_sequence);
    bool Init();
    void Run();
private:
    bool init_cameras();

    bool initialized_ = false;
    std::string path_to_sequence_;
    std::shared_ptr<Sequence> sequence_ = nullptr;
    std::shared_ptr<Frame> prev_frame_ = nullptr;
    std::chrono::time_point<std::chrono::steady_clock> prev_time_point_;
    std::vector<std::shared_ptr<Camera>> cameras_;
    double prev_timestamp_ = 0;
};

#endif //VISUAL_SLAM_ODOMETRY_H
