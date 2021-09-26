//
// Created by andy on 8/9/21.
//

#ifndef VISUAL_SLAM_SYSTEM_H
#define VISUAL_SLAM_SYSTEM_H

#include <memory>
#include <Eigen/Core>
#include "sequence.h"
#include "sensor/camera.h"

class System {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    System(std::string path_to_sequence);
    bool Init();
    void Run();
private:
    std::string path_root_;
    std::string calibration_file_;
    std::shared_ptr<Sequence> sequence_ = nullptr;
    std::vector<std::shared_ptr<Camera>> cameras_;
    bool initialized_ = false;
};

#endif //VISUAL_SLAM_SYSTEM_H
